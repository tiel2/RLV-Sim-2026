"""Educational 15-error-state strapdown INS + GNSS/radar ESKF.

Drop-in RLV-Sim navigation module. Sensor sample rate is scenario.horizon.nav_dt;
GNSS/radar are time-gated. Measurement.r_ned/v_ned are noisy GNSS when
available (NaN otherwise); q_bn/w_body are compatibility logging fields.
The estimator NEVER consumes measured q_bn or any truth fields other than the
synthetic inertial/GNSS/radar channels, except mass (perfect mass gauge).

IMU force is synthesized by differentiating *truth velocity at nav ticks*;
this is a sampled approximation, not a high-rate native accelerometer model.
A previous truth state is held only in sensor generation, never in the filter.
For a production-quality sensor, use plant force/acceleration diagnostics.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from core.interfaces import Measurement


def skew(v):
    x,y,z = v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]], float)


def qnorm(q):
    a = np.asarray(q, float)
    return a / np.linalg.norm(a)


def qmul(a,b):
    w,x,y,z=a; W,X,Y,Z=b
    return np.array([w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
                     w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W])


def dq_body(dtheta):
    angle=np.linalg.norm(dtheta)
    if angle<1e-12:
        return qnorm(np.r_[1.0, 0.5*np.asarray(dtheta)])
    axis=np.asarray(dtheta)/angle
    return np.r_[np.cos(angle/2), np.sin(angle/2)*axis]


def rot(q):
    w,x,y,z=qnorm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


@dataclass(frozen=True)
class SensorMeasurement(Measurement):
    accel_body: object = None  # specific force, m/s^2
    gyro_body: object = None   # angular rate, rad/s
    gnss_valid: bool = False
    radar_alt_m: float = float('nan')
    radar_valid: bool = False


class SyntheticESKF:
    def __init__(self, *, imu_accel_sigma=0.08, imu_gyro_sigma_deg=0.025,
                 accel_bias=(0.025,-0.02,0.035), gyro_bias_deg=(0.005,-0.006,0.004),
                 gnss_pos_sigma=2.5, gnss_vel_sigma=0.15, gnss_dropout=0.03,
                 gnss_period=0.1, radar_sigma=0.5, radar_period=0.05,
                 radar_range=1500.0, gnss_outage_start=-1.0,
                 gnss_outage_end=-1.0, seed_offset=0):
        self.sa=float(imu_accel_sigma)
        self.sg=float(np.deg2rad(imu_gyro_sigma_deg))
        self.ba=np.asarray(accel_bias,float)
        self.bg=np.deg2rad(np.asarray(gyro_bias_deg,float))
        self.sp=float(gnss_pos_sigma)
        self.sv=float(gnss_vel_sigma)
        self.drop=float(gnss_dropout)
        self.gperiod=float(gnss_period)
        self.srad=float(radar_sigma)
        self.rperiod=float(radar_period)
        self.rrange=float(radar_range)
        self.outage0=float(gnss_outage_start)
        self.outage1=float(gnss_outage_end)
        self.seed_offset=int(seed_offset)
        self._prev_truth=None  # sensor generation only

    def initial_memory(self):
        return {}  # independent EKF state per episode

    def _rng(self, seed, t, channel):
        # keyed by epoch and channel; deterministic per seed, independent streams
        key = np.asarray(seed).reshape(-1)
        ints=[int(v) for v in key]
        ss=np.random.SeedSequence([*(x & 0xffffffff for x in ints),
                                   int(round(t*1e6)) & 0xffffffff,
                                   int(channel), self.seed_offset & 0xffffffff])
        return np.random.default_rng(ss)

    @staticmethod
    def _due(t, period):
        return abs(t/period - round(t/period)) < 1e-5

    def measure(self, state, seed):
        t=float(state.t)
        p=np.asarray(state.r_ned,float)
        v=np.asarray(state.v_ned,float)
        q=qnorm(state.q_bn)
        w=np.asarray(state.w_body,float)
        # approximate gravity in fixed-origin NED; must match INS mechanization
        h=max(-p[2],0.0)
        g=9.80665*(6371000.0/(6371000.0+h))**2
        g_n=np.array([0.0,0.0,g])
        if self._prev_truth is None or t <= self._prev_truth[0]:
            a_n=g_n.copy()  # first sample cannot infer derivative
        else:
            tprev,vprev=self._prev_truth
            a_n=(v-vprev)/(t-tprev)
        self._prev_truth=(t,v.copy())
        accel=rot(q).T@(a_n-g_n)+self.ba + self.sa*self._rng(seed,t,1).standard_normal(3)
        gyro=w+self.bg + self.sg*self._rng(seed,t,2).standard_normal(3)
        gps_on=self._due(t,self.gperiod)
        gps_on &= not (self.outage0 <= t <= self.outage1 and self.outage1>self.outage0)
        if gps_on:
            gps_on=self._rng(seed,t,3).random() >= self.drop
        gp=p+self.sp*self._rng(seed,t,4).standard_normal(3) if gps_on else np.full(3,np.nan)
        gv=v+self.sv*self._rng(seed,t,5).standard_normal(3) if gps_on else np.full(3,np.nan)
        radar_on=self._due(t,self.rperiod) and 0<=-p[2]<=self.rrange
        ra=-p[2]+self.srad*self._rng(seed,t,6).standard_normal() if radar_on else float('nan')
        # Compatibility fields for default navigation log; q and w here represent
        # gyro-integrated orientation only after estimate, not a measured quaternion.
        return SensorMeasurement(r_ned=gp,v_ned=gv,q_bn=q,w_body=gyro,m=float(state.m),t=t,
                                 accel_body=accel,gyro_body=gyro,gnss_valid=bool(gps_on),
                                 radar_alt_m=float(ra),radar_valid=bool(radar_on))

    @staticmethod
    def _update(mem,z,h,H,R):
        P=mem['P']; S=H@P@H.T+R
        K=np.linalg.solve(S,(P@H.T).T).T
        dx=K@(z-h)
        mem['r']+=dx[:3]; mem['v']+=dx[3:6]
        # attitude error uses navigation-frame small-angle rotation
        dtheta=dx[6:9]
        mem['q']=qnorm(qmul(dq_body(dtheta),mem['q']))
        mem['bg']+=dx[9:12];mem['ba']+=dx[12:15]
        I=np.eye(15); A=I-K@H
        mem['P']=0.5*((A@P@A.T+K@R@K.T)+(A@P@A.T+K@R@K.T).T)

    def estimate(self, meas, act, memory):
        mem=dict(memory)
        t=float(meas.t)
        if 'P' not in mem:
            # Initial alignment assumed known; GNSS initializes r,v when available.
            mem.update(r=np.nan_to_num(np.asarray(meas.r_ned,float)),
                       v=np.nan_to_num(np.asarray(meas.v_ned,float)),
                       q=qnorm(meas.q_bn), bg=np.zeros(3),ba=np.zeros(3),
                       P=np.diag([25.0]*3+[4.0]*3+[np.deg2rad(1)**2]*3+
                                 [np.deg2rad(0.05)**2]*3+[0.1**2]*3),t=t)
        else:
            dt=t-mem['t']
            if dt <= 0: raise ValueError('navigation timestamps must increase')
            w=np.asarray(meas.gyro_body)-mem['bg']
            f=np.asarray(meas.accel_body)-mem['ba']
            Rbn=rot(mem['q'])
            h=max(-mem['r'][2],0)
            g=np.array([0.0,0.0,9.80665*(6371000/(6371000+h))**2])
            a=Rbn@f+g
            mem['r']+=mem['v']*dt+0.5*a*dt*dt
            mem['v']+=a*dt
            mem['q']=qnorm(qmul(mem['q'],dq_body(w*dt)))
            F=np.zeros((15,15))
            F[:3,3:6]=np.eye(3)
            F[3:6,6:9]=-skew(Rbn@f)
            F[3:6,12:15]=-Rbn
            F[6:9,9:12]=-Rbn
            Phi=np.eye(15)+F*dt
            # Continuous noise PSD approximately sample-noise^2 * nav interval.
            Q=np.zeros((15,15))
            Q[3:6,3:6]=np.eye(3)*self.sa**2*dt*dt
            Q[6:9,6:9]=np.eye(3)*self.sg**2*dt*dt
            Q[9:12,9:12]=np.eye(3)*np.deg2rad(0.0005)**2*dt
            Q[12:15,12:15]=np.eye(3)*0.0005**2*dt
            mem['P']=Phi@mem['P']@Phi.T+Q
            mem['t']=t
        if meas.gnss_valid:
            H=np.zeros((6,15));H[:6,:6]=np.eye(6)
            z=np.r_[meas.r_ned,meas.v_ned]
            pred=np.r_[mem['r'],mem['v']]
            rr=np.diag([self.sp**2]*3+[self.sv**2]*3)
            self._update(mem,z,pred,H,rr)
        if meas.radar_valid:
            H=np.zeros((1,15));H[0,2]=-1
            self._update(mem,np.array([meas.radar_alt_m]),np.array([-mem['r'][2]]),
                         H,np.array([[self.srad**2]]))
        # mass remains truth: no propellant-gauge model yet; avoid falsely claiming full sensor nav
        estimate=Measurement(r_ned=mem['r'].copy(),v_ned=mem['v'].copy(),
                             q_bn=mem['q'].copy(),w_body=np.asarray(meas.gyro_body)-mem['bg'],
                             m=meas.m,t=t)
        return estimate,mem


def build(ctx):
    cfg=ctx.cfg
    kwargs={k: cfg[k] for k in (
        'imu_accel_sigma','imu_gyro_sigma_deg','accel_bias','gyro_bias_deg',
        'gnss_pos_sigma','gnss_vel_sigma','gnss_dropout','gnss_period',
        'radar_sigma','radar_period','radar_range','gnss_outage_start',
        'gnss_outage_end','seed_offset') if k in cfg}
    return SyntheticESKF(**kwargs)
