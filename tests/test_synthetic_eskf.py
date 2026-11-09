from types import SimpleNamespace
import numpy as np
from gnc.navigation.synthetic_eskf import SyntheticESKF, rot, qnorm


def test_rotation_identity():
    assert np.allclose(rot([1,0,0,0]),np.eye(3))


def test_stationary_navigation_and_dropout():
    nav=SyntheticESKF(accel_bias=(0,0,0),gyro_bias_deg=(0,0,0),
       gnss_pos_sigma=2.5,gnss_vel_sigma=0.15,gnss_dropout=0,radar_sigma=0.5)
    mem={}
    for i in range(101):
        t=i*0.02
        # rigid body at rest; a=0, so accelerometer must measure -g in NED
        state=SimpleNamespace(t=t,r_ned=np.array([0.,0.,-100.]),
          v_ned=np.zeros(3),q_bn=np.array([1.,0.,0.,0.]),w_body=np.zeros(3),m=40000.)
        obs=nav.measure(state,np.array([12,34],dtype=np.uint32))
        est,mem=nav.estimate(obs,None,mem)
    assert np.linalg.norm(est.r_ned-np.array([0,0,-100]))<3.0
    assert np.linalg.norm(est.v_ned)<0.5
    assert np.isclose(np.linalg.norm(est.q_bn),1)
    assert mem['P'].shape==(15,15)
    assert np.all(np.linalg.eigvalsh(mem['P']) >= -1e-8)


def test_gnss_outage():
    nav=SyntheticESKF(gnss_dropout=0,gnss_outage_start=1.,gnss_outage_end=2.)
    state=SimpleNamespace(t=1.5,r_ned=np.array([0.,0.,-2000.]),v_ned=np.zeros(3),
       q_bn=np.array([1.,0.,0.,0.]),w_body=np.zeros(3),m=40000.)
    obs=nav.measure(state,[1,2])
    assert not obs.gnss_valid
    assert not obs.radar_valid
