# Thread E 비판 리뷰 — GP-MPC · conformal prediction · learning-flight · density-MPC (19편 병합·재교정)

> 스레드 E = GP-residual→Σ→chance 파이프라인의 기초·calibration·실시간 구현·대기밀도 외란 보정. 도메인은 quadrotor/통계라 점수와 분리해 *방법 재사용성*으로만 판단한다.
> 재교정: E1~E3 part 점수를 한 앵커로 통일했다. 기존 part는 E1의 top-ML 논문(ACI 88, ICML 85)이 venue 후광으로 후했고, E3 일부(zoRO·Dual-GP)는 C-점수보다 Total이 높게 적혀 있었다 — 둘 다 같은 엄격 앵커·동일 가중으로 다시 계산했다.
> 공정성: top venue(NeurIPS/ICML/Ann.Stat./Sci Robot)라고 자동 가점하지 않는다(venue≠품질). 80+는 분야창시/정전급이 C-기준으로 실제 받쳐줄 때만. C4(위험모델 충실도)가 이 스레드 핵심: GP variance를 그냥 쓰면 3, conformal/sharp-calibration으로 진짜 보정하면 4, variance를 아예 안 쓰면 2; conformal 커버리지가 marginal/장기평균이면 C4 상한 4.

---

## (2020, IEEE TCST, Cautious Model Predictive Control Using Gaussian Process Regression: Hewing et al.)

**1. 핵심 아이디어.** 알려진 nominal 동역학에 데이터로 학습한 가우시안 프로세스(GP) 잔차 모델을 더한 hybrid 모델 구조 위에서, GP가 제공하는 잔차 불확실성을 chance constraint로 명시적으로 다루는 "cautious" MPC를 제안한다. 상태 분포를 시간 전파하고 확률적 도달집합(PRS) 기반 제약 조임으로 안전을 확보하며, MPC에 특화된 dynamic sparse GP 근사로 수 ms 샘플링에서도 연산을 가능케 한다. AUV 시뮬레이션과 실제 RC 자율주행 레이싱(20ms)에서 성능과 안전을 동시에 개선함을 보인다.

**2. 알고리즘.** 모델: x(k+1)=f(x,u)+B_d(g(x,u)+w(k)), g를 출력차원별 GP로 학습(squared-exponential 커널, 식 2/4). 불확실성 전파는 nominal f와 GP 평균 μ^d에 1차 Taylor 근사를 적용한 EKF형 공분산 업데이트: μ^x_{i+1}=f(μ)+B_d μ^d(·), Σ^x_{i+1}=Ã_i Σ^x_i Ã_i^T + B_d(Σ^d+Σ^w)B_d^T (식 8). 피드백 정책 π_i=K_i(μ^x_i−x_i)+μ^u_i로 예측 분산을 줄인다. Chance constraint는 i-step PRS R_i(Σ^x_i)로 정의되어 half-space는 z: h^T z ≤ b − φ^{-1}(p)·sqrt(h^T Σ h), two-norm ball은 χ_n^2(p) 분위수 기반으로 조임(식 9/10). 결과적으로 식 (11)의 deterministic nonconvex NLP를 SQP/내부점(FORCES Pro)으로 receding-horizon 해결. Dynamic sparse GP는 이전 MPC 궤적 X̄,Ū를 따라 inducing point를 배치하는 transductive 국소 근사.

**3. 비판적 검토.**
- 핵심 전파가 1차 Taylor + jointly-Gaussian 근사이며, 저자도 "strict guarantee를 잃는다"고 명시한다. 따라서 chance constraint 만족은 근사적 보장이고, 재귀적 실행가능성·안정성에 대한 형식적 정리는 없다.
- baseline이 nominal 컨트롤러와 linear stochastic MPC로 모두 내부(self) 비교다. 외부 경쟁 학습기반 MPC와의 정량 비교나 Monte-Carlo 통계적 평가가 없어 C5가 제한된다.
- GP 평균의 lateral velocity 예측을 신뢰도 문제로 생략하는 등 실제 적용에서 임시적 단순화가 있고, 이는 제안 프레임워크의 완결성보다 구현 휴리스틱에 의존함을 보여준다.
- 하이퍼파라미터(커널 길이척도/분산), 데이터 갱신 정책 등은 상세히 주어졌으나 코드/데이터는 공개되지 않았다.

**4. 시뮬/실험.** (sim) AUV 깊이제어: pitch reference 30°→0→45° 추종, 2.28%(2σ) 위반확률 chance constraint, 온라인으로 30개 데이터 학습. GP-MPC가 linear stochastic MPC 대비 최소 pitch 각 제약 위반을 막고 보수성을 줄임. (real) 1:43 스케일 RC car 자율주행 레이싱, 20ms 샘플링, EKF+적외선 카메라 상태추정, 20랩. Table I: GP-based가 nominal 대비 평균 랩타임 10.32→9.61s(약 7%↑), 동역학 오차 ||e|| 0.65→0.34, 제약 안전성 개선, T_c<20ms 비율 76.3%→99.8%. 10개 inducing point sparse 근사 사용 시 full GP 대비 예측오차 증가 약 3%.

**5. Contribution(delta).** 출판년(2020) 기준, GP 잔차 모델의 불확실성을 PRS 기반 chance constraint로 원리적으로 다루면서 동시에 MPC 전용 dynamic sparse GP로 연산을 줄여, 이 복잡도의 시스템을 20ms 샘플링으로 실제 하드웨어에서 구동한 최초의 GP-MPC 구현을 보였다는 점이 핵심 delta다. 선행 GP-MPC([2][5][6])는 느리고 작은 시스템에 국한됐고 stochastic reachable set([10])은 선형계였는데, 이를 nominal+GP hybrid·고속·실차에 통합한 것이 진전이다.

**6. Limitation·Future work.** 저자: 평균/분산 전파의 strict guarantee 부재, sparse 근사의 trajectory 의존성, lateral velocity 예측 생략. 내가 보는 한계: 형식적 안정성/실행가능성 보장 부재, 외부 baseline·MC 통계 부재, robotics(수중차·RC카) 검증으로 항공우주 G&C로의 직접 전이는 간접적(고속 entry/descent 동역학·열·공력 잔차 환경에서의 적용성은 미검증). Future work으로는 strict probabilistic guarantee 회복, 더 높은 차원/빠른 동역학에서의 sparse GP 확장이 자연스럽다.

**평가:** 84/100 (strong) · 한줄평: GP 잔차 불확실성을 chance constraint로 원리적이면서 실차 20ms로 구동한 영향력 큰 학습기반 MPC 표준 참조 — 다만 검증은 로보틱스 도메인으로 항공우주 G&C 전이는 간접적.

---

## (2021, RA-L, Data-Driven MPC for Quadrotors: Torrente et al.)

**1. 핵심 아이디어.** 고속·고가속(최대 14 m/s, 4g 이상) 비행에서 공기역학 효과(로터 드래그, 동체 드래그, 난류)는 모델링이 어려워 추적 오차의 주원인이 된다. 본 논문은 단순 공칭(nominal) 쿼드로터 모델의 잔차 동역학(residual dynamics)을 Gaussian Process(GP)로 학습하여 모델을 보정하고, 이 보정 모델을 모터 입력 수준까지 내려가는 full-state MPC 파이프라인에 긴밀히 통합한다. 결과적으로 공칭 MPC 대비 추적 오차를 최대 70% 줄인다.

**2. 알고리즘.** 공칭 동역학 x_dot = f_dyn(x,u) (13차원 상태: 위치, 자세 쿼터니언, 속도, 각속도; 입력은 개별 로터 추력)에 GP 사후 평균 mu를 더해 보정 동역학 f_cor(x,u) = f_dyn(x,u) + B_d*mu(z_k)를 구성한다. GP는 body-frame 속도 B_v로부터 가속도 잔차 B_a_e = mu(B_v)를 축별로(x,y,z 각각 1D, RBF 커널) 예측한다. RK4로 이산화(x_{k+1}=f_RK4(x_k,u_k,dt))하고, 다중 슈팅 + SQP의 RTI 스킴(ACADOS/CaSADi)으로 이차형 비용 ||x-x*||_Q^2 + ||u-u*||_R^2를 입력 제약 u_min<=u<=u_max 하에서 최소화한다. 실시간성을 위해 inducing point를 20개로 제한한다.

**3. 비판적 검토.**
- 핵심 수학 기제(GP 회귀, MPC, RK4, 미분평탄성 기반 공칭 모델)는 모두 표준/외부 차용이며, 자체 정리·증명·이론적 보증은 없다. 엔지니어링 통합 논문이다.
- GP가 제공하는 예측 공분산(Sigma_mu)을 명시적으로 제어에 사용하지 않는다(저자도 명시). 즉 불확실성을 모델링만 하고 강건성·chance constraint로 활용·검증하지 않아, 잔차 평균 보정에 그친다.
- 비교 대상 baseline이 본질적으로 선형 드래그 모델(RDRv) 하나에 집중되어 있고, 신경망 기반 잔차 학습 등 다른 데이터 기반 방법과의 정량 비교는 없다.
- 정량 결과에 몬테카를로/통계적 유의성 처리(분산, 신뢰구간)가 부재하며, RMSE 평균값 위주다.

**4. 시뮬/실험.** 3단계 검증: (i) 단순 forward-integration 시뮬(0.5ms step, GP training-size 절충 분석), (ii) Gazebo/RotorS(AscTec Hummingbird), (iii) 실기체(0.8kg, T/W 5:1, 100Hz 광학추적, 50Hz body-rate). 3개 궤적(Random/Circle/Lemniscate)을 다양한 최대속도에서 수행. (sim) Table I: Circle v_peak=12에서 Nominal 338.3mm -> GP-MPC 28.4mm(-93%), Lemniscate v_peak=12에서 212.4mm -> 24.4mm(-88%); opt dt 4.13ms로 실시간 가능. (real) Table II: Circle Nominal 319.7mm -> GP 172.9mm(-46%), Lemn. 396.2mm -> 254.2mm(-36%). (real) Table III(드래그 보드 부착, 비대칭 외란): RDRv가 선형 적합 한계로 8-10 m/s에서 crash, GP-MPC 20은 안정적으로 최대 77% 개선. Baseline은 선행 연구 RDRv(외부).

**5. Contribution(delta).** 출판년(2021) 기준 SOTA 대비: Kabzan/Hewing의 자율주행 레이스카 GP-MPC 잔차 학습 라인을 3차원 쿼드로터로 확장하고, 가상입력(bodyrate/collective thrust)이 아니라 개별 모터 추력까지 모델링하여 실제 작동 한계(actuation limit)를 제약으로 반영한 최초의 full-state GP+MPC 쿼드로터 정식화. 선형 드래그(RDRv) 대비 고차/비대칭 공기역학 효과까지 포착하는 점이 명확한 우위. 다만 GP-MPC 잔차 학습 패러다임 자체는 이미 확립된 방향의 도메인 이식·확장이다.

**6. Limitation·Future work.** 저자: 예측 불확실성을 활용한 장애물 근접 안전 궤적, GP 빠른 적합을 이용한 온라인 재학습(바람·배터리 전압 변화 적응)을 향후 과제로 제시. 내가 보는 한계: (a) 공분산 미활용으로 강건성 보증 부재, (b) inducing point 20개·축별 1D 가정이 강한 커플링·난류 상황에서 표현력 제약 가능, (c) 검증이 쿼드로터에 국한되어 항공우주 G&C(파워드 디센트, 재진입 등)로의 직접 전이는 미입증, (d) 통계적 검정 부재.

**평가:** 68/100 (good) · 한줄평: GP 잔차 학습을 모터 입력 수준 full-state MPC에 실시간 통합해 실기체에서 최대 70% 추적 개선을 입증한 견고한 데이터 기반 제어 논문이나, 자체 이론 기여는 약하고 검증이 쿼드로터에 한정되어 항공우주 G&C 관련성은 방법론적 전이(잔차 적응 MPC) 측면에서 간접적이다.

---

## (2022, ACC, A real-time GP based MPC for quadcopters with unknown disturbances: Schmid et al.)

**1. 핵심 아이디어.** 미지의 외란(바람·테더에 의한 잠재력)을 가우시안 프로세스(GP)로 온라인 학습하여 MPC 예측에 통합한다. 고전 GP의 O(n^3) 복잡도 문제를 회피하기 위해 정상 커널 GP를 상태공간(LTI) 형태로 재정식화(Hartikainen-Särkkä)하여 칼만필터/RTS 평활로 O(n) 추론·학습을 수행하고, 이를 통해 Raspberry Pi 4 B에서 실시간 실행을 입증한다.

**2. 알고리즘.** 쿼드콥터를 호버 조건 근방에서 선형화한 12상태 모델(x=[p, p_dot, Phi, Phi_dot])에 세 축(x,y,z)의 1차원 GP 외란 모델 z_dot_i=F_i z_i, y_i=H_i z_i 를 증강. 증강 모델 (10): x_dot=Ax+sum_i C_i H_i z_i + Bu, z_dot_i=F_i z_i. 외란 가속도는 측정/예측 속도 차로 추출 p_dot_dot_d=(p_dot_meas - p_dot_pred)/dT. 불확실성 전파 Sigma^x_{t+k+1}=A Sigma^x A^T + sum_i C_i Sigma^z_i C_i^T, 칸스트레인트 타이트닝 (15)로 chance constraint Pr(x in X)>=p_x 보장. 불안정 시스템 공분산 발산 억제용 LQR 피드백 u=u_bar+K_inf(x_bar-x) 사용. 50회 하이퍼파라미터 갱신(eta=(0.03,0.01,0.005)) 후 nominal MPC -> GP-MPC로 전환, OSQP로 해를 구함.

**3. 비판적 검토.**
- 이론적 기여의 대부분이 인용/임포트다. 상태공간 GP는 [8], chance-constrained 타이트닝과 공분산 전파는 [5](Hewing), 증강 외란 모델은 [6][7]에서 가져왔다. 본 논문 고유의 정리/증명은 없고 "실시간 가능성 증명"은 사실상 구현·실측 시연이다.
- 검증이 단일 시나리오에 그친다. jMAVSim 한 가지 바람 설정(평균 22 m/s + 필터링 백색잡음)에서의 1회 비행만 제시하고 Monte Carlo, 다중 시드, 통계적 처리가 없다. baseline도 self-baseline(nominal MPC)뿐이며 외부 기법(DAC[2], data-driven MPC[13]) 비교가 없다.
- 실기체 비행이 없다. RPi 연산시간은 host 머신에서 sleep 명령으로 모사했고 실제 PX4 하드웨어 in-the-loop이 아니다. 저자도 향후 실내 비행시험을 과제로 남겼다.
- 선형화(호버 근방) 모델 의존성이 크다. 저자 스스로 z축 진동·y 변위 시 진동이 비선형 모델-실시스템 불일치에서 비롯되며 "정밀한 모델 예측에 의존한다"고 인정하는데, 이는 외란이 큰 실제 RLV/UAV 환경에서 robustness 한계로 직결된다.
- GP-MPC가 nominal 대비 더 공격적이고 진폭 큰 액추에이터 신호(Fig.2)를 만든다 - 추종 정확도 향상의 대가로 입력 진동이 증가, 실기체 마모/포화 측면 분석이 빠졌다.

**4. 시뮬/실험.** jMAVSim 시뮬레이션 1회(PX4 offboard, MAVSDK 연동), 강풍(평균 22 m/s) 하 기준궤적 추종. 정량: GP-MPC의 RMS 유클리드 오차 0.82 m vs nominal MPC 2.83 m (sim) - 약 3.5배 개선. 연산시간(RPi 4B 모사): nominal MPC 10 ms, GP-MPC 20 ms, GP 1개 하이퍼파라미터 갱신(batch 50) 최대 2 ms. baseline=conventional(nominal) MPC, self-baseline. MC/통계/실기체 없음.

**5. Contribution(delta).** 출판년(2022) 기준, 상태공간 GP의 선형복잡도 온라인 학습[8]을 chance-constrained MPC[5]와 결합하여 쿼드콥터 외란 보상을 임베디드(RPi 4B) 실시간으로 구현 가능함을 보인 것이 delta다. [5][6][7]은 주로 오프라인/개념 검증이었던 데 비해 본 논문은 온라인 학습 + 저가 임베디드 실시간성을 명시적으로 입증한 점이 진전이다. 다만 구성요소 각각은 선행 기법이며 새 정식화나 보장 정리를 제시하지는 않아 incremental에 가깝다.

**6. Limitation·Future work.** 저자: 무한지평 GP[10]로 배치 truncation 아티팩트 제거, nominal->GP 전환을 위한 분류기 설계, 모델 불일치 학습[13] 결합, 더 복잡한 쿼드콥터 모델, 실내 비행시험. 내가 보는 추가 한계: (i) 단일 시나리오·MC 부재로 통계적 신뢰성 미확보, (ii) 선형 호버 모델 의존으로 대각도/대외란 일반화 불확실, (iii) chance-constraint 보장이 가정(외란 무상관·정상 커널) 충족 시에만 성립하나 실외란이 이를 위반할 가능성, (iv) 액추에이터 진동 증가에 대한 안정성/포화 분석 미흡.

**평가:** 60/100 (fair) · 한줄평: 상태공간 GP+chance-MPC를 임베디드 실시간으로 잘 통합한 견실한 robotics-transfer 워크지만 단일 시뮬·self-baseline·고유 이론 부재로 항공우주 G&C 관련성은 간접적이다.

---

## (2022, Sci Robotics, Neural-Fly enables rapid learning for agile flight in strong winds: O'Connell et al.)

**1. 핵심 아이디어.** 강풍 환경에서 쿼드로터의 미지(unmodeled) 공력 외란을 빠르게 학습·보상하는 학습기반 적응제어 프레임워크다. 핵심 통찰은 (i) 서로 다른 바람 조건의 공력이 공통 표현(wind-invariant representation)을 공유하고, (ii) 바람 특이적 정보는 저차원 선형계수 공간에 놓인다는 것이다. 오프라인에서 DAIML(domain adversarially invariant meta-learning)로 공통 기저함수(DNN φ)를 학습하고, 온라인에서 합성 적응제어(composite adaptation)로 선형계수 a만 실시간 갱신한다.

**2. 알고리즘.** 잔차력 분해 f(q,q̇,w) ≈ φ(q,q̇)·a(w), φ는 DNN 기저, a는 바람별 선형계수. 오프라인 DAIML(Alg.1): 적응집합에서 최소제곱으로 a*(φ) 구하고, φ는 SGD+spectral normalization으로 갱신, 판별자 h와 min-max 적대학습으로 wind-invariance 강제. 온라인 제어법칙: u_NF = M q̈_r + C q̇_r + g − K s − φ(q,q̇)â (s = q̃̇ + Λq̃ 합성 추종오차). 적응법칙(합성): â̇ = −λâ − PφᵀR⁻¹(φâ − y) + Pφᵀs, 공분산 Ṗ = −2λP + Q − PφᵀR⁻¹φP (Kalman-Bucy 필터 유도, 자동 게인 튜닝).

**3. 비판적 검토.**
- 안정성 보장(Theorem 1)은 추종오차가 표현오차 ‖d(t)‖, 측정잡음 ‖ε‖, 바람 변화율 ‖ȧ‖에 비례하는 오차구로 지수수렴함을 주장하고 보충자료(S5)에 완전 증명을 제시한다. 다만 합성적응·spectral norm·Kalman-Bucy 등 핵심 기계는 기존 결과를 가져온 것이고, φa 분해 존재성은 인용된 보편근사정리에 의존한다.
- 검증 도메인이 쿼드로터 UAV(로보틱스)로, 항공우주 G&C(발사·재진입·착륙·자세)와는 거리가 있다. 단 Euler-Lagrange 일반식으로 정식화되어 VTOL·우주기체로의 전이 여지는 명시되어 있다(인용에 재사용 발사체 적응제어 포함).
- PWM을 φ 입력에 넣어 제어입력 u에 비선형의존이 생기는 문제를 직전 시점 PWM 대입으로 우회한다 — 실용적이나 이론적 엄밀성에는 약한 고리.
- MC/통계적 처리는 제한적(라프별 SD만 보고). 외부 baseline 비교는 강하지만 정식 분산분석은 없다.

**4. 시뮬/실험.** 전부 실기(real) 검증. Caltech Real Weather Wind Tunnel(1296팬, 최대 43.6 km/h=12.1 m/s), 자작 2.6kg 쿼드로터, Raspberry Pi 4 온보드 50Hz. figure-8 추종을 풍속 0/4.2/8.5/12.1 m/s + 시변 사인풍에서 6랩 측정(Table 1). RMS 추종오차(cm): NF가 0→3.2, 4.2→4.0, 8.5→5.8, 12.1→9.4, 사인→7.6으로 전 조건 최저. baseline: Nonlinear(11.9/23.9), INDI(7.3/10.7), L1(4.6/22.7) 대비 평균 66%(nonlinear)/42%(L1)/35%(INDI) 개선. 좁은 게이트(10cm 여유) 통과, 다른 드론(Intel Aero) 전이(NF-Transfer), 옥외 GPS 비행(7.5cm mean) 추가. 온보드 추론 3.5ms+적응 3.0ms=총 10ms.

**5. Contribution(delta).** 2022 출판년 기준 SOTA(INDI[4], L1[5], GP기반 MPC[Torrente 11]) 대비: (i) 사전학습 DNN을 적응제어 기저로 쓰면서 폐루프 지수안정·강건성을 보장하는 최초의 결합 — 기존 신경망 적응제어는 얕은 단층망에 국한, 기존 메타학습 제어(저자 선행[37], Richards[38])는 표현이 비효율적이거나 시뮬·무보장. (ii) DAIML의 적대적 정규화로 wind-invariant 표현을 disentangle. (iii) Kalman-Bucy 기반 합성적응으로 persistent excitation 없이 빠른 적응. 학습기반 적응제어에 보장을 결합한 방향을 연 기여.

**6. Limitation·Future work.** 저자: 1cm 한계 달성에는 코드(C++)·통신지연·자세추종 개선 필요; 학습 동역학과 계획·작동기 한계의 결합 미해명; cascaded 자세제어와의 상호작용 추가연구 필요. 내가 보는 한계: 검증이 쿼드로터 단일 플랫폼·figure-8 위주로 항공우주 임무 프로파일(대각속도·대각가속·추력제약) 검증은 없음; ‖ȧ‖ 큰 급변풍에서 오차구가 커지는 본질적 한계; PWM 비선형우회의 이론적 정당성 보강 필요.

**평가:** 80/100 (strong) · 한줄평: 보장 있는 학습기반 적응제어의 방향을 연 수작 — 실기·온보드·외부 baseline까지 탄탄하나 도메인은 항공우주가 아닌 쿼드로터 전이(방법은 Euler-Lagrange 일반성으로 G&C 전이 가능).

---

## (2023, AIAA SciTech, Multihierarchy Gaussian Process Models for Probabilistic Aerodynamic Databases using Uncertain Nominal and Off-Nominal Configuration Data: Scoggins et al.)

**1. 핵심 아이디어.** 확률적 공력 데이터베이스(aerodatabase) 구축을 위한 다계층(multihierarchy) Gaussian Process 프레임워크를 제안한다. 기존 다충실도(multifidelity) GP가 단일 "진실" 데이터에 앵커된 단일 충실도 계층을 전문가가 정의해야 하는 한계를 지적하고, 대신 하나의 데이터셋을 "nominal(이상화된 기준 형상)"으로 두고 표면 거칠기·비대칭 등 특정 물리현상에 대응하는 다수의 "correction" 함수를 선형 결합하는 구조로 일반화한다. Orion Crew Module 풍동 데이터에 적용해 양력·항력·피칭모멘트 계수와 그로부터 파생된 트림 받음각/트림 항력의 조건부 확률분포를 산출한다.

**2. 알고리즘.** nominal f0(x)~GP(0,k0); 각 correction에 대해 f_i = rho_i f0 + delta_i, delta_i~GP(0,k_i)로 2충실도 prior. 결합 모델 f(x)=f0(x)+sum_i w_i*Δf_i(x), Δf_i=f_i-f0. 전체가 독립 GP의 선형 혼합이므로 다시 GP prior가 되며(부록 A에서 혼합 공분산 cov(f_i,f_j)=sum_k phi_ik*phi_jk*K_k 유도). 학습은 Le Gratiet의 recursive AR1 방식으로 각 충실도를 개별 학습(복잡도 O(m^3)→재귀로 완화). 가중치 w는 posterior 평균을 적분소거한 선형 최소제곱 [mu1-mu0, mu2-mu0][w1,w2]^T = y-mu0로 추정. 커널은 ARD 가중 SE, 하이퍼파라미터는 L-BFGS-B(20 random restart)로 marginal likelihood 최대화. 사후 표준편차를 nominal/거칠기/비대칭 기여로 분해(σ=σ0+σ1+σ2).

**3. 비판적 검토.**
- 이론적 기여의 다수는 GP/co-kriging 기성 결과(Rasmussen-Williams, Kennedy-O'Hagan, Le Gratiet)의 import이며, 자체 신규 정리나 보장은 없다. 부록의 혼합 공분산 유도는 깔끔하나 표준적 선형변환 결과다.
- 검증이 정성적이다. 보류된 700D Rough 구성과의 일치, v0.60 참조 aerodatabase와의 일치 모두 "그림상 합리적 일치" 수준으로, RMSE·log-likelihood·calibration(예측구간 적중률) 같은 정량 지표나 통계가 전혀 제시되지 않는다.
- 외부 baseline 비교가 없다. 표준 AR1/recursive co-kriging나 단일 GP 대비 정량 우위를 보이지 않아 제안 구조의 실익이 수치로 입증되지 않는다.
- 단일 캠페인(133-CA)의 부분집합, β=0, 정규화 좌표만 사용하는 등 범위가 좁아 일반화 주장이 제한적이다. correction 가중치 결정이 "진실" 데이터 가용성에 의존한다는 핵심 약점은 저자도 인정한다.
- G&C 관점에서는 제어/유도 법칙이 아니라 6-DOF 시뮬레이션에 투입되는 공력 모델·불확실성 인프라이며, MC 궤적 분석에서 일관된 함수 샘플링을 가능케 한다는 점이 간접적 연관이다.

**4. 시뮬/실험.** NASA LaRC NTF 133-CA Orion CM 풍동 캠페인의 부분집합(α 158-202°, M∞ 0.2-0.9, ReD 3.8e6-28e6). 7개 구성 중 4개를 nominal(IDAT smooth)·거칠기 보정(IDAT 240grit)·비대칭 보정(700D smooth)·테스트(700D rough)로 사용. 빈 처리 후 nominal 1192/거칠기 389/비대칭 713/테스트 204점. 산출물은 C_L,C_D,C_m,cg 사후분포, 트림 받음각·트림 항력의 조건부 확률밀도(Fig.12-13), 불확실성 분해(Fig.10). 정량 결과로는 학습된 12개 파라미터(보정 가중치 w1,w2, 상관 rho1,rho2, 커널 σ·길이척도; 예: C_L w1=0.48,w2=0.16; C_D 거칠기 ReD 길이척도=8.02e6로 항력의 Re 의존성 포착) 제시. 모두 (sim)이며 baseline은 자체 보류 데이터 및 v0.60 참조 DB의 시각 비교에 그친다. MC 통계·정량 오차 없음.

**5. Contribution(delta).** 출판년(2023) 기준, 표준 다충실도 GP가 요구하는 단일 충실도 계층 가정을 제거하고 nominal+다중 correction이라는 물리현상별 분해 구조로 재정식화한 점이 핵심 delta다. 이를 통해 (i) 불확실성이 특정 off-nominal 현상(거칠기/비대칭)에 추적가능하게 귀속되고, (ii) 비행데이터로 가중치를 Bayesian 갱신할 여지를 제공하며, (iii) 트림 받음각/트림 항력 등 파생량의 일관된 확률분포를 marginalization으로 산출한다. AR1을 특수 경우로 포함하는 일반 GP 혼합으로 위치시킨 점은 명확한 개념적 진전이나, 새로운 추론 알고리즘이 아니라 기존 recursive co-kriging의 재배열이라 delta 규모는 중간이다.

**6. Limitation·Future work.** 저자: 가중치 추정이 진실 데이터에 의존(데이터 없으면 prior 설정 모호, Apollo 데이터 전이 제안), 가중치를 입력 의존 기저함수 선형결합으로 확장 가능, Gaussian 분포를 기존 aerodatabase API에 통합하는 작업 필요. 내가 보는 한계: 정량 검증·외부 baseline·calibration 지표 부재로 우수성이 입증되지 않음, 단일 캠페인/β=0/정규화 좌표로 범위 협소, sting 효과 무시 및 σ_n=0.1 보수적 가정의 영향 미평가, 6-DOF 궤적/제어 성능에 대한 end-to-end 영향(landing ellipse 등) 미제시.

**평가:** 63/100 (fair) · 한줄평: 물리현상별 분해형 확률적 aerodatabase라는 깔끔한 GP 혼합 프레임워크지만 정량 검증·외부 baseline 부재로 실익 입증이 약하며, G&C에는 시뮬 투입용 공력 UQ 인프라로서 간접 관련된다.

---

## (2024, ACC, Safety-Critical Control with Uncertainty Quantification using Adaptive Conformal Prediction: Zhou et al.)

**1. 핵심 아이디어.** 미지의(그리고 시간에 따라 변하는) 운동 잡음을 갖는 확률적 시스템에 대해, 분포 가정 없이(distribution-free) 고확률 안전 보장을 제공하는 안전 제어 프레임워크를 제안한다. 핵심은 Adaptive Conformal Prediction(ACP)으로 미래 상태 예측의 불확실성을 온라인으로 정량화하고, 이를 확률적 Control Barrier Function(CBF)/Safety Barrier Certificate(SBC) 제약에 주입하여 MPC 최적화에 내장하는 것이다. 다로봇 충돌 회피(단일 적분기·유니사이클)에서 검증한다.

**2. 알고리즘.** 이산 확률 동역학 x̂_{k+1}=f̃(x̂_k)+g̃(x̂_k)u_k+ε_k (ε_k 분포 미지)에서 칸스 제약 P(h(x̂_{k+1})≥0)≥1-β를 만족시키고자 한다. CBF 항 B_k=L_f h+L_g h·u_k+K(h(x_k)). 비적합도(nonconformity) 점수 E_{B_k}=|B̂_k-B_k|를 비감소 집합 𝔈에 저장하고 r=⌈k(1-α)⌉ 분위수 E_{B_k}^{(r)}를 취해, 결정론적 제약 B_k-E_{B_k}^{(r)}=L_f h+L_g h·u_k+K(h)-E_{B_k}^{(r)}≥0 (Thm 1)을 MPC에 삽입. ACP 온라인 갱신: α_{k+1}=α_k+δ(α-e_k), e_k는 S_k≤S_k^{(r)}이면 0, 아니면 1. 미래 시점 B̂_k^τ를 모르므로 [18]의 time-lagged 방식으로 τ-step ahead 점수를 대용. Prop 1: Lipschitz 상수 L로 E_{B_k}≤L·E_{x_k}, Prop 2: MPC 하에서 H→∞이면 보장이 1-α로 수렴.

**3. 비판적 검토.**
- 핵심 기계(ACP의 분위수 적응 [16][17], CP 보장 [15], 확률적 SBC [8], ACP-for-MPC 분석 [18])가 거의 전부 인용·요약되어 있고, 저자 고유 기여(Thm 1, Prop 1)는 이들을 결합·재배치하는 비교적 가벼운 유도다. Lemma 2와 ACP 확률 보장(Lemma 3, Prop 2)은 명시적으로 [8],[18]에서 가져옴.
- 비적합도 집합의 i.i.d./교환가능성 가정이 시변 잡음에서 쉽게 깨진다는 점을 스스로 인정하고 ACP로 우회하지만, time-lagged 구성과 Lipschitz 경계의 정당화는 다소 휴리스틱하다.
- 안전 보장이 점근적(H→∞)이라 유한 호라이즌에서의 보장 손실(p=(α+δ)/(Hδ))이 남고, MPC 분석은 τ=1 단일 스텝으로 단순화("for analysis convenience")했다.
- 외부 SOTA와의 정량 비교 부재: 가장 가까운 선행인 Dixit [18](거리 기반 ACP-MPC)와 직접 대조하지 않고, baseline은 자신들이 개선 대상으로 삼은 CBF/SBC뿐이다.

**4. 시뮬/실험.** 모두 시뮬레이션. (a) 단일 적분기에서 H/γ/α 파라미터 분석 및 ACP-SBC가 B를 95% 신뢰수준(α=0.05)으로 추정(Fig 2d)(sim). (b) 정규/균등/혼합 분포 잡음 하 분포-free 검증: CBF-기반은 장애물과 충돌, ACP-SBC는 충돌 회피(Fig 3)(sim). (c) 30로봇 무장애물 조율, 20개 random seed로 20회 반복, 최소거리 곡선으로 무충돌 확인(Fig 4)(sim). (d) CoppeliaSim 유니사이클 Khepera IV 6대, 3장애물 환경에서 ACP-SBC 무충돌, CBF는 충돌(Fig 5)(sim). 정량 성공률 표는 없고 대부분 궤적·최소거리 곡선 위주.

**5. Contribution(delta).** ACP(분포 이동 하 온라인 분위수 적응)와 확률적 CBF/SBC를 결합해, 미지·시변 잡음에 대해 분포 가정 없는 고확률 안전 제어를 MPC에 내장한 점이 delta. 출판년(2024) 기준 ACP+MPC 자체는 Dixit [18]이 거리 기반 제약으로 선점했으나, 본 논문은 거리 기반이 긴 호라이즌을 요구한다는 [19]의 약점을 들어 CBF/SBC 제약으로 대체한 명확하지만 점진적인 재조합이다.

**6. Limitation·Future work.** (저자) 보장이 점근적이며 time-lagged 대용에 의존; 향후 다양한 동역학/실로봇 확장 시사. (내가 보는 한계) 검증이 전부 지상 다로봇 시뮬레이션이고 실하드웨어·실시간 타이밍/복잡도 분석이 없으며, 가장 가까운 선행과의 정량 대조가 빠져 delta의 크기를 입증하기 어렵다. 항공우주 G&C 관점에서는 직접 적용 사례가 아니라 전이 가능성 수준의 관련성.

**평가:** 68/100 (good) · 한줄평: ACP와 확률적 CBF의 견고한 분포-free 안전 제어 재조합이나, 기계가 대부분 인용이고 검증이 지상 다로봇 시뮬에 국한되어 항공우주 G&C로는 간접적 전이 도메인.

---

## (2024, ECC, Efficient Zero-Order Robust Optimization for Real-Time Model Predictive Control with acados: Frey et al.)

**1. 핵심 아이디어.** 불확실성 전파(공분산/타원체 튜브)를 MPC 문제 외부에서 수행하여 명목 변수만으로 축소 부분문제를 푸는 zero-order robust optimization(zoRO) 알고리즘을, acados 프레임워크 안에 효율적인 오픈소스 C 구현으로 통합한다. BLASFEO 선형대수와 RTI(실시간 반복) 스킴을 결합하고, custom-update 템플릿 인터페이스를 통해 backoff 계산을 준비/피드백 단계에 분산 배치한다. 기존 Python zoRO 구현 대비 약 10배의 계산시간 단축을 보인다.

**2. 알고리즘.** 원 robust/stochastic OCP는 명목 동역학 x_{k+1}=psi(x_k,u_k,0), 불확실성 전파 P_{k+1}=Phi_k(P_k)=(A_k+B_kK_k)P_k(A_k+B_kK_k)^T + G_kW_kG_k^T, 그리고 backoff를 더한 제약 0 <= h_k+beta_k로 구성된다. backoff는 beta_{k,i}=gamma*sqrt(grad h^T [I;K]^T P_k [I;K] grad h). zoRO는 P를 OCP에서 제거하고 명목 변수만의 부분문제 (5)를 풀되, (1) backoff 전파(3)+(4)를 현재 명목 궤적으로 근사 갱신, (2) 부분문제를 SQP로 근사 해 — 두 단계를 교대한다. 단일 SQP 반복은 backoff의 명목/P 미분을 무시하는 inexact Newton에 해당하며(식 (7) 항 무시), RTI에서는 준비단계에서 backoff를 미리 계산하고 피드백단계에서 x_0-bar로 QP를 풀어 배포한다.

**3. 비판적 검토.**
- 핵심 알고리즘 zoRO와 수렴성은 선행 [6,7,9]에서 가져온 것이며, 본 논문 고유의 새 정리/증명은 없다. "무시한 Jacobian 항이 불확실성 크기에 따라 스케일하므로 수렴성이 유지된다"는 논증은 정성적 수준에 머문다.
- baseline이 모두 자기/선행 구현(zoRO-21, nominal, naive)이며 외부 경쟁 기법(tube-MPC 다른 구현, scenario-MPC 등)과의 비교가 없어 10배 속도향상의 절대적 의미를 가늠하기 어렵다.
- 폐루프 robustness/제약 만족에 대한 형식적 보장의 실험적 검증이 없고, diff-drive에서 process noise를 주입한 단일 시나리오만 제시된다(다양한 노이즈/시나리오에 대한 MC 통계 부재).
- 검증 플랫폼이 노트북 CPU(i5-8365U)이며 실제 임베디드 타깃 배포는 보이지 않는다(타이밍은 임베디드 지향이나 demo는 데스크톱).

**4. 시뮬/실험.** (1) chain 벤치마크: 질량수 n_mass=3~6에 대해 OCP당 평균 CPU 시간 스케일링 비교(zoRO-24 vs zoRO-21 vs nominal vs naive), zoRO-24가 명목 OCP에 근접한 복잡도 달성 (sim). (2) differential-drive robot: 상태 (px,py,theta,v,omega), 20 shooting interval, horizon 2.0s, 충돌회피 제약 (9), W=diag(2e-6,2e-6,4e-6,1.5e-3,7e-3), P0=W, gamma=3.0, RK4 폐루프에 process noise 주입; 50회 동일 노이즈 실행 최소값 기준 총 계산시간 약 0.7ms, zoRO-21 대비 약 10배 단축, propagation/backoff 비중이 zoRO-21은 약90%인데 zoRO-24는 약10% (sim). 모든 코드 오픈소스(acados v0.2.5). baseline: 내부/선행 구현(zoRO-21, nominal, naive).

**5. Contribution(delta).** 출판년(2024, arXiv 2023) 기준 zoRO 알고리즘 자체는 [6,7]에서 이미 제안된 선행 기술이다. 본 논문의 delta는 (i) zoRO를 acados/BLASFEO 위에 효율적 C로 구현하고, (ii) RTI 준비/피드백 분할에 backoff 계산을 자연스럽게 통합하며, (iii) json 기반 custom-update 템플릿 인터페이스로 사용자가 손쉽게 zoRO를 구성하게 한 점이다. 즉 새 G&C 방법론이 아니라, tube/robust MPC를 널리 쓰이는 오픈소스 프레임워크에서 실시간 가용하게 만든 구현/엔지니어링 기여(약 10배 속도향상).

**6. Limitation·Future work.** 저자: adjoint correction(차선성 보정)·linear feedback K 최적화 미통합, 향후 확장 예정. 내가 보는 한계: 고유 이론 기여 부재, 외부 baseline 부재, 폐루프 보장의 정량 검증/MC 통계 미흡, 검증이 robotics 단일 예제에 국한되고 임베디드 실측 부재. 항공우주 G&C(동력 하강·재진입 등)로의 직접 적용·검증은 향후 과제.

**평가:** 71/100 (good) · 한줄평: zoRO를 acados 생태계에서 실시간화한 견고한 구현/도구 기여이나 고유 이론·외부 baseline이 얇고, 검증이 robotics 예제에 한정되어 항공우주 G&C 관련성은 간접적(도구로서는 전이 가능).

---

## (2025, Annu. Rev. Control, Gaussian processes for dynamics learning in model predictive control: Scampicchio et al.)

**1. 핵심 아이디어.** Gaussian process(GP) 회귀로 미지의 동역학 잔차를 학습해 stochastic nonlinear MPC에 통합하는 전 과정을 자기완결적으로 정리한 서베이 논문이다. 공칭 모델 g_nom에 GP로 학습한 잔차 항 B_d·g(x,u)를 더한 동역학(식 17)을 두고, 이를 chance-constraint 하에서 receding-horizon으로 푸는 문제(P)를 핵심 골격으로 삼는다. 저자들은 이 파이프라인을 (i) 확장성(scalable GP), (ii) 예측 horizon 상의 불확실성 전파, (iii) 폐루프 안전성 보장의 세 축으로 분해해 각 분야의 방법론을 망라하고, MPC에 아직 적용되지 않은 유망한 UQ 패러다임(conformal prediction, Wasserstein 기반 한계 등)까지 짚는다.

**2. 알고리즘.** 동역학: x_{i+1}=g_nom(x_i,u_i)+B_d·g(x_i,u_i)+v_i, v_i~N(0,σ_v²I). GP 사후분포: g_{Z*}|Y ~ N(μ(Z*),Σ(Z*)), μ=K_{Z*,Z}(K_{Z,Z}+σ_w²I)^{-1}Y. Stochastic OCP(식 19)을 MPC(식 20/45)로 근사 — moment 기반 형태로 μ^x_{i+1|k}=ρ(·), Σ^x_{i+1|k}=Φ(·)를 전파하고, chance-constraint h_j(μ^x,u)+ᾱ·sqrt(∇_x h_j^T Σ^x ∇_x h_j)≤0 (ᾱ=Φ^{-1}(p_j))로 tightening. 확장성: SoR/FITC/VFE/SSGP/SKI 등 O(N³)→O(NM²)·O(M³) 근사. 불확실성 전파: linearization(식 34-35), exact moment matching, sigma-point 전파(식 38-40). 안전성: RKHS 농도 부등식으로 신뢰영역 P(△)≥p 구성 후 robust-in-probability 보장(식 42-44).

**3. 비판적 검토.**
- 저자 고유의 정리·증명이 없다. 핵심 기계(농도 부등식, RKHS 한계, robust-in-probability, KL/Wasserstein 한계)는 모두 인용으로 수입되며, 본문 worked example(FITC+linearization+chance-constraint, 식 33 박스)이 유일한 자체 유도다. 서베이로서는 자연스러우나 이론적 기여의 깊이는 제한적이다.
- 자체 실험이 전무하다. Fig 6의 15점 toy GP 외에는 Table 2로 타 문헌 ~40편의 응용을 카탈로그화할 뿐, MC·baseline·정량 비교를 직접 수행하지 않는다. 따라서 어떤 근사·전파 조합이 실제로 우월한지에 대한 1차 증거는 제공하지 않는다.
- independence(Markovianity) 가정의 폐기가 아직 receding-horizon MPC에 적용되지 못했음을 저자 스스로 인정(7.1.2). 즉 다단계 예측의 핵심 한계가 미해결로 남는다.
- 안전성 보장의 토대인 RKHS norm 상한 가정이 데이터로 검증 불가능하다는 점(7.4, [503])을 정직하게 지적하나, 이는 GP-MPC 안전성 주장의 실용성을 근본적으로 약화시킨다.

**4. 시뮬/실험.** 자체 시뮬레이션 없음. 예시는 Fig 6(균일격자 z∈[3,7], 15 학습점 GP에서 linearization/moment matching/sigma-point의 사후분포 근사 비교, 정성적). Table 2가 문헌 응용을 집계: full GP/SSGP/FITC/SoR 모델 × 90/117/396 등 전파기법 × pH 중화·가스분리·UAV·quadrotor·autonomous racing·skid-steer·inverted/Furuta pendulum·반회분식 반응기 등. 폐루프 보장(✓)·sim-only·online 여부를 표기하나 수치 baseline 비교는 원문 참조로 위임.

**5. Contribution(delta).** 출판년(2025) 기준, scalable GP·불확실성 전파·폐루프 안전성 세 갈래를 단일 GP-MPC 파이프라인으로 통합 정리하고, 각 블록의 계산복잡도(Table 1)와 응용 현황(Table 2)을 체계화한 점이 delta다. 특히 지구통계·다중팔 밴딧·conformal 등 control 외부에서 개발돼 MPC에 미적용된 UQ 결과를 명시적으로 발굴해 향후 방향으로 제시한다. 그러나 '새 G&C 방법'이 아닌 종합·정리 성격이라 novelty는 incremental에 가깝다.

**6. Limitation·Future work.** 저자: 폐루프 보장의 이론 부족, 제어 루프 내 online learning의 엄밀한 분석, 근사로부터 전파된 불확실성 한계의 back-off 통합, latent variable 모델 기반 분석을 challenge로 명시. 내가 보는 한계: (i) 자체 검증·baseline 부재로 권고가 정성적 수준에 머묾, (ii) RKHS norm 가정의 비검증성이 안전성 주장의 실효성을 제약, (iii) 비-Markovian 상관의 MPC 통합 미해결, (iv) 도메인 측면에서 항공우주 G&C(강하·재진입·자세제어) 사례가 없어 해당 도메인 적용성은 간접적이다.

**평가:** 65/100 (good) · 한줄평: 권위 그룹의 포괄적·정확한 GP-MPC 참조 서베이로 control-theory 근접 도메인에서 가치가 높으나, 자체 실험·고유 증명 부재와 항공우주 G&C 직접 사례 부재로 영향은 간접적이다.

---

## (2025, Automatica, Learning for Predictive Control: A Dual Gaussian Process Approach: Liu et al.)

**1. 핵심 아이디어.** 미지의 비선형 동역학 g를 가우시안 프로세스(GP)로 학습해 MPC 예측모델을 보강하되, 인간의 장기/단기 기억 개념을 모방한 Dual Gaussian Process(DGP) 구조를 제안한다. 장기 GP는 오프라인으로 학습한 시간불변 불확실성을 보존(망각 방지)하고, 단기 GP는 온라인 재귀 업데이트로 시변 외란·급변에 빠르게 적응한다. 두 GP의 합으로 예측분포를 구성하고 이를 chance-constraint MPC에 임베딩한다.

**2. 알고리즘.** 시스템: x(k+1)=Ax+Bu+g(x,u)+v. 미지항 Δ=Δ_l+Δ_s를 두 독립 GP로 모델링(가정1). 단기 GP는 sparse variational GP의 재귀 베이즈 업데이트(식 22: m_u^k=m_u^{k-1}+L_k r_k, 망각인자 0<λ≤1, S_u^k=λ^{-1}(S_u^{k-1}-L_k G_k L_k^T))로 평균·분산을 동시 갱신. DGP의 ELBO를 해석적으로 유도(식 36)해 단기 GP 변분파라미터 최적해(식 37) 도출. 예측 평균/분산(식 40)을 exact moment matching(부록 B)으로 H-step 전파(식 43: μ_x^{i+1}=Aμ_x^i+Bu+μ_Δ, Σ_x^{i+1}=AΣ_xA^T+Σ_Δ+Σ_ε). polytopic 제약은 quantile 함수로 chance constraint(식 45-47)→결정론적 형태(식 48)로 변환 후 표준 NLP로 해결.

**3. 비판적 검토.**
- MPC 논문임에도 폐루프 안정성·재귀 실행가능성(recursive feasibility)에 대한 형식적 보증이 전혀 없다. chance constraint는 quantile 기반의 명목적 처리일 뿐이며, GP 평균/분산이 부정확할 때 제약 위반 가능성에 대한 보장이 없다.
- 가정1(장·단기 GP 독립)은 편의적이다. 단기 GP가 장기 GP 평균 주변을 학습한다는 설명은 직관적이나, 두 GP가 같은 잔차를 공유할 때 식별 가능성(identifiability)·이중계수 문제에 대한 논의가 없다.
- 회전 서브시스템을 PD로 단순화하고 병진 서브시스템에만 MPC를 적용했다. 실제 고기동에서 회전-병진 결합이 GP 보상 성능에 미치는 영향이 평가되지 않았다.
- 핵심 이론기계(SVGP, moment matching)는 기존 문헌에서 수입한 것이고, 본 논문의 신규 유도는 dual 구조 ELBO와 재귀 업데이트에 집중된다.

**4. 시뮬/실험.** 쿼드로터(m=1.9kg) 헬릭스 궤적 추종 단일 시나리오(sim). T=20s, Ts=0.05s, H=5, N=1000, 의사입력 M=20, 0-10s 정상풍→10s 시변풍 급변. baseline은 자가구성 ablation 3종(Baseline MPC, 장기만 LGP-MPC, 온라인만 OGP-MPC). 반복학습 효과: DGP-MPC MSE(×10^-3) 1st iter X/Y/Z=0.86/0.70/0.87 → 2nd iter 0.07/0.04/0.51로 baseline MPC(3.30/7.71/2.93)·LGP·OGP 대비 대폭 개선. 단 Monte Carlo 없음, 외부 baseline 없음, 단일 외란 프로파일, 실기·HIL 없음.

**5. Contribution(delta).** 출판년(2025, 사실상 2022 arXiv) 기준 SOTA 대비 delta: 기존 GP-MPC는 (i) 오프라인 GP 고정(LGP, 적응 불가) 또는 (ii) 순수 온라인 GP(망각 발생)의 양자택일이었다. DGP는 장기 기억 보존과 단기 빠른 적응을 단일 구조로 통합한 최초 프레임워크로 제시되며, 단기 sparse GP의 평균·분산 동시 재귀 업데이트(식 22)와 DGP 전용 ELBO 해석해(식 36-37) 및 불확실 입력 모멘트 매칭(부록 B)을 자체 유도했다. 저자 자신의 CDC 2021 예비연구의 확장이다.

**6. Limitation·Future work.** 저자: DGP를 MPC 외 다른 제어기로 일반화, 비반복 작업에서는 OGP와 성능 유사. 내가 보는 한계: (1) 안정성/실행가능성 보증 부재로 안전필수 G&C 적용에 직접 신뢰 어려움; (2) 단일 시나리오·MC 부재로 통계적 강건성 미검증; (3) 온보드 실시간성에 대한 실측 타이밍 없음(복잡도 논의만 존재); (4) 망각인자 λ·초기 β 등 하이퍼파라미터 튜닝 민감도 미분석.

**평가:** 74/100 (good) · 한줄평: 장·단기 이중 GP로 기억과 적응을 통합한 잘 유도된 GP-MPC 기여이나 폐루프 보증 부재와 단일 시뮬 검증이 한계 — 쿼드로터 검증의 control-theory 방법으로 G&C 적응제어에 전이 가능성은 있으나 항공우주 직접성은 간접적.

---

## (2025, JGCD, Atmospheric-Density-Compensating Model Predictive Control for Targeted Reentry of Drag-Modulated Spacecraft: Hayes·Caverly)

**1. 핵심 아이디어.** 추진제가 없는 드래그-변조(drag-modulated) 소형 위성을 대기밀도 불확실성 하에서 목표 재진입 지점으로 유도하는 추정-제어 통합 프레임워크다. 핵심은 비행 중 실제 밀도와 가이던스 생성에 쓴 공칭 밀도 간 오차를 GPS 상대운동 측정으로 EKF가 추정하고, 이 추정 밀도오차를 LTV-MPC의 예측 외란으로 주입해 추적성능·제어노력·포화 강건성을 동시에 개선하는 것이다. 역사적 우주기상 데이터를 사용한 1000회 몬테카를로로 검증한다.

**2. 알고리즘.** 드래그 가속 a_d = ρ v² C_b, 제어입력은 탄도계수 변화 ΔC_b이며 밀도오차는 정규화 스케일 c_Δρ로 모델링(Δρ = c_Δρ·ρ_nom). 상대운동은 Schweighart-Sedwick(J2 보정 CW) LTV 모델 ẋ_dyn = A(t)x_dyn + [0; -Δa_d]. EKF는 x_KF=[x_dyn; c_Δρ]로 증강하고 c_Δρ는 이산 랜덤워크로 처리, ZOH 이산화 후 표준 시간/측정 업데이트로 Δρ̂ 추출. MPC 비용은 J=Σ x_k Q_c x_k + x_N Q_{c,N} x_N + Σ R_c(ΔC_{b,k+1}-ΔC_{b,k})²로 제어입력 '변화'를 패널티하며(보유 비용 0), 입력 박스제약 ΔC_{b,min}≤ΔC_b≤ΔC_{b,max}, 종단비용 Q_{c,N}은 DARE 해로 설정. convex QP(quadprog, 임베디드는 OSQP/CVXgen 가능)로 매 스텝 풀고 첫 입력만 적용. 가이던스는 Omar 방식(C_b1→C_b2 단일 스위치 t_swap)을 단순화하고 C_{b,mid}=√(C_{b,min}C_{b,max})로 중앙화해 제어권한을 확보.

**3. 비판적 검토.**
- 이론적 핵심 기구가 대부분 인용/수입이다. 상대운동 모델(Schweighart-Sedwick), 가이던스(Omar), EKF/MPC/DARE는 표준이며, 본 논문의 기여는 이들의 조합·문제특화 구성이다. MPC의 안정성/재귀가능성(recursive feasibility)이나 EKF 수렴에 대한 형식적 보증은 없다.
- 비교 baseline이 자기참조적이다. 핵심 비교는 동일 제어기를 Δρ 추정 사용/미사용으로 나눈 것이고, '선행' [33]도 같은 그룹 작업이다. 외부 독립 방법(예: incremental MPC [34], LQR 추적 [29])과의 정량 비교가 없어 우월성 주장 강도가 약하다.
- 횡단(cross-track) 운동을 제어불가로 보고 상태에서 제거했는데, 장기 궤도에서 횡단오차 성장에 대한 정량 근거가 본문 가정 수준에 머문다.
- c_Δρ 랜덤워크 프로세스 모델은 궤도주기 진동을 포착하지 못해(Fig.10) 추정에 구조적 바이어스가 남으며 저자도 인정한다.
- 온보드 실시간성을 'convex QP라 빠르다'로 주장하나 측정된 solve time·복잡도(36스텝×5서브스텝) 수치가 없다.

**4. 시뮬/실험.** 1000회 몬테카를로(sim), MATLAB Aerospace Toolbox, NRLMSISE-00 밀도·EGM2008(공칭 차수15/분산 차수40) 중력·역사적 F10.7/Ap·GPS 잡음(σ_pos=5m, σ_vel=0.05m/s). 초기 ~375km, 종단 100km von Karman. 결과: Δρ 추정 사용 시 재진입 추적오차 평균 12.1 km, 99.7%가 100km 미만, 성공률 98.4%; 미사용 시 평균 244 km, 100km 미만 23.0%에 불과. 가이던스 자체 타게팅 평균오차 6.31km(97.7% < 10km). R_c 튜닝 스윕(10^10,10^12,10^14)으로 작동기 사용량 vs 추적오차 트레이드오프 제시. 실패 16건은 2005년 태양폭풍급 극단 밀도변동으로 드래그장치 포화. 외부 baseline·HIL·실비행 없음.

**5. Contribution(delta).** 출판년(2025) 기준 선행 [33](LTI MPC, 밀도지식 가정·획득법 부재)·[34](incremental MPC, 소시간간격 제약) 대비: (i) GPS 상대측정 기반 EKF로 day-of-flight 밀도오차를 실제 획득가능한 방식으로 추정, (ii) LTI→LTV MPC 확장으로 긴 예측구간(약 4궤도) 확보, (iii) 탄도계수 '값'이 아닌 '변화'에 패널티를 둬 보유에너지 0 가정과 작동기 사용 최소화 일치, (iv) 역사적 우주기상 기반 대규모 MC로 현실 검증. 신규성은 새 제어이론이 아니라 드래그-변조 재진입에 추정-제어를 통합·현실화한 시스템 레벨 기여다.

**6. Limitation·Future work.** 저자: 더 정확한 밀도추정(필터를 제어보다 고속 구동), 궤도주기 진동 포착 프로세스 모델, 고도별 가변 제어주기, FOH 이산화. 추가로 내가 보는 한계: 형식적 안정성/재귀가능성 보증 부재, 외부 baseline 비교 부재(자기참조), 실시간 solve-time 미측정으로 온보드 주장 미검증, 가이던스 재생성 실패케이스(16건)에 대한 closed-loop 대안 부재, 단일 t_swap 가이던스의 보수성.

**평가:** 70/100 (good) · 한줄평: 표준 기구의 견고한 시스템 레벨 통합과 현실적 대규모 MC가 강점이나 형식적 보증·외부 baseline·실시간 timing 부재로 in-domain 견실 저널 논문 수준의 핵심 G&C 기여.

---

## (2025, IEEE TCST, L4acados: Learning-Based Models for acados, Applied to Gaussian Process-Based Predictive Control: Lahr et al.)

**1. 핵심 아이디어.** 학습 기반(신경망·가우시안 프로세스) 잔차 모델을 실시간 최적제어 소프트웨어 acados에 통합하기 위해, SQP 반복마다 모델·자코비안 민감도를 외부 Python 모듈에서 계산해 주입하는 범용 프레임워크 L4acados를 제안한다. 비선형 OCP를 매 반복에서 아핀(LTV) 동역학으로 재구성하고 그 계수를 모델 파라미터로 set/get함으로써, 기존 Gauss-Newton SQP와 동일한 반복점을 정확히 복원한다. 이를 zero-order robust optimization(zoRO)에 확장해 효율적·모듈형 GP-MPC(zoGPMPC)를 구현하고 자율주행 두 하드웨어에서 검증한다.

**2. 알고리즘.** 진동역학 x(t+1)=f(x,u)+B_d g^tr(x,u)+w. 각 SQP 반복에서 비선형 동역학 (4c)를 아핀 (8c): x_{k+1}=Â_k x_k + B̂_k u_k + ĉ_k 로 치환하되, Â_k,B̂_k = ∂(f+B_d g)/∂(x,u)|_iterate (식 6), ĉ_k = f+B_d g - Â_k x̂_k - B̂_k û_k (식 9)로 설정 → 원 선형화 (5c)와 정확히 일치. acados는 명목 f와 ∂f를 내부 계산, 잔차 g와 ∂g/∂x,∂g/∂u는 PyTorch/GPyTorch ResidualModel이 N개 선형화점 배치로 한 번에 평가(병렬화). GP-MPC: 공분산 동역학 Ψ_k=A Σ A^T + B_d Σ^g B_d^T + Σ_w (식 15), chance-constraint tightening β_j=γ_j sqrt(C_j Σ C_j^T) (식 16), zoRO는 (17)로 Σ를 전반복 궤적 기준 전파하고 tightening 자코비안은 동결(식 19)해 suboptimal-but-feasible KKT점 도달.

**3. 비판적 검토.**
- 핵심 "정확 복원" 논증은 LTV 재구성이 동일 Gauss-Newton 반복을 낳는다는 표준 SQP 등가성에 기반하며, 자체 신규 증명이 아니라 잘 정리된 엔지니어링 논거다. zoRO/GP-MPC 수학([35],[38])은 대부분 인용·차용이고 수렴/안정성 형식 보장은 제시되지 않는다.
- zoGPMPC는 제약 tightening 자코비안 전체를 무시([33],[38]와 동일 방향)해 [32],[35]보다 더 거친 근사를 쓴다 - feasible는 유지되나 보수성/최적성 손실의 정량적 특성화는 부족.
- 기여의 큰 부분이 소프트웨어 통합·병렬화 성능이며, 방법론적 novelty(LTV 파라미터화로 외부 민감도 주입)는 영리하지만 개념적으로 얇다.
- 검증은 전적으로 지상 차량(미니어처·풀스케일)으로, G&C 도메인 직접 검증은 없다(전이 가능 도구로서 평가).

**4. 시뮬/실험.** (1) NN-MPC 벤치마크(sim): double-integrator+제로가중 NN으로 width/depth(파라미터 6.7만~105만), 예측구간 N=1~1000, CPU/GPU·OpenMP 병렬화 스윕; L4acados가 L4CasADi·CasADi 대비 N≥10에서 최대 ~100배 speedup. (2) 미니어처 레이싱(real, 1:24 Mini-Z, Qualisys): N=40, 30Hz, N_sim=3000(100s); Table I에서 zoGPMPC(SQP-RTI, exact GP) 총 4ms·feedback 2ms, exact/online D=100까지 실시간, 트랙제약 100% 준수하며 Nominal 대비 충돌 회피, D=400에서 공격적 주행, laps/cost 정량 비교(외부 baseline: cautious GPMPC, CasADi/L4CasADi). (3) 풀스케일 차량(real, ISO 3888-1 이중차선변경 30km/h): zoGPMPC 총 6ms(95% CI tightening, p_j=95%), nominal 대비 actuation·comfort·tracking 개선, 20ms 주기 내 실시간(Table II).

**5. Contribution(delta).** 출판년(2025) 기준, 학습 기반 모델을 acados에 통합하는 기존 방식(do-mpc, HILO-MPC, LbMATMPC, L4CasADi)이 모델을 CasADi로 재구현/JIT 컴파일에 의존해 ML 생태계·배치 병렬화와 단절되던 한계를, 외부 Python 민감도를 solver-interface 수준에서 주입(LTV 재구성)하고 N-stage 배치 평가를 가능케 해 해소한 점이 delta. 추가로 zoRO를 zero-order GP-MPC(zoGPMPC)로 확장, 효율적 공분산 전파·온라인 GP 업데이트를 모듈형 오픈소스로 제공하고 풀스케일 차량 실시간 시연. novelty는 신규 이론보다 통합·성능·일반성에 있음.

**6. Limitation·Future work.** 저자: 현재 학습 모델은 동역학에 한정(비용·제약 학습 미지원), L4acados는 JIT 미사용으로 N<10 소구간에서 L4CasADi 대비 불리, GP 온라인 업데이트 O(D^2) 스케일링으로 대용량 데이터셋 제약. 추가로 내가 보는 한계: 자체 수렴/안정성·robust feasibility 형식 보장 부재, tightening 자코비안 동결의 보수성 정량 분석 부족, 검증이 비-항공우주(지상차량)에 국한되어 G&C(강하·재진입 등) 직접 적용성은 미입증.

**평가:** 82/100 (strong) · 한줄평: acados 생태계에 학습 기반 모델·GP-MPC를 실시간·오픈소스로 견고히 끌어들인 고품질 도구/제어이론 논문이나, 신규 이론은 얇고 검증이 지상차량에 한정되어 G&C 관련성은 전이 가능한 솔버 도구로서 간접적이다.

---

## (2026, JGCD, Transformer-Based Atmospheric Rocket Landing Guidance Using Meta-Reinforcement Learning: Carradori·Sagliano·Mooij)

**1. 핵심 아이디어.** 재사용 발사체 1단의 대기권 동력 착륙(terminal powered landing)을 위해, Gated Transformer XL(GTrXL) 어텐션 신경망을 메타강화학습(meta-RL, PPO)으로 학습시켜 통합 유도·제어(G&C) 정책을 얻는다. 6자유도(병진+회전) 강체 모델, 공력 핀, TVC, 가변 질량/관성/CoM, 바람(HWM14)·돌풍·항법/제어 오차·초기조건 분산을 모두 포함하는 고충실도 환경에서 학습하여, 다양한 분산 시나리오에 일반화되는 강건 피드백 정책을 생성하는 것이 목표다. 종말 수직속도 제약만 만족이 어려워, 마지막 수 m 구간에서 추력을 균일가속 운동방정식으로 보정하는 "terminal patch"를 추가로 도입한다.

**2. 알고리즘.** 정책 출력은 7개 비율 명령 u=[Ṫ, ε̇_T, ψ̇_T, β̇_f,pitch1, β̇_f,pitch2, β̇_f,yaw3, β̇_f,yaw4](추력률·TVC 2축률·핀 4개 편향률). 학습은 PPO 클립 목적함수 J^CLIP=E[min(r_t A, clip(r_t,1-ε,1+ε)A)], r_t=π_θ(a|s)/π_θ_old(a|s), 전체 목적 J^PPO=J^CLIP - c1(V_θ-V_targ)² + c2·H. 메타-RL 목적은 태스크 분포 P(T)에 대한 기대누적보상 J(θ)=E_{T~P(T)}[E_τ[G_T(τ)]] 최대화. GTrXL은 길이 L 관측을 n 단위 τ 세그먼트로 나눠 마스크드 멀티헤드 어텐션 A(Q,K,V)=softmax(χ(QK^T)/√d)V로 처리하고, 메모리 M_τ=[H^j_{τ-T},...,H^j_{τ-1}]로 과거 T 세그먼트를 유지하여 RNN 유사 메모리를 구현. 보상은 위치(로그형 dense)·속도(지수 기준프로파일 편차)·각속도·수직각·연료의 dense 항과, 종말 제약 충족 시 로그 보너스/위반 시 2차 페널티의 terminal 항을 ε-constraint로 점진 강화: R=R_pos+R_vel+R_|ω|+R_δ+R_mass+(ε0/ε)R_terminal.

**3. 비판적 검토.**
- 핵심 방법(meta-RL + 트랜스포머 유도)은 선행연구(Federici & Furfaro [26] 6-DOF 행성착륙, Briden [23], Guffanti [24])에서 이미 확립되어 있어, 본 논문의 신규성은 "더 현실적인 대기권 6자유도 문제로의 확장"에 한정된다. 방법론 자체의 발명은 아니다.
- 종말 수직속도 제약을 정책 단독으로 만족시키지 못하고(평균 약 -3.4 m/s, 일부 -2~-5 m/s) 추가 terminal patch에 의존한다. 이는 보상함수 항 간 충돌(저고도 hovering 보상 vs 속도 감속)의 미봉책이며, 학습된 정책의 본질적 한계를 드러낸다.
- 보상함수가 수십 개 항/가중치의 수작업 trial-and-error 튜닝(수 주 소요)에 의존하며, 저자 스스로 이를 AI 방법론의 주요 단점으로 인정. 이론적 수렴 보장이 없고 국소최소해 가능성을 명시.
- V&V 부재를 저자도 NASA interagency 보고서를 인용해 문제로 지적했으나, 본 연구 자체는 형식적 검증 없이 MC 캠페인에만 의존한다.
- 단일 미션 단계(종말 착륙)만 검증. downrange/RTLS 프로파일 독립이라 주장하나 실제 다양한 진입 조건 일반화는 미입증.

**4. 시뮬/실험.** 6자유도 고충실도 시뮬, 1000-run MC. 초기조건: 고도 2000 m, 수직속도 -100 m/s, 질량 4000 kg, roll/pitch/yaw 분산 등(Table 1), 밀도±10%/공력±15% 분산, HWM14 바람+50% 확률 1-cosine 돌풍, 가우시안 항법/제어 오차. 결과(Table 6, GTrXL): 수평위치 1.01±0.55 m(<10 m 제약 1000/1000 충족), 수평속도 0.23±0.12 m/s(<1.5), 각속도 0.55±0.28 deg/s(<3), 수직각 0.82±0.37 deg(<3) 모두 100% 충족; 수직속도 -3.39±0.80 m/s(<2 m/s 위반). Patch 적용 시 수직속도 -0.83 m/s, 997/1000(99.7%) 충족하며 타 제약 불변, 연료 536→544 kg(소폭 증가). Baseline: LSTM(48.9% 타제약 성공률, 각속도 4.39 deg/s로 크게 열등), OCP+LQR(외부 최적제어, 연료 7%↓ 502 kg이나 종말제약 충족 390/1000으로 빈약). 온보드: i7-8565U에서 forward pass ~6 ms (sim).

**5. Contribution(delta).** 출판년(2026) 기준 SOTA에 비추어, (1) 트랜스포머/메타-RL 유도를 행성착륙(저충실도)에서 대기권 1단 6자유도(완전 회전동역학+공력핀+TVC+가변 MCI+HWM14 바람)로 확장하여 현실성을 크게 높였고, (2) GTrXL vs LSTM 정량 비교로 어텐션 구조의 우월성(100% vs 48.9% 성공률)을 실증했으며, (3) 관례적 open-loop 비교가 아닌 closed-loop OCP+LQR과 공정 비교, (4) terminal patch로 가장 까다로운 수직속도 제약을 99.7%까지 해소. 다만 방법론적 발명이 아닌 현실적 문제 적용·실증이 delta의 본질이다.

**6. Limitation·Future work.** 저자: 수렴 보장 부재·국소최소해 가능성, 보상 튜닝 수작업 비용, 수직속도 제약의 정책 단독 미충족(patch 필요), ML G&C의 체계적 V&V 부재. 내가 보는 추가 한계: 신규성이 방법보다 적용에 치우침; 단일 착륙단계만 검증해 진입 다양성 일반화 미입증; OCP+LQR baseline이 비강건(분산 미반영)으로 설계되어 비교가 RL에 유리하게 편향; patch가 균일가속 가정에 의존해 강한 외란/고고도 활성화 시 회전 제약 악화 위험 자인. Future: 학습기반 V&V 프레임워크, 자동 보상설계, 수직속도 제약의 네트워크 내재화.

**평가:** 69/100 (good) · 한줄평: 방법은 선행에서 가져왔으나 대기권 6자유도 현실성·1000-run MC·외부 baseline·온보드 타이밍을 갖춘 견실한 core-G&C RL 착륙유도 연구로, 수직속도 patch 의존이 본질적 약점.
