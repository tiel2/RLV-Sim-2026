# 스레드 B — RLV 착륙·재진입 GNC 응용 (30편)

> 스레드 B = 재사용발사체(RLV) 착륙·재진입 유도제어 응용 도메인(convex/heuristic/desensitization/dispersion 계열).
> 점수 엄격 재교정: 5=예외적·분야선도(드묾)·4=명확히 평균이상·3=견실/보통·2=약함·1=없음·N/A=scope 밖. '작동한다'는 대부분 3, '전 기준 4'≠80.
> 공정성: post-hoc fudge·venue-prestige offset 전부 제거; Total은 공식(Σwᵢsᵢ/Σwᵢ×20) 결과 그대로. 결정론 backbone은 C4=N/A 또는 2(점추정)·sim-only는 C7 비감점.

---

## (2020, JGCD, Adaptive and Efficient Model Predictive Control for Booster Reentry: Chai et al.)

**1. 핵심 아이디어.** 재사용 부스터의 재진입 종방향(받음각) 제어를 위해 Algebraic MPC(AMPC, 단일 예측점·고유분해 기반 천이행렬로 closed-form 이득 산출)와 L1 적응 증강을 결합한다. AMPC로 LTV 모델의 최적 추종 이득과 목표 폐루프 동특성 Am을 구하고, 이를 partial closed-loop 형태로 캐스팅하여 정합/비정합 불확실성에 대한 L1 적응 합성을 가능케 한다. 목표는 정확한 모델 의존성을 줄이면서 200 Hz 고속 갱신을 계산적으로 감당하는 것이다.

**2. 알고리즘.** AMPC 최적이득 K=(G^T Q G + R)^{-1} G^T Q, 자유/강제응답 F=CΦ(δt), G=CA^{-1}(Φ(δt)-I)B_m, Φ(δt)=S diag(e^{λ_i δt}) S^{-1} (절단오차 없는 정확 행렬지수). 제어법칙 u_opt=K(y_r - Fx). 전체입력 u=u_opt+u_ad를 대입해 partial closed-loop ẋ=(A-B_mKF)x + B_m(Ky_r+u_ad), Am=A-B_mKF. L1: piecewise-constant 적응법 σ̂(iT_s)=-B^{-1}Φ_ad^{-1}(T_s)μ(iT_s), 상태예측기 + 저역통과필터 C(s)=ω_c/(s+ω_c)로 적응속도와 강인성 분리. 비최소위상 H_m^{-1} 문제는 DC 이득 근사 H_m^{-1}H_um=-(CA_m^{-1}B_m)^{-1}H_um으로 대체.

**3. 비판적 검토.**
- 핵심 정리(L1 안정성·L1-norm 충분조건 Eq.30, AMPC 폐형식)는 모두 [13][26][27]에서 인용·import되며 본 논문 고유 증명은 없다. 신규 기여인 partial closed-loop 유도(Eq.11-14)는 정확하지만 초등적 대수 조작이다.
- baseline이 비적응 AMPC(self-baseline) 하나뿐이다. 외부/공표된 강인·적응 제어기와의 비교가 없어 정량 우위의 절대적 의미가 약하다.
- 단일 부스터·단일 궤적에 국한된 검증. 비최소위상을 DC 이득으로 근사한 부분은 저자도 인정한 근사이며, TDM은 해석적 유도 없이 수치적으로만 산출.
- L1 증강이 TDM을 300ms→~67ms로 크게 줄인다(강인성-성능 트레이드오프)는 점은 솔직히 보고되었으나, 이는 실제 지연이 있는 임베디드에서 위험요소다.

**4. 시뮬/실험.** Mach 5/26 km에서 시작하는 LTV 종방향 모델, 받음각 ramp 풀업 기동, 200 Hz. 5개 케이스(공칭, 40%/30% 입력이득 감소+모델불일치, 정현 외란, 100회 MC) (sim). 추종오차 2-norm: 공칭 83 vs 88(거의 동일), 입력이득 40% 감소 시 baseline 202 vs L1 90(2배+), 30%+불일치 258 vs 81(3배+), 외란 223 vs 89. MC 100회 평균/표준편차 baseline 97/29.1 vs L1 90/8.0. 강인성: LTI 게인/위상여유 표(PM 125° 부근, GM ∞), 계산시간 AMPC 0.0013s·L1 0.0022s로 10-pt QP-MPC(0.0208s) 대비 ~10-15배 빠름. baseline=비적응 AMPC(self).

**5. Contribution(delta).** 출판년(2020) 기준 델타는 (a) AMPC를 LTV 부스터 재진입에 적용, (b) 비제약 AMPC를 partial closed-loop(Am=A-B_mKF)로 재구성해 정합/비정합 L1 합성을 가능케 한 점, (c) 비최소위상·비정합 입력이득을 DC 이득 근사로 처리한 점이다. L1+MPC([19]), 적응 MPC([15-18]), AMPC([13])가 모두 선행 존재하므로 novelty는 기존 구성요소의 새 도메인 결합·재캐스팅 수준의 점진적 기여다.

**6. Limitation·Future work.** 저자: HIL 미수행(임베디드 계산 타당성 미검증), TDM 감소(강인성 비용), 비최소위상 근사. 추가로 내가 보는 한계: 고유 정리 부재, self-baseline·단일 시나리오로 통계적 강도 약함, L1-norm 충분조건을 해당 plant에 대해 실제 검증하지 않고 인용에 머묾, 비정합 불확실성 추정치가 거의 비활성(Fig.3-4에서 σ̂2≈0)이어 비정합 처리의 실효성이 약하게만 입증됨.

**평가:** 67/100 (good) · 한줄평: 기존 AMPC와 L1을 부스터 재진입에 능숙히 결합해 강인성·온보드 연산성을 충실히 입증한 견실한 core-G&C 노트이나, 고유 이론·외부 baseline 부재와 낮은 피인용(1.5/yr)으로 영향력은 제한적.

---

## (2021, AIAA SciTech, Onboard Guidance for Reusable Rockets: Aerodynamic Descent and Powered Landing: Sagliano et al.)

**1. 핵심 아이디어.** 재사용 발사체(CALLISTO급 40 kN 로켓)의 하강 두 국면 — 추력이 없는 공력 하강과 추력 착륙 — 을 하나의 통일된 순차 볼록 최적화(SPCP) 틀로 온보드 유도하는 전략을 제안한다. 운동방정식을 표적 중심 Downrange-Crossrange-Altitude(DCA) 직교좌표로 표현해 경도/위도 식에 나타나는 삼각함수 비볼록항을 제거하고, 제어를 오일러각 자체가 아닌 오일러각 변화율(피치/요/롤 rate)로 잡아 시스템을 제어에 대해 affine으로 만든다. 비선형성은 중력과 공력 가속도 항에만 모이도록 구조화하고 그 부분만 수치 선형화한다.

**2. 알고리즘.** 공력 하강: x=[r_DCA, v_DCA, θ, ψ], u=[u_θ, u_ψ], ẋ = f_nc(x)+f_c(x)+Bu+Cν 형태로 분해(f_c는 상수 A_c·x, f_nc는 중력+공력만 포함). 비용은 J=w_u∫(u_θ²+u_ψ²)dt. 착륙: 상태를 [r,v,m,φ,θ,ψ,T]로 확장, 제어 [u_φ,u_θ,u_ψ,u_T], 비용 J=-w_m·m(t_F)+w_u∫uᵀRu dt (연료 최대화+제어율 평활화). 비볼록항 f_nc만 A_k=∂f_nc/∂x|_{x_k}로 선형화해 부분문제 ẋ=A_c x+A_k x+Bu+Cν+G_k를 반복(SCvx식). 가상제어 ν(인공 비실현성 회피)와 신뢰영역 ζ를 slack(s_η, s_ζ)으로 비용에 패널티. 동압/하중계수 상한(식12), glideslope 70도(식20), 종말 5초 측면 위치/속도 구속(식21)을 모두 2차 원뿔(SOCP)로 정확히 표현. 전사는 hp flipped-LGR 의사스펙트럴, 자유종단시간 t_F를 결정변수로 포함(식56 시간매핑). 초기 추정은 [24]의 단순화 모델 해를 변환해 k=0으로 사용.

**3. 비판적 검토.**
- 핵심 수렴 기계(SCvx, successive convexification)와 hp 의사스펙트럴 전사가 모두 선행연구([20][24][32][35][36])에서 수입된 것으로, 본 논문 고유의 수렴 증명이나 정식 보장은 없다. Remark 3에서 자유종단시간의 쌍선형항 선형화를 "구현 편의상 수행하지 않고 근사"한다고 명시 — 수치적으로만 확인했다고 적어 이론적 공백이 있다.
- 실시간/온보드를 표제와 초록에서 반복 주장하나 CPU 시간, 반복당 solve 시간, 문제 크기 대비 복잡도 분석이 전혀 제시되지 않는다. "potentially employed online"은 검증되지 않은 가능성에 머문다.
- 견고성 검증이 없다. 대기/공력 불확실성과 이전 국면 오차로 인한 큰 초기 분산이 이 문제의 본질이라고 본문에서 강조하면서도, 각 국면당 단일 공칭 시나리오만 풀고 Monte Carlo나 분산 캠페인이 없다. 저자도 "다양한 초기조건에 대한 검증"을 future work로 미룬다.
- 검증 baseline이 자기-일관성(RK45 재전파 오차 위치 ~2-3 m, 속도 <0.2 m/s)뿐이며, 외부 방법과의 비교가 없다. 가중치(w_vc, w_tr, w_u, w_m 등)는 수동 튜닝값이고 민감도 분석이 없다.
- 3-DOF 점질량 모델에 사후 trimming으로 6-DOF 실현성을 "확인"하는 방식이라 자세 추종 가능성은 보장이 아니라 사후 점검 수준이다(Remark 1, Fig.7/17의 fin 편향·잔여 토크).

**4. 시뮬/실험.** CALLISTO급 40 kN 로켓, 정규화 변수. 공력 하강(Table 1)·추력 착륙(Table 2) 각 1개 공칭 시나리오 (sim, 외부 baseline 없음). 결과: 모든 경계조건/구속(동압·하중계수≤1 정규화, glideslope, 자세율 ±2 deg/s 공력단계·±5 deg/s 착륙) 만족. 정확도는 최적해 대 RK45 전파 비교 — 공력단계 상대오차 위치 ~5e-5(서브미터)·속도 ~4e-4(<0.2 m/s), 착륙 위치 2-3 m·속도 <0.2 m/s. 수렴: 공력 7회·착륙 9회 반복으로 비용/증강비용이 6번째 이후 수렴, 신뢰영역 상한 1e-6 이하, t_F 변동 <1 s. MC·HIL·실비행 없음.

**5. Contribution(delta).** 출판년(2021) 기준 delta: (1) 순수 공력(무추력) VTVL 하강 유도를 볼록 최적화로 다룬 최초 시도 — Liu[25]의 2D 결합 접근을 넘어 full 3D, full 공력 데이터베이스(상수항력·무양력 등 단순화 거부); (2) 오일러각 변화율 제어 도입으로 제어행렬을 상태에서 분리하고 affine 구조 확보 + 제어율 명시적 구속으로 자세제어기 추종성 향상; (3) 공력 하강과 추력 착륙을 동일 hp-fLGR 전사·SPCP 틀로 통합하고 자유종단시간을 자연스럽게 포함. 기존 entry energy-domain[13][14][15]은 VTHL용이었고, VTVL 착륙 볼록화[19-22]는 추력 단계 위주였다는 점에서 두 국면 통합 + 공력단계 볼록화가 새 방향이다.

**6. Limitation·Future work.** 저자: 다양한 초기조건 검증, 가상제어·신뢰영역 가중치 선택 효과의 정밀 평가를 future work로 명시. 내가 보는 한계: 실시간성 주장에 대한 타이밍/복잡도 근거 부재, 견고성(MC·불확실성 전파) 부재, 외부 baseline 부재, 자유종단시간 쌍선형 항의 비증명 근사, 3-DOF→6-DOF 실현성의 사후 trimming 의존. SciTech 발표 논문 성격상 이론 보장보다 통합 프레임 시연에 무게가 실려 있다.

**평가:** 67/100 (good) · 한줄평: RLV 하강 전 국면을 단일 볼록화 틀로 통합하고 공력단계 볼록 유도를 처음 시도한 견실한 core-G&C 기여이나, 실시간성 타이밍·Monte Carlo 견고성·외부 baseline 부재로 flagship급에는 미치지 못한다.

---

## (2021, Acta Astro, Adaptive powered descent guidance based on multi-phase pseudospectral convex optimization: Song et al.)

**1. 핵심 아이디어.** 행성 연착륙/RLV 회수의 연료최적 동력강하유도(PDG)를 다상(multi-phase) 의사스펙트럴 볼록최적화(MPPCvx)로 푼다. Bang-Bang 추력 프로파일의 추력크기 스위칭 시점을 기준으로 전체 강하 과정을 위상(phase)으로 분할하고, 각 위상을 LGR 의사스펙트럴로 이산화한 뒤 볼록화한다. 핵심 차별점은 간접법(PMP)으로 스위칭함수를 해석해 연료최적 추력 프로파일이 최대 3상(max-min-max) 구조임을 보이고, 이를 이용한 적응형 3상 초기화 기법을 제안한 점이다.

**2. 알고리즘.** 동역학 r'=v, v'=g+T/m, m'=-||T||/(Isp·g0). 손실없는 볼록화로 ||u_p||<=Γ_p (비볼록 추력하한 완화), 비선형 동역학은 연속 볼록화(1차 Taylor)+trust region+가상제어(virtual control)로 처리해 Problem 3(SOCP)로 변환, ECOS/CVX로 반복해(Algorithm 1). 간접법: 해밀토니안 H=λr·v+λv·(g+Tmax·u/m·α - Tmax/(Isp·g0)·λm)로부터 Euler-Lagrange 코스테이트 방정식, 최적 추력방향 α*=-λv/||λv||, 스위칭함수 ρ=1-Isp·g0·||λv||/m-λm 부호로 Bang-Bang 결정. 스위칭함수 1차도함수가 단조성을 가짐을 보여 영점 최대 2개 -> max-min-max 3상 구조 도출. 자유종단시간은 볼록화 전 이산화로 각 위상 비행시간 t_p를 직접 변수화.

**3. 비판적 검토.**
- 핵심 볼록화 기계(손실없는 볼록화 최적성, 연속 볼록화 수렴성)는 Acikmese, Mao/Szmuk를 인용해 가져온 것으로, 본 논문 고유의 이론 기여는 PMP 기반 위상분할 해석에 집중된다.
- 온보드 실시간성을 동기로 내세우지만 계산시간(solver runtime, iteration당 ms)을 전혀 보고하지 않아 C7 근거가 약하다. ECOS 임베디드 SOCP라는 점만으로 실시간성을 주장.
- 대기항력·비관성력을 무시(중력 상수 가정)해 RLV 대기권 회수의 핵심 외란을 배제했다. "RLV recovery" 시나리오라는 동기와 모델 충실도 간 간극.
- baseline TPBVP/SCvx는 모두 저자 자체 구현이며, 외부 벤치마크나 타 연구 결과와의 직접 비교는 없다. 다만 간접법이 최적해 기준(ground truth)으로 작동하는 점은 검증력이 있다.

**4. 시뮬/실험.** Intel i7 2.6GHz. 2상 케이스(r0=[200,2000,-100], v0=[0,-20,0]): 5회 반복 수렴, 비행시간 51.0982s, 연료 5195.4595kg. MPPCvx vs 간접법(TPBVP) 연료오차 0.0003kg, SCvx는 1.2386kg 편차 -> MPPCvx의 위치/속도 적분검증오차가 SCvx보다 2자릿수 작음(3.9e-2 m, 7.9e-5 m/s). 3상 케이스(r0=[2000,2000,0]): MPPCvx 연료 5822.4211kg vs 간접법 5822.3321kg, 스위칭시점 8.3496/12.8460s vs 간접법 8.3736/12.7836s 일치. 적응성: 3상 초기화가 2상/단상으로 자동 축약(min-max, max). Monte Carlo 300샘플: 평균 위치/속도오차 4.79e-2 m, 7.997e-3 m/s, 최대 1.35e-1 m, 1.11e-2 m/s, 절반 이상 5회 내 수렴, 최대 14회. baseline: SCvx(단상 볼록), TPBVP(간접법) (sim).

**5. Contribution(delta).** 출판년(2021) 기준 볼록 PDG와 의사스펙트럴 볼록, hp 다상법(Sagliano, Wang의 MPC식 위상분할)은 이미 존재. 본 논문의 delta는 (i) 간접법 PMP로 연료최적 추력크기 프로파일의 max-min-max 3상 상한 구조를 형식적으로 증명하고, (ii) 이 구조를 활용한 적응형 3상 초기화로 위상 수를 데이터에 따라 자동 축약(불필요 위상 제거로 효율 향상)시킨 점. 즉 "어떻게 위상을 나눠야 최적인가"를 휴리스틱이 아닌 간접법 스위칭함수로 결정해 의사스펙트럴 정밀도와 효율을 동시 확보.

**6. Limitation·Future work.** 저자: 단상(full-throttle) 임계경계 케이스는 위험하여 미검증. 내가 보는 한계: 계산시간 미보고로 실시간 주장 미입증, 대기/공력 외란 배제로 RLV 회수 충실도 부족, 외부 벤치마크 부재, 평면 지형·일정 중력 가정. Future: 항력·glideslope 동적환경 확장, 임베디드 타이밍 분석, 6-DOF/자세결합 확장이 자연스러움.

**평가:** 75/100 (strong) · 한줄평: 간접법으로 위상분할 최적성을 형식 증명하고 적응형 다상 볼록 PDG로 연결한 견실한 in-domain G&C 기여이나, 실시간 타이밍 미보고와 외란 배제가 임팩트를 제한.

---

## (2021, IROS, Design, Optimal Guidance and Control of a Low-cost Re-usable Electric Model Rocket: Spannagl et al.)

**1. 핵심 아이디어.** 재사용 발사체 1단의 핵심 동역학(공력 핀, 짐벌 추력벡터)을 모사하면서 1000 USD 미만 상용 부품으로 제작 가능한 소형 전동 VTOL 테스트베드(EmboRockETH)를 설계하였다. 이 플랫폼 위에 자유종단시간 최소연료 최적유도와 오프셋프리 추종 MPC 위치제어기를 결합한 실시간 G&C 파이프라인을 제안하고, 모든 연산을 온보드(Raspberry Pi 4)에서 수행하며 실내·실외 자율비행으로 검증한다. 기여는 하드웨어 설계와 G&C 파이프라인 두 축이다.

**2. 알고리즘.** 유도: min over t_f, T(·) of ∫(||T(t)|| + λ1||Ṫ(t)||^2 + λ2 η(t)^2) dt, 3-DoF 점질량 동역학 ṗ=v, v̇=(1/m)T+g 하에서 초기/종단 위치·속도 박스제약, 글라이드슬로프 콘 e1·(p−p_ref) ≥ tan(γ)||[e2 e3]·(p−p_ref)||, 추력·추력률 박스, 틸트 e1·T ≥ cos(θ_max)||T||. 자유종단시간은 [0,1] 의사구간으로 사상하여 forward Euler 30점 이산화. 위치제어: 6-상태(p,v,q,ω,T) 비선형 모델을 RK4(4중간노드, 25Hz)로 이산화, 외란을 상수 0차 동역학으로 모델링해 EKF로 상태와 오프셋 w를 추정. MPC: min Σ l_i(x,u,w,r) + l_f, 입력 폴리토프제약 Au≤b (짐벌각·추력 한계), 비용항에 추정 외란 dv, da를 보정한 오프셋프리 항. 저수준 PID(자세 250Hz, 자세율 1kHz) + 제어할당(τ,T_x → 짐벌각 θ1,θ2 및 DC모터 PWM, 3차 다항식 맵).

**3. 비판적 검토.**
- 유도 정식화는 [20][21](Pascucci, Acikmese의 free-final-time mars landing)을 본 플랫폼에 적응한 것으로, 정식화 자체의 신규성은 제한적이다. 무손실 볼록화 같은 최적성·실현가능성 보장 논의가 없고, 비선형 SQP(FORCES)로 국소해만 구한다.
- 이론적 기여보다 시스템 통합·실증에 무게가 실려 있다. MPC 안정성/재귀적 실현가능성에 대한 형식적 보장이 없으며, 오프셋프리 추종도 표준 외란증강(Borrelli-Morari [23]) 기법의 적용이다.
- 정량 평가가 약하다. 외부 baseline과의 비교가 전무하고, Monte-Carlo나 통계적 처리가 없으며 단일 시나리오 데모 위주다. 착륙 정확도 50cm 단일 수치만 보고된다.
- 핀(공력면)이 제어할당에서 제외되어 향후 과제로 남아, 재사용 발사체의 핵심 차별점인 공력제어가 실제로 검증되지 않았다.

**4. 시뮬/실험.** 세 종류 실험: (1) 모터 명령맵 식별(로드셀), (2) 실내 Vicon 비행 — set-point A↔B 추종, 오프셋프리 MPC가 CoG-바리센터 정렬오차 보정, 배터리 0.3V 강하 중에도 고도 유지(da_x 적응) (real). (3) 실외 GPS 비행 — 고도 10m 상승 후 병진+하강 기동, 최대속도 3m/s, 목표 50cm 이내 착륙 (real). 온보드 평균 solve time: 유도 120ms, MPC 30ms (Raspberry Pi 4). 외부 baseline·MC·통계 없음.

**5. Contribution(delta).** (i) 재사용 1단의 짐벌 추력벡터+공력 핀 동역학을 모사하는 1000 USD 미만 소형 전동 테스트베드 설계는 기존 quadrotor 기반 G&C 검증 플랫폼([15][16])보다 발사체 동역학에 가까운 새로운 기여다. (ii) 최소연료 자유종단시간 유도 + 오프셋프리 MPC + 제어할당을 온보드 실시간으로 통합해 실내외 완전자율 비행으로 검증한 점. 출판년(2021) 기준으로 알고리즘 각 요소는 기존 기법의 조합·적응이며 신규성은 주로 저비용 통합 플랫폼과 실증에 있다.

**6. Limitation·Future work.** 저자: 핀의 제어할당 미포함(향후 과제), 배터리 전압 변동의 맵 미반영(MPC 오프셋으로 흡수). 내가 보는 한계: 형식적 보장(유도 최적성/실현가능성, MPC 안정성) 부재, 외부 baseline 비교·MC·통계 처리 부재, 단일 데모 수준 정량평가, 3-DoF 유도와 6-상태 추종 모델 간 일관성·자세 proxy 가정의 영향 미분석, 공력제어 미검증으로 실제 발사체 G&C 일반화에는 거리가 있다.

**평가:** 63/100 (fair) · 한줄평: 저비용 발사체-모사 테스트베드와 온보드 실시간 G&C 통합·실증은 견고하나, 알고리즘 신규성·이론적 보장·정량적 baseline 비교가 약해 in-domain 시스템 논문으로서 fair 수준이다.

---

## (2022, IJASS, Integrated Optimal Guidance for Reentry and Landing of a Rocket Using Multi-Phase Pseudo-Spectral Convex Optimization: Hwang·Ahn)

**1. 핵심 아이디어.** VTVL 재사용 발사체의 재진입~착륙 전 구간을 재진입 연소(reentry burn)·코스팅(coasting)·착륙 연소(landing burn)의 다단계(multi-phase) 연료최적 최적제어 문제로 통합 정식화한다. 기존 연구가 착륙 연소(powered descent)에 집중했던 것과 달리, 재진입 연소와 자유낙하 구간까지 하나의 궤적 최적화로 묶어 전체 최적성과 지상 안전을 동시에 고려한다. 비볼록 원문제를 flipped-Radau 의사스펙트럼(fRPM) 이산화 + 축차 볼록화(successive convexification)로 변환해 온보드 실시간 후보 알고리즘을 제시한다.

**2. 알고리즘.** 무차원 2D 점질량 동역학(r,φ,v,γ,m)에 추력크기 T·전단각 α를 제어로 사용. 비선형 동역학을 control-affine으로 만들기 위해 u1=Tcosα/m, u2=Tsinα/m, σ=T/m, μ=log m 도입 → x_dot=f(x)+B(x)u. 자유종단시간 문제를 각 phase 비행시간(Δt^k)에 대해 1차 선형화: x_dot=(A x^k + B u^k + c)Δt^k + f(Δt-Δt^k). 비볼록 IIP 안전 제약(φ(t1f)+re·φ_IIP≤0)과 추력방향 제약은 Taylor 전개로 선형화, 등식제약은 가상제어 ε로 완화. 목적함수에 패널티 λ1·ε^Tε + λ2·δ^2 (trust region) 추가, 수렴 시 |J^(k+1)-J^(k)|/J^(k)<Jc 및 Σε^Tε<εc로 종료. CVX+MOSEK로 SOCP 풀이.

**3. 비판적 검토.**
- 핵심 수치 기법(fRPM은 Garg/Wang, 축차 볼록화·가상제어 수렴성은 Mao et al.)이 모두 인용·차용이며, 통합 문제에 대한 자체 수렴성 정리나 무손실 볼록화(losslessness) 증명은 없다. 엔지니어링 조립 수준의 기여.
- 검증이 단일 명목 시나리오 1건에 그치고, baseline이 외부 벤치마크가 아니라 저자 자신이 돌린 GPOPS-II 오프라인 해(self baseline). MC·분산 해석·통계가 전무.
- 불확실성 처리가 빈약하다. IIP 제약이 엔진 재점화 불확실성·지상 안전을 개념적으로 다루지만 정량적 강건성(바람, 초기조건 분산)은 다루지 않으며 저자도 바람 강건성을 future work로 명시.
- 비볼록 IIP 제약의 1차 선형화 완화가 보수성/타당성 측면에서 더 정량 분석될 여지가 있다.
- 온보드 주장 대비 실측은 데스크톱 i9-9900K + CVX/MOSEK 환경의 초 단위 계산시간(2.67/4.67s)에 머물러, 임베디드 실시간 실증은 아니며 저자도 onboard 구현 심층연구가 필요하다고 인정.

**4. 시뮬/실험.** SpaceX Falcon 9 제원(건조질량 22,200kg, 추진제 25,000kg, 추력 93.2tonf×3 재진입/93.2 착륙, Isp 311s, Cd 1.5) 기반 단일 RTLS형 시나리오 (sim). 초기 고도 70km·v=1400m/s에서 착륙(touchdown 5m/s, γ=-89°, 전단각 180°) (sim). 노드수 30/30/60. 3방식 비교: 오프라인(GPOPS-II) CPU 14.65s/최종질량 25,784kg, convex-uniform 2.67s/25,665kg, convex-pseudo-spectral 4.67s/25,721kg. 상태·제어 이력이 세 방식 간 거의 일치, 의사스펙트럼이 오프라인 대비 동등 정확도+빠른 계산. 외부 baseline·MC 없음.

**5. Contribution(delta).** 선행 대비 delta는 (1) 착륙 연소 위주였던 RLV 볼록최적 유도를 재진입 연소·코스팅 포함 전 구간 통합 다단계 문제로 확장하고, (2) IIP 기반 지상안전 제약을 통합 정식화에 포함하며, (3) Sagliano(2018)·Wang&Cui(2018)의 의사스펙트럼-볼록화를 다단계·재진입까지 확장한 점. 2022년 SOTA 기준 가장 가까운 선행(powered descent 의사스펙트럼 볼록화)에 대한 명확한 증분적 확장이며, 새로운 방향을 여는 수준은 아니다.

**6. Limitation·Future work.** 저자: 실제 온보드 컴퓨터 구현·계산시간 심층연구 필요, 바람(2단계 상층풍/최종단계 하층풍) 강건성 해석 필요. 추가로 내가 보는 한계: 2D 점질량·비회전 원형지구 가정으로 측면운동/3DOF·자세동역학 미반영, 단일 시나리오·self baseline·MC 부재로 통계적 일반성 미확보, 비볼록 제약 선형화의 수렴 보장 미증명.

**평가:** 61/100 (fair) · 한줄평: 재진입~착륙 통합 다단계 의사스펙트럼-볼록화를 깔끔히 조립한 견실한 in-domain G&C 논문이나, 자체 이론 증명·강건성 검증·외부 baseline·실시간 실증이 부족한 증분적 기여.

---

## (2022, Acta Astro, Model predictive control for reusable space launcher guidance improvement: Guadagnini et al.)

**1. 핵심 아이디어.** 재사용 발사체의 동력 하강 유도(PDG)를 6-DoF 비볼록 최적제어 문제로 정식화하고, 이를 successive convexification(SCvx)으로 SOCP화하여 온보드 재계산이 가능한 MPC형 유도로 구현한다. 가변 대기 모델과 가변 중력장을 문제에 포함시키고, 두 가지 G&C 아키텍처(MPC-유도 + PID-제어 vs 고빈도 MPC-G&C)를 산업용 비선형 시뮬레이터에 연결해 강건성과 실시간 구현 가능성을 비교 평가한다.

**2. 알고리즘.** 비볼록 자유종단시간 최적제어: minimize J = t_f s.t. 질량감쇠 m_dot = -||T_B||/(I_s g0) - beta_m, 병진 r_dot=v, v_dot=(1/m)R(T_B+A_B)+g_I, 회전 쿼터니언/Euler 강체 동역학. 제약: 글라이드슬로프 콘, 틸트각, 각속도 한계, ||q||=1, 받음각 STC(동압 q>=q_dyn일 때 -e1·v_B>=cos(alpha_max)||v_B||), 추력 T_min<=||T_B||<=T_max, 짐벌각. SCvx: 이전해 주위 1차 Taylor 선형화 -> ZOH 이산화 -> LTV, 가상제어 nu로 dynamics relaxation, trust region(||δx||,||δu||,||δσ||) 페널티. 자유종단시간은 tau∈[0,1] 정규화 + dilation σ=(dτ/dt)^-1로 고정종단시간화. 비볼록 제약(쿼터니언 norm, 받음각, 추력 하한)도 1차 전개로 볼록화.

**3. 비판적 검토.**
- 핵심 수학적 기계장치(SCvx 수렴성, STC 정식화, 받음각/공력 모델)는 Mao·Acikmese·Szmuk(refs 6,7,15,16)에서 그대로 가져왔고, 이 논문 자체의 정리나 수렴 증명은 없다. 검증은 알려진 해석해 문제에 대한 적용으로 인용 처리.
- 실시간성 주장이 핵심 동기인데 실제 타이밍/연산시간 측정이 없다. MATLAB 구현이며 C++/mex 변환은 future work로 미룸. 두 번째 아키텍처의 0.1Hz 이상에서의 수렴 실패율(18%/50샘플)만 보고.
- 베이스라인이 자기 자신의 두 아키텍처뿐이다. G-FOLD/lossless convexification, Szmuk SCvx 등 외부 방법과의 정량 비교가 없어 우위 주장이 상대적이다.
- throttle rate(추력 변화율) 제약을 "문제 단순화"를 위해 제외하고 사후 점검으로 대체 — 실제 엔진 제약 측면에서 정당화가 약하다.
- 신규 기여인 가변 대기/중력의 영향이 별도로 ablation되지 않아 그 효과가 정량화되지 않는다.

**4. 시뮬/실험.** 6-DoF 비선형 시뮬레이터(DEIMOS-FES Atmospheric Flight Simulation Core 대비 검증, NRL 바람/거스트 모델 포함)에서 평가 (sim). 초기조건 r=[50,-30,0]km, 명목 궤적 t_f~80s. 민감도: 초기조건/발사체파라미터(질량±2~5t, COM shift, 추력 정렬오차)/대기조건(밀도·압력±10%, 거스트±50m/s)/최악조건 4케이스, 아키텍처1은 100샘플·아키텍처2는 50샘플 MC. 결과: 최종위치 평균오차 아키텍처1 ≈23.50m, 아키텍처2 ≈51.74m; 최종속도 3.05 vs 6.26 m/s. 50m/3m/s 이내 성공률로 비교. 아키텍처2는 수렴실패 18%. baseline은 두 자체 아키텍처(외부 baseline 없음).

**5. Contribution(delta).** SCvx 기반 6-DoF PDG는 출판년(2022) 기준 이미 확립된 방법(Szmuk 2018, Mao 2017, Acikmese)이므로 방법 자체는 신규가 아니다. 본 논문의 delta는 (i) 가변 지수 대기 모델 + 가변 중력장을 PDG 정식화에 통합, (ii) MPC-유도+PID vs 고빈도 MPC-G&C 두 아키텍처를 산업용 검증 시뮬레이터 상에서 대규모 민감도/최악조건으로 정량 비교하고 SCvx가 receding-horizon에 부적합함을 실증한 점이다. 알고리즘적 신규성은 incremental, 응용·아키텍처 트레이드오프 분석에 실질 가치가 있다.

**6. Limitation·Future work.** 저자: 더 정밀한 대기/관성/COM·COP 모델, 받음각·마하 의존 공력 DB, throttle 제약 추가, discretization/동적 trust region 개선, C++/mex 실시간 구현·평가 필요. 추가로 본인이 보는 한계: 실제 온보드 타이밍 부재로 실시간성 주장이 미검증, 외부 SOTA 대비 베이스라인 부재로 성능 우위가 상대적, 평면회전 무시·일정 질량/관성 등 강한 가정, 신규 가변대기/중력 기여의 효과 격리 부재.

**평가:** 68/100 (good) · 한줄평: 확립된 SCvx 6-DoF PDG를 가변대기·아키텍처 비교로 견실히 응용한 in-domain G&C 논문이나, 자체 알고리즘 신규성과 실시간 실증·외부 baseline이 부족하다.

---

## (2024, AIAA SciTech, Powered Atmospheric Landing Guidance for Reusable Rockets: the CALLISTO studies: Sagliano et al.)

**1. 핵심 아이디어.** DLR/JAXA/CNES 공동 재사용 로켓 실증기 CALLISTO의 대기권 내 동력 착륙(Powered Atmospheric Landing Guidance, PALG) 유도 문제를 다룬다. 진공 가정의 고전적 동력 하강 정식화(Acikmese-Ploen 등)와 달리 실측 기반 룩업테이블 형태의 대기밀도·공력계수·음속을 그대로 반영하고, 연료최적성에서 출발해 안전성·운용제약·6-DoF 트림 가능성까지 점증적으로 포함시킨 5개 정식화(Mark I~V)를 제안한다. 각 Mark는 Sequential Pseudospectral Convex Programming(SPCP)으로 풀어 성능 대 안전성의 트레이드오프를 비판적으로 비교한다.

**2. 알고리즘.** 3-DoF DCA(Downrange-Crossrange-Altitude) 좌표계에서 비용 J=∫T_vac dt(Mark I) 최소화. 상태 r,v,m; a^grav(중심체)+a^thr=û·T_atmo/m + a^aero=R_BODY^DCA·F_BODY^aero/m + a^fict. T_vac=T_atmo+p_atmo·A_nz로 대기 추력손실 반영. û^Tû=1는 Augmented Convex-Concave Decomposition으로 처리. 글라이드슬로프는 2차 원뿔 ([r_y,r_z], r_x/arctanγ_gs)∈K^3, 꼬리 받음각 α_T∈[-2,2]deg로 트림성 보장. Mark II는 제어를 [T_atmo,α,β,ε1,ε2]로 재정의(공력각+TVC), Mark III은 각속도 율 상한 추가, Mark IV는 추력율 u_T를 상태로 승격하고 최종 수평위치를 소프트 제약(slack s_D,s_C)으로 완화, Mark V는 고도임계(h≤h_land→T=T_land)·동압임계(q_dyn≤q*_dyn→핀 비활성) 제약을 LCP 해석해(variable-triggered constraints, v=-min(g,0)·c≤0)로 부과. hp flipped-Legendre-Gauss-Radau(fLGR) 이산화 + 가상제어 ν 페널티(w_μ=10^4)로 인공 비실현성 방지.

**3. 비판적 검토.**
- 불확실성에 대한 정량 검증이 전무하다. 본문은 공력계수 불확실성이 설계 동기라고 반복 언급하나, Monte-Carlo도 강건성 분석도 없고 모두 future work로 미뤄졌다. "Mark II는 위험/Mark V는 안전"이라는 안전성 판정(Table 1)이 정성적이며 불확실성 하 검증으로 뒷받침되지 않는다.
- 온보드 유도를 표방하나 실제 CPU 시간·실시간성 측정이 보고되지 않았다. fLGR의 large-n/low-p가 온라인용이라는 언급만 있고 타이밍은 결론에서 future work로 명시된다. 따라서 C7 deploy 주장은 미검증.
- 핵심 수치 기법(SPCP, fLGR, ACCD[10], state-triggered/variable-triggered[11], LCP[12])이 모두 선행 연구에서 도입된 것을 조합·적용한 것으로, 새로운 수렴 정리나 자체 증명은 없다.
- baseline이 외부 방법이 아니라 5개 Mark 간 자기비교이며, 각 Mark는 단일 공칭 시나리오 1회만 제시된다.
- 3-DoF 해의 6-DoF 양립성을 α_T∈[-2,2]deg 등 트림 제약으로 "보장"한다고 하나, 실제 6-DoF 시뮬레이터 검증은 제시되지 않고 Remark 1의 의도 표명에 머문다.

**4. 시뮬/실험.** RTLS 데모 프로파일(고도 ~6.5km에서 착륙, 비행시간 ~33-37s) 단일 공칭 궤적에 대해 5개 Mark를 각각 (sim) 제시. 정량 비교는 Table 1: Mark I 기준 추가연료 II −7.2%/III +0.6%/IV +2.9%/V +19.4%, 안전성(II 나쁨→III~V 좋음), 수렴성(II 나쁨→III~V 좋음), 정식화 강건성(I·II 나쁨→IV·V 좋음). Mark V의 동압 ~35kPa 피크, α_T≈180deg 근방 유지, TVC각 ε≤0.4deg, 핀편향 |δ|≤2deg로 트림·기계제약 양립 확인. 외부 baseline·MC·통계 없음.

**5. Contribution(delta).** 출판년(2024) 기준, 진공 동력하강 정식화를 실측 기반 대기/공력 모델로 확장하고, 연료최적→안전→운용제약→트림 가능성을 단계적으로 통합하는 체계적 Mark 방법론을 제시한 점이 delta다. 특히 고도/동압 임계 제약을 LCP 기반 variable-triggered constraint로 atmospheric landing에 적용한 점과, 공력각·TVC·추력율을 제어/상태로 명시 파라미터화해 6-DoF 트림 가능한 3-DoF 해를 얻는 정식화는 진공 G-FOLD류 대비 실용적 진전이다. 다만 신규 알고리즘이라기보다 기존 convex/pseudospectral 도구의 통합·실증 적용이다.

**6. Limitation·Future work.** 저자 인정: 온보드 CPU 시간 측정 미실시, 초기추정 생성 전략 미개발, 강건성 평가용 대규모 MC 미수행, 고정밀 6-DoF 비선형 시뮬레이터와의 병합 검증 미완. 추가로 본 평자가 보는 한계: 외부 baseline 부재로 SPCP 자체의 우월성이 입증되지 않음; 안전성 판정의 정량 기준 불명확; 단일 시나리오로 일반화 주장 근거 약함; variable-triggered 제약의 수렴/feasibility 보장에 대한 이론적 분석 부재.

**평가:** 67/100 (good) · 한줄평: 실측 대기·공력을 반영한 재사용 로켓 착륙 유도의 체계적 정식화 비교로 실무 적합성은 높으나, 불확실성·실시간성 정량 검증이 빠진 통합·실증형 core-G&C 연구.

---

## (2025, AIAA SciTech, Ignition Point Reachability for Aerodynamically-Controlled Reusable Launch Vehicles: Chung·Echigo·Açıkmeşe)

**1. 핵심 아이디어.** 공력 제어가 가능한 재사용 발사체(RLV)의 1단이 공력 하강에서 추진 하강으로 전환(점화)하는 "점화점"의 도달가능 집합을 계산한다. 11차원(5-DoF) 상태·제어 제약을 모두 갖는 고차원 도달가능 문제를 Hamilton-Jacobi나 집합전파로 풀기 불가능하다는 점에 착안해, 관심 있는 3차원(위치) 부분공간으로의 투영만을 반복적 궤적최적화(SCP) 표본추출로 근사한다. 결과적으로 안전한 점화가 가능한 공간 체적(폴리토프)을 산출하여 임무계획·범위안전에 활용한다.

**2. 알고리즘.** 축대칭 강체의 5-DoF 동역학(질량고갈, 병진, 2각 오일러 자세, 각속도, 관성텐서 선형보간)을 비차원화·다중슈팅으로 이산화하고, prox-linear 기반 SCP(정확 L1 페널티 신뢰영역 + Mao et al. 적응가중)로 비선형 궤적최적화(P1)를 푼다. 핵심 기여 중 하나는 축대칭 본체의 양력/모멘트 특이점(α=0)을 csc(α)를 룩업테이블에 흡수시켜 제거한 정식화: C_l,mod(α,M)=C_l·csc(α) (α≠0), L_I=½ρ C_l,mod v_I×(b̂_z×v_I). 도달가능 분석은 Chan의 defect hull을 사용 — 폴리토프 표면점 n_{I,o}에서 무작위 방향 d̂로 max μ s.t. n_I=r_I(0.5)=μd̂+n_{I,o} (P2)를 풀어 가장 먼 가용 점을 찾고 P_{i+1}=convexhull(P_i, n_I*)로 확장. CTCS로 연속시간 제약위반 억제, 공력/추진 두 시간팽창 파라미터 τ_a,τ_p로 2상 전환 시점을 자유화.

**3. 비판적 검토.**
- 저자 스스로 명시하듯 산출된 체적이 참 도달가능 집합의 상/하근사임을 보장하지 못한다(비볼록성으로 hole·오목·분리집합 가능). 즉 "탐색된 점은 도달가능"만 보증되는 표본추출적 근사이며, 안전 보증용으로 쓰려면 보수성 정량화가 빠져 있다.
- 검증이 단일 시나리오(KSP Kerbin 귀환 1궤적)에 국한되고 외부·독립 baseline과의 정량 비교가 없다. 진공/무양력 정식화 대비 정성적 관찰(swerve, 비추진 양력감속)만 제시.
- defect hull 표본추출이 naive(Chan의 dual tangent 정보 미사용)하여 국소최소에 자주 빠지고, 10,000 반복·4시간 20분이 소요. 온보드/실시간과는 거리가 멀다.
- 불확실성·분산(공력계수 오차, 바람) 처리가 없어 nominal 모델 한정. 범위안전이라는 동기와 비교하면 robustness가 약하다.
- KSP/Ferram 공력은 medium-fidelity이나 실제 비행·고충실도 CFD가 아니며, Kerbin의 비현실적 물리(빠른 자전·중력감쇠)로 지구 시나리오 일반화는 별도 검증 필요.

**4. 시뮬/실험.** 규모: KSP(Ferram Aerospace) medium-fidelity 6-DoF 시뮬레이터의 stock 부품 발사체, Falcon 9 post-retropropulsion에 견줄 만한 귀환 시나리오 1건. 초기화로 min-fuel·min-time 궤적(N=41 노드)을 SCP로 수렴(약 25~30 iter), 점화점 폴리토프를 10,000 반복 확장하여 체적 ~1.0×10^10 m^3, AMD Ryzen AI 9 HX 370 32스레드에서 총 4시간 20분(서브문제당 평균 1.63 s). 생성 해 중 4.3%는 동역학 비가용, 0.02%는 수치불안정으로 기각. 정량 baseline: 외부 baseline 없음(자체 vacuum/lift-free 정식화와 정성 비교만). (sim only)

**5. Contribution(delta).** 출판년(2025) 기준 SOTA 대비: (i) 완전한 5-DoF + 전체 상태·제어 제약을 갖는 RLV에 대해 점화점 도달가능 집합을 계산한 최초 사례(선행 Eren·Açıkmeşe 2015는 볼록 동역학, Chan 2022 thesis는 단순 외기권 lander). (ii) 궤적최적화 표본추출과 defect hull을 결합해 고차원·비선형 도달가능 문제를 저차원 투영으로 근사하는 틀. (iii) 축대칭 본체 공력의 양력/모멘트 특이점을 csc(α) 흡수로 제거한 최적화-친화적 정식화(원 룩업테이블 대비 <2% 오차). 단, 알고리즘 구성요소(prox-linear, CTCS, defect hull, Möller-Hughes)는 대부분 기존 기법의 조합이라 정리 수준의 새 이론은 아니다.

**6. Limitation·Future work.** 저자: 상/하근사 미보증, 느린 수렴(naive 표본추출), convex parsing 오버헤드 ~30%, 향후 customized sparse 구현·Auto-SCvx·GPU 병렬화로 분~초대 단축, CVaR 확률적 도달가능·다상 확장 제안. 추가로 내가 보는 한계: 불확실성/분산 robustness 부재, 단일 시나리오·외부 baseline 부재로 일반화·우월성 입증 미흡, medium-fidelity·비현실 행성 검증으로 실비행 적용성 불명확, 폴리토프가 안전 보증용 도구로 쓰이기엔 보수성 정량화가 빠짐.

**평가:** 68/100 (good) · 한줄평: 완전제약 5-DoF 공력 RLV 점화점 도달가능이라는 실제 임무계획 공백을 정직하게 다룬 견실한 core-G&C 기여이나, 근사 보증·외부 baseline·실시간성·불확실성 처리가 빠져 영향력은 아직 잠재적 수준.

---

## (2025, ASR, Fuel-optimal boost-back guidance algorithms for reusable launch vehicles: Jo·Han·Ahn)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 boost-back 연소 구간을 위한 두 가지 해석적(준 반복) 유도 알고리즘을 제안한다. (1) 기존 IIP(순간 충돌점) 유도(Jo & Ahn 2018)에 비행경로각(FPA) 변화율 제어를 추가하여 RTLS 시나리오에서의 성능 저하를 해소한 IIP-FPA 유도, (2) hit equation 기반으로 최소 속도증분(Δv)을 만드는 목표 속도를 먼저 계산하는 최소충격(MIG) 유도이다. MIG는 반복변수를 FPA로 두는 방식과 TOF(Lambert 문제)로 두는 방식 두 가지로 구현된다. 둘 다 추진제 소모 최소화를 목표로 DRL/RTLS 모두에 적용 가능하다.

**2. 알고리즘.** IIP-FPA: 최단경로 조건 식(16)과 목표 FPA 변화율 식(17~18)을 동시에 만족시키도록 가속도 명령 a_T,1/a_T,2/a_T,3 (식 20~22)을 비반복 해석식으로 산출. MIG: hit equation r0/rt = (1-cos φd)/(r0 vd^2/μ · cos^2 γd) + cos(φd+γd)/cos γd 로 Δv를 γd만의 함수 식(33)으로 표현, ∂(Δvd^2)/∂γd=0 (식34)을 Nelson-Zarchan 경계 식(35) 내 이분법으로 풀어 최적 γd*, vd* 도출. 지구자전에 따른 ECI 표적 이동은 targeting 외부루프(식37~40)로 보정. 접근2는 TOF에 대한 golden-section 탐색으로 Lambert 해를 선택. 추력명령 a_T = (Δvd*/|Δvd*|) a_m (식41).

**3. 비판적 검토.**
- 기여가 자기 선행연구(Jo & Ahn 2018/2020) 라인의 점진적 확장에 가깝다. IIP-FPA는 RTLS 하강궤적 문제를 고치는 실용적 패치이고, MIG의 hit-equation/Lambert 적용은 고전 궤도역학의 재활용이라 방법론적 신규성의 폭이 제한적이다.
- IIP-FPA는 목표 FPA 변화율(γ̇t=0.6 deg/s)을 오프라인 궤적최적화로 사전 결정해 외부에서 주입한다. 이 참조값 의존이 RTLS에서 적응성/최적성을 떨어뜨리며(최적성 갭 3.9%), 저자도 단점으로 인정한다.
- 점질량·Keplerian IIP 가정에 기반하며 공력/섭동 오차는 후속 phase에서 보정한다고 전제한다. boost-back 고고도 구간에서는 합리적이나, 외란 누적 시나리오에 대한 worst-case 분석은 부재하다.
- baseline이 오프라인 GPOPS 최적해와 자기 선행 IIP 법으로, 외부 경쟁 알고리즘(예: Brendel 2019 PMP, Wei 2019)과의 정량 비교가 없어 상대적 우위 입증이 약하다.

**4. 시뮬/실험.** Falcon 9(CRS-10) 기반 현실 제원(추력 279.6 tonf, Isp 311 s, m0 79.2 t)으로 DRL/RTLS 두 시나리오 (sim). 연소시간 결과: DRL—최적 17.59 s 대비 IIP-FPA 17.70(0.6%), MIG1 17.76(0.9%), MIG2 17.62(0.2%); RTLS—최적 43.29 s 대비 기존IIP 54.39(25.6%)→IIP-FPA 44.96(3.9%), MIG1 43.75(1.1%), MIG2 43.35(0.1%). RTLS 500-run Monte Carlo(추력/Isp ±5%, 위치 ±5 km, 속도 ±50 m/s, 3σ): miss 평균 4.3~4.7 m, 표준편차 2.5~2.9 m. 계산비용(16-core 3.4 GHz): IIP-FPA 평균 1.7 μs/max 32.6 μs 기준, MIG1 1.73/2.04배, MIG2 3.29/3.10배. 오프라인 최적화 대비 수 초→마이크로초로 대폭 단축.

**5. Contribution(delta).** 선행 IIP 유도(Jo & Ahn 2018)의 RTLS 성능 저하(25.6% 갭)를 FPA 변화율 제어 추가로 3.9%까지 줄인 점, 그리고 hit equation/Lambert를 boost-back에 적용해 참조값 없이 거의 최적(0.1~1.1% 갭)인 최소충격 유도를 비반복/준반복 해석식으로 제시한 점이 핵심 delta. 출판년(2025) 기준으로 RLV boost-back용 해석적·온보드 지향 유도의 빈 곳을 메우는 실용 기여이나, 새로운 유도 패러다임을 여는 수준은 아니다.

**6. Limitation·Future work.** (저자) IIP-FPA의 RTLS 참조값 의존과 상대적으로 큰 최적성 갭, MIG의 반복성·제약 부과 곤란, 온보드 구현·worst-case 검증 미흡, 후속 phase 유리한 초기조건 설계와 공력 고려 virtual targeting 필요. (추가) 외부 경쟁기법과의 정량 비교 부재, 제약(추력/각속도 한계, FPA 제약) 명시적 처리 미흡, 코드 미공개로 재현성 제한, γ̇t의 실시간 최적화 부재.

**평가:** 75/100 (strong) · 한줄평: RLV boost-back 유도의 실용적·해석적 기여로 검증이 견실한 core-G&C 논문이나, 신규성은 자기 선행연구 라인의 점진 확장에 가깝다.

---

## (2025, JGCD, Autotuned Primal–Dual Successive Convexification for Reentry Guidance: Mceowen et al.)

**1. 핵심 아이디어.** 비볼록 극초음속 재진입 최적제어 문제를 순차 볼록화(SCvx/SCP)로 풀 때, 제약 위반을 벌하는 페널티 가중치(virtual buffer penalty weight)를 사용자가 손으로 튜닝해야 하는 점이 실용적 약점이다. 본 논문은 듀얼 변수 정보를 이용해 각 제약·각 시간노드의 2차 페널티 가중치와 선형 페널티 가중치를 닫힌형(closed-form)으로 자동 갱신하는 Auto-SCvx를 제안한다. 그 결과 페널티 가중치에 대한 hand tuning(초기값 추정조차)이 불필요해지고, 넓은 미션 파라미터 공간에서 신뢰성 있게 동적 실현가능 궤적을 얻는다.

**2. 알고리즘.** 자유 종단시간 비볼록 OCP를 다중슈팅+FOH+시간구간 dilation+inverse-free exact discretization으로 유한차원 비볼록 문제(Problem 2/3)로 변환. 일반 템플릿에서 제약 위반량을 듀얼변수(λ,μ)와 virtual buffer(p,q)로 정량화하고, 부분 라그랑지안 L2의 정상성 조건으로부터 W_h p* = (λ̂−λ̄), W_g q* = (μ̂−μ̄) 관계를 도출. 이로부터 2차 가중치 대각원소 갱신 ω_{h_i} ← ω_{h_i} p_i* / (sgn(p_i*) ε_{h_i}) (식 30), 선형계수는 augmented Lagrangian식 Δλ*=s_λ p*, Δμ*=max(−μ̄, s_μ q*)로 갱신. primal 업데이트는 deviation 변수 기반 LTV 볼록 QP(식 32, OSQP)로 푼다. 비볼록 동역학은 buffering하지 않고 집합 Z에 직접 포함, 종단·경로 제약만 buffering.

**3. 비판적 검토.**
- 저자 스스로 "Auto-SCvx에 대한 형식적 수렴 분석은 수행되지 않았다"고 명시한다. 수렴 보장은 CT-SCvx(Ref. 63)의 결과에 의존하며, Auto-SCvx는 그 refined initial guess 역할로 정당화될 뿐 자체 수렴증명이 없다. 페널티 자동갱신이 SCP 반복의 수렴/안정성에 미치는 영향에 대한 이론적 보증이 비어 있다.
- baseline이 사실상 동일 연구그룹의 PTR이다. 외부 솔버/타 그룹 SCP 구현과의 비교가 없어, 우수성 주장이 in-house 비교에 한정된다. PTR의 hand-tuned 사례가 진정한 best-effort였는지 검증하기 어렵다.
- intersample(노드 간) 제약 위반이 여전히 존재(저자도 인정). 직접법 공통의 약점이지만 NFZ·동압 제약에서 안전성 측면 함의가 있다.
- 자동튜닝이 step size s_z,s_λ,s_μ, 신뢰영역, ε_{w,min} 등 또 다른 하이퍼파라미터로 부담을 이동시킨다. "tuning-free"는 페널티 가중치에 한정된 주장임.
- HIL/실시간 임베디드 실증은 없음. solve time은 데스크톱 추정치.

**4. 시뮬/실험.** 3-DOF 회전구체 지구 재진입, N=40 노드. 예제 A(bank만 제어), 예제 B(bank+받음각). PTR(w=0.1/10/1000/hand-tuned) 대비 비교 (sim). 예제 A: Auto-SCvx 10.9회 수렴, 종단속도 449.32 m/s, 수렴률 100%; PTR w=0.1/10은 수렴 0%, w=1000은 90%(461 m/s), hand-tuned 100%(458 m/s). 예제 B(분산 10케이스): Auto-SCvx 수렴 100%·362.8 m/s vs PTR hand-tuned 80%·549 m/s, solve time 153 vs 420 ms/iter. 몬테카를로: 거친 격자 216케이스 93.5% 수렴, 조밀 1230케이스 92.9% 수렴(초기 고도/속도/FPA/질량 균일분산). 경로제약(heat rate 33.3 kW/m^2, 동압 18 kPa, normal load 2.5g) 및 NFZ 2개 적용.

**5. Contribution(delta).** 선행 SCP/PTR은 large positive 상수 페널티를 손으로 튜닝(또는 단일 스칼라 augmented Lagrangian)했다. 본 논문의 delta는 (1) 듀얼변수로부터 제약별·노드별 2차 페널티 가중치를 닫힌형으로 자동 갱신하는 새 규칙(식 30, 핵심 기여), (2) 선형 가중치는 augmented Lagrangian식 갱신과 결합, (3) inverse-free exact discretization + 가변 시간격자 + deviation-변수 virtual buffer를 재진입에 통합. 2025년 SOTA 기준, SCP 페널티 자동튜닝은 closest prior art(hand-tuned PTR, exact-penalty SCvx) 대비 명확한 실용적 진전이나, 새 방향을 여는 수준은 아님.

**6. Limitation·Future work.** 저자: 형식적 수렴분석 부재(향후 CT-SCvx 보증 활용), 비볼록 문제의 실현가능성 사전판정 불가(수렴 실패가 조기종료인지 본질적 infeasibility인지 구별 못함), 최대반복 20회 제한이 일부 실패 원인일 수 있음. 추가(내가 보는): 외부 baseline 부재, intersample 위반, 임베디드/HIL 실증 부재, step-size류 잔여 하이퍼파라미터, 모델링 오차·외란 하의 강건성(stochastic) 미평가.

**평가:** 79/100 (strong) · 한줄평: 재진입 G&C의 SCP 실용성 병목(페널티 hand-tuning)을 듀얼 기반 닫힌형 자동튜닝으로 해소한 견고한 in-domain 기여이나, 자체 수렴증명과 외부 baseline이 빠져 flagship에는 못 미친다.

---

## (2026, AST, Online reentry trajectory planning for reusable rockets based on intelligent phase transition: Zhou et al.)

**1. 핵심 아이디어.** 기존 재사용 로켓 연구가 종말 수직착륙(고도 0~4 km)에 집중한 것과 달리, 본 논문은 재진입 궤적을 4~80 km 범위로 확장하여 3차원 온라인 궤적계획 문제로 다룬다. 재진입을 (1) 추력 동력강하, (2) 저동압 자유낙하 근사, (3) 공력강하의 3개 서브페이즈로 분해하는 하이브리드 최적화 프레임워크를 제안한다. 핵심은 종말제약을 파라미터화에 직접 내장한 신규 재진입 프로파일(해석해)과, 도달가능영역(reachable area) 기반 안전제약 하에서 페이즈 전환 시점을 결정하는 최대엔트로피 강화학습(ANME-RL)이다.

**2. 알고리즘.** 페이즈1: 최소추진제 목적의 볼록최적화로 기준궤적 생성 후 MPC 추종(J_MPC = ∫[(r-r_ref)'Q(r-r_ref)+(u-u_ref)'R(u-u_ref)]dt, 추력 0.7P_max≤‖u‖≤P_max). 페이즈3: 재진입 프로파일 h(R_togo) = -β⁻¹ln F(R_togo), F는 M차 다항식; γ(R_togo)=arctan(F'cos(ψ_T-ψ)/(βF))을 해석적으로 산출하고, 종말제약(h_T, γ_T, R_togo,T=0)을 계수방정식(15)에 삽입해 비선형계획에서 제거. 속도는 공력모델(c_D 곡선적합)을 적분해 폐형식 V(R_togo)(식 30~31)으로 도출, N_norm은 h의 2차 도함수로 연속성 보장. 페이즈 전환: ANME-RL이 r_shut(t), α_max, K_gA, h₁,h₂,h₃를 결정; 진화전략(ERL)+최대엔트로피항 k_H·H(π)+KL발산으로 제한된 적응노이즈 ε_σ를 결합(식 37~40), 보상은 1/J_MOP(도달영역 면적) 기반.

**3. 비판적 검토.**
- 해석해가 다수의 공학적 근사(ψ_T-ψ≈0, 사이클당 γ 상수, g_r·A_V 무시, RL이 튜닝하는 보정상수 K_gA)에 의존한다. 결과는 매끄럽지만 근사오차를 K_gA로 흡수하는 구조라 이론적 보장이 약하다.
- ANME-RL은 진화전략+최대엔트로피+적응노이즈의 조합으로, 각 구성요소는 기존(OpenAI ES, max-entropy RL, KL제한)에서 차용된 것이며 수렴/최적성에 대한 형식적 증명은 없다.
- 도달가능영역을 직사각형으로 근사(식 34)하는데, 저자도 보수적임을 인정한다. 더 정확한 convex hull은 계산비용 증가로 회피했다고만 언급.
- 외부(타 논문/실비행) baseline이 없다. 비교는 자체구현 pseudospectral(SNOPT)과 표준 RL(PPO/DPG/ERL)에 한정된다.
- 페이즈2(저동압)는 "자유낙하 근사+선형 α, σ=0"으로 사실상 단순화되어, 3-페이즈 분해의 실질적 기여 중 하나가 다소 얇다.

**4. 시뮬/실험.** Falcon 9 v1.2 Block 5(P_max=2235 kN, Isp=285.5 s, 총질량 81,900 kg) 기반. 경계: h₀=75 km, V₀=1400 m/s, γ₀=-30°→h_T=3 km, V_T=300 m/s, γ_T=-70°. 5회 독립 RL 학습(수렴 일관성) + 300회 MC(초기상태·추력·항력·밀도 균일분포 외란). 정량결과: 종말속도오차 평균 1.69 m/s, σ=1.1 m/s, 99.7%가 <3.5 m/s; 소비추진제 29,034 kg(평균); 경로제약(N_norm<4.49, q<57 kPa, α<55°, α_d=10°/s) 전부 충족 (sim). CPU 런타임 평균 0.28 s, 최대 1.59 s, 98%가 <0.8 s — pseudospectral 9.5 s 대비 큰 효율 개선. RL 비교(Table II): ANME-RL 최종보상 0.941 vs PPO 0.931/ERL 0.893/DPG 0.859. 추력 30% 저하 극한 시나리오에서도 안전궤적 생성.

**5. Contribution(delta).** 출판년(2025/2026) SOTA 대비 delta: (i) 종말착륙(0~4 km) 또는 활공체 방법(20~60 km)에 머물던 재사용 로켓 재진입을 4~80 km 광역 온라인 다페이즈 문제로 정식화하고, 로켓 동역학(짧은 비행시간·큰 강하율, quasi-equilibrium glide 가정 무효)을 활공체와 명확히 구분; (ii) 종말제약을 프로파일 파라미터화에 내장해 비선형계획의 결합을 분리하고 폐형식 해석해(h, γ, V)를 제공해 온보드 효율 확보; (iii) 도달가능영역 안전제약 하 RL로 페이즈 전환을 결정. novelty는 신규 방법의 발명이라기보다 도메인 적합 재정식화+기존 RL 구성요소의 효과적 결합에 가깝다.

**6. Limitation·Future work.** 저자: 도달영역의 직사각형 근사는 보수적이며 정밀 convex hull은 계산부담; 페이즈2 단순화. 내가 보는 한계: (a) K_gA로 모델오차를 흡수하는 구조라 해석해의 독립적 정확성이 불분명; (b) RL 수렴·안전성에 대한 형식 보장 부재로 학습범위 밖 외란에서의 보장이 경험적(MC)에 그침; (c) 외부/실비행 baseline 및 6-DoF·자세동역학 미고려; (d) HIL/임베디드 실증 없이 노트북 CPU 타이밍으로 온보드 가능성 주장.

**평가:** 76/100 (strong) · 한줄평: Falcon 9 현실 파라미터·해석해·명시적 온보드 타이밍·300회 MC를 갖춘 견실한 core-G&C 재진입 계획 논문이나, 다수 근사와 조합형 RL에 의존해 방법론적 신규성은 분야 판도를 바꾸는 수준은 아니다.

---

## (2026, JGCD, Explicit Trajectory Dispersion Control for Precision Landing Guidance of Reusable Rockets: Chen et al.)

**1. 핵심 아이디어.** 대기권 내 재사용 로켓의 정밀착륙에서, 외란(추력편차·자세추종오차·공력계수·대기밀도·바람)이 만드는 궤적 분산(상태와 지령 모두의 평균·분산)을 실시간으로 직접 성형(shaping)하는 온라인 분산 제어 기법을 제안한다. 핵심은 시변 2차 가중치로 매개화된 최적 피드백 유도법칙(POFGL)이며, 이를 기반으로 (i) 폐루프 확률 동역학을 gPC 전개와 의사스펙트럴 collocation으로 고차원 결정론적 선형계로 근사해 분산을 온라인 예측하고, (ii) 경사하강 기반 실시간 파라미터 튜닝으로 3σ 확률제약(착륙오차·지령진폭)을 만족시키며 성능지표를 최적화한다.

**2. 알고리즘.** POFGL: u(τ)=u_r(τ,x0)+K_u(τ,x0,θ)[x(τ)-x_r], a(τ)=a_r(x0)+K_a(τ,x0,θ)[x(τ)-x_r]. 공칭궤적(Problem 2)과 피드백행렬(Problem 3)은 시변 가중치 R_x,R_u,R_a,R_f를 라그랑주 보간으로 매개화(식 28)하고 PDDP(의사스펙트럴 DDP)로 동시 산출. 분산예측: 폐루프계 x'=F[τ,x,ξ_w,θ]를 ξ_w 기준 gPC 전개(식 31) → 공칭 주위 1차 선형화(식 33) → Galerkin 투영으로 A_G=Σ Φ_i⊗A_i^c 형태의 고차원 선형계(식 38) → LGR collocation으로 해석해 → 가우스적분으로 평균(식 45)·분산(식 46) 산출, 정규근사+3σ로 99.74% 분산 표현. 튜닝: θ(τ_{c+1})=Π_Θ[θ(τ_c)-γ(∂J_t/∂θ)^T], 3σ 섀도우 제약을 smoothed Hinge 벌점으로 무제약화(식 56-58). 초기값 θ_init는 LHS 무작위탐색으로 오프라인 산출.

**3. 비판적 검토.**
- 정규분포 근사에 기반한 3σ 칸스 처리: 비선형 동역학에 대해 분산이 정규를 따른다는 가정과 1차 선형화(식 33)에 의존하는데, 근사오차에 대한 상한이나 수렴 보장이 없어 확률제약 만족이 형식적 보장이 아니라 경험적이다.
- 자기참조적 검증: 차량 파라미터·POFGL·PDDP 모두 저자 자신의 Ref [22](2024)에서 가져왔고, 외부 baseline(공분산 제어, 강건궤적최적화)과의 정량적 head-to-head 비교가 없이 서술적 비교에 그친다.
- 단일 시나리오: 하나의 6-DoF 벤치마크, 하나의 외란 설정에 대한 검증으로 일반화 근거가 제한적이다(다양한 초기조건·고장모드 미포함).
- 회전동역학 무시·트림 추력각 가정은 모델 단순화로 합리적이나, 실제 자세루프와의 상호작용이 분산예측 정확도에 미치는 영향은 다루지 않는다.

**4. 시뮬/실험.** 6-DoF 대기권 내 동력강하 1개 벤치마크(Ref [22] 파라미터). 외란: 추력 3%, 자세 0.5°, 공력계수 50%, 밀도 30%, 바람 30/60/40 m/s(고도 3점 다항피팅), 초기상태 위치 300m·속도 15m/s·자세 5° (모두 3σ). Case 0(오프라인)/1(예측)/2(온라인 100회 튜닝) 비교 + 착륙정확도 분석. 정량: 온라인 튜닝으로 종단오차 평균·표준편차를 5m/2(m/s)/2° 요구에서 1m/0.5(m/s)/1° 요구까지 충족(sim); J_t는 약 40회에서 수렴. MC 1000회 대비 3σ 예측 포락이 거의 전 궤적 포괄(sim) — 예측이 MC와 동등 정확. 실시간성: 분산예측 평균 10.06ms(최대 12.32, 최소 9.59ms) < 튜닝주기 100ms (AMD Ryzen 7 6800H, 3.2GHz). 외부 baseline 없음, HIL/실비행 없음.

**5. Contribution(delta).** 선행 공분산제어 유도법(가우스 백색잡음·공력 무시 환경 전제)과 오프라인 강건궤적최적화(PCE+GA, 보수적·오프라인)의 한계를 넘어, 출판년(2025 arXiv/2026) SOTA 기준으로 (a) 임의 확률분포 외란을 ξ로 매개화한 폐루프 분산을 gPC+collocation으로 온라인 예측(공분산제어보다 정확하다고 주장), (b) 상태뿐 아니라 지령 분산까지 실시간 직접 성형, (c) 경사하강 해석적 기울기로 온라인 튜닝하는 일반 프레임워크(POFGL 교체 가능)를 제시한 점이 delta. 다만 저자 자신의 POFGL/PDDP(Ref [22]) 위의 확장이라는 점에서 증분적 성격이 있다.

**6. Limitation·Future work.** 저자: POFGL은 다른 매개화 유도법으로 대체 가능, 튜닝은 베이지안 최적화·강화학습으로도 가능하다고 제안. 내가 보는 한계: 정규근사·1차 선형화의 오차 상한/수렴 보장 부재로 확률제약이 형식적 보장이 아님; 외부 baseline 부재·단일 벤치마크로 우월성 입증이 약함; 회전동역학 무시의 영향 미평가; 실비행/HIL 부재(데스크톱 타이밍만 제시). 추가로 다중 시나리오·고장모드·항법오차 결합 하의 강건성 검증이 필요하다.

**평가:** 75/100 (strong) · 한줄평: 재사용 로켓 정밀착륙의 온라인 궤적분산 직접성형이라는 핵심 G&C 문제를 gPC 예측+실시간 튜닝으로 정교하게 푼 방법론적으로 탄탄한 note이나, 자기참조적 단일 벤치마크·외부 baseline 부재·근사 기반(비형식적) 보장이 상한을 누른다.

---

## (2025, AST, Reentry trajectory optimization for reusable launch vehicle using marine predator whale optimizer and smoothing technique: Su·Liu)

**1. 핵심 아이디어.** RLV 재진입 궤적 최적화를 자연영감 메타휴리스틱(MPWOA, marine predator whale optimization algorithm)으로 풀되, 다중사격(multiple shooting) 이산화에서 급격한 고도·속도 변화 시 발생하는 제어 프로파일 진동을 줄이기 위해 진화 과정 안에 5종의 스무딩 기법(Average/Median/Gaussian-based, Uniform/Nonuniform B-spline)을 삽입한다. 타원체·자전 지구(J2~J4 중력, geodetic/geocentric 위도 구분) 위 3D 무동력 활공 동역학을 사용하고, ε-constrained 제약처리와 4종 모집단 초기화(UI/OI/LI/CI) 및 모집단 크기 민감도 분석을 함께 제시한다. 최소비행시간(MFT)·최대종단속도(MTV) 두 시나리오를 대상으로 한다.

**2. 알고리즘.** 목적함수는 min J=t_f (MFT), min J=-V_k(t_f) (MTV). 동역학은 타원체·자전 지구상 3DOF EoM(식(1), 부록 A에서 7개 좌표프레임 변환으로 유도), 중력은 타원체 포텐셜의 구배 g=-∇U(식(4)-(8), J2=0.00108263 등). 경로제약: 가열률 Q̇=K_Q ρ^0.5 (u√(g0R0))^3.15 ≤ Q̇max, 동압 q, 과부하 n ≤ n_max(식(9)); 제어제약 |α|≤α_max, |σ|≤σ_max. 다중사격 이산화는 Gauss-Legendre 노드 + 4차 RK(식(28)). MPWOA는 비선형 전이파라미터 A=3(1-t/T)sin(2πr)(식(17))로 exploration/transition/exploitation 단계 전환(식(18)-(20)), FADs 효과(식(21)), DE 변이 기반 ranking 갱신(식(22)). 스무딩은 zigzag 검출 시(sign(u_k-u_{k-1})·sign(u_{k+1}-u_k)<0) 3점 평균/중앙값/가우시안 가중(ω≈0.274,0.452,0.274) 또는 전구간 B-spline u(t)=Σ N_{i,p}(t)P_i (식(23)-(27)). 제약처리는 동적 ε(t)=ε(0)(1-t/T_c)^cp(식(29)).

**3. 비판적 검토.**
- 핵심 탐색엔진 MPWOA는 저자 선행/타 연구 [38]에서 그대로 가져온 기성 알고리즘이며, 본 논문의 알고리즘적 신규성은 "RLV 재진입에 적용 + 스무딩 삽입 + 초기화/모집단 민감도"의 조합에 머문다. 메타휴리스틱 자체에 대한 수렴성·최적성 보장은 없고, 최적성은 GPM Hamiltonian(식(35)-(36)) 형상 비교라는 경험적 검증에 의존한다.
- 스무딩 5종 비교에서 GB(Gaussian-based)를 "두 시나리오 모두 균형적으로 우수"하다고 선택하지만, 실제 ARV 순위(Table 5)에서 GB는 어느 시나리오에서도 1위가 아니며(MFT 2위, MTV 2위) AB/NBS가 각각 1위다. 선택 근거가 "feasibility 100%+균형"으로 사후적이고, 통계적 우열 폭이 좁다.
- 보고된 CPU 시간이 데스크톱 i9에서 ~324-380s 수준으로, 온보드/실시간과는 거리가 멀다. 저자 스스로 gradient 기법의 초기추정치 용도로 위치시키므로 실용 배치 측면 기여는 제한적이다.
- "robustness"라고 부르는 것은 알고리즘 확률성에 대한 20회 MC 분산이며, 모델 파라미터·대기·외란 등 물리적 불확실성에 대한 강건성 검증은 없다(저자도 future work로 인정).
- α를 속도 스케줄로 고정(식(34))하여 사실상 단일 제어변수(σ)만 최적화하므로 제어 자유도가 축소되어 있고, 이것이 진동/스무딩 논의의 일반성을 약화시킨다.

**4. 시뮬/실험.** 두 시나리오(MFT, MTV), 초기 h=100km/V=7450m/s, 종단 h=25km/φ_c=73.5°/λ=11°/γ=-10°/χ=90°; Q̇max=1500kW/m², q_max=18kPa, n_max=2.5g0(Table 3-4). N_p=100, Gauss-Legendre N=32, T=300 iter, 20회 독립 MC. baseline: GPM(GPOPS+SNOPT)을 최적성 기준으로, 그리고 외부 NIA 5종(PSO/WOA/HHO/MPA/NMPA)과 비교(Table 7-8). 통계검정: Wilcoxon signed-rank(5%) + Friedman ARV. 결과: MPWOA가 MFT overall rank 1(ARV 2.3542, feasibility 100%), MTV rank 2(NMPA에 1위 양보); PSO/WOA/HHO는 MFT feasibility 10%/0%/0%로 사실상 실패. 스무딩 GB feasibility 100%·CPU 326-380s. 모두 (sim), HIL/실비행 없음.

**5. Contribution(delta).** 선행 대비 delta: (i) 기성 MPWOA를 타원체·자전 지구 고충실도 3D 재진입 OCP에 적용, (ii) 진화 루프 내 5종 스무딩 기법을 통합해 제어 프로파일 진동을 완화하고 GPM 대비 정합 향상, (iii) 4종 초기화 및 모집단 크기(100-150 권장)의 정량 민감도 분석. 2024-2025 출판 시점 기준으로 메타휴리스틱+재진입 궤적은 이미 포화된 분야(PSO·sparrow·pigeon·sine-cosine 등 다수 선행)여서, novelty는 새 방법론 개창보다 "기존 구성요소의 조합·튜닝 + 진동완화 처리"라는 점진적 개선에 해당한다.

**6. Limitation·Future work.** 저자: gradient 기법과의 하이브리드, 실제 비행데이터 기반 ML 통합, 환경 불확실성·fault-tolerance·다목적 최적화. 추가로 내가 보는 한계: 메타휴리스틱의 최적성/수렴 이론 부재, GB 선택의 사후적 근거와 좁은 통계적 마진, 단일 제어변수(σ)로의 사실상 축소, 물리적 불확실성 강건성 미검증, 오프라인 계산비용(수백 초)으로 인한 온보드 부적합, 코드 비공개(데이터 on request).

**평가:** 60/100 (fair) · 한줄평: 고충실도 타원체·자전 지구 재진입 모델과 견실한 NIA 벤치마크/통계검정을 갖춘 in-domain G&C 논문이나, 핵심 최적화기는 기성 메타휴리스틱이고 기여가 조합·진동완화 수준의 점진적 개선에 머문다.

---

## (2025, IJCAS, Optimal Landing Guidance of Reusable Launch Vehicles With Dual-convex Optimization: Jung et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV) 착륙연소의 유도를 두 개의 독립된 최적제어문제(OCP)로 분리해 순차볼록계획법(SCP)으로 푸는 모델예측 유도 기법이다. (i) 3-DOF 점질량 모델로 점화시각·비행시간까지 최적화하는 연료최적 계획(Problem P), (ii) 자세·스로틀 천이동역학을 포함한 5-DOF 모델로 가변 예측구간을 갖는 추종(Problem T)으로 구성된다. 계획의 모델충실도 부족을 추종단계와 TVC 편향을 고려한 양력보상으로 메워 closed-loop에 가까운 성능을 노린다.

**2. 알고리즘.** 계획: minimize J = -m(t_f), 동역학 r'=v, v'=(T+F_aero)/m+g, 시간확장 변수 η와 정규시간 τ=t/η로 free-final-time을 변수화. 추력하한 비볼록성은 보조변수 Γ=||T|| 도입 후 ||T||≤Γ로 볼록완화. 추종: 상태에 θ,ψ,Γ 증강, θ'=(θ_c-θ)/τ_θ 등 1차 천이동역학으로 control-affine화, LQ 추종비용 J=X_e^T W_F X_e + ∫(X_e^T W_Q X_e + Δ^T W_R Δ)dt를 QP로 풂. 양력보상: TVC 편향 δ_T,A로 인한 양력계수를 C_L' = C_L - (l_cp/l_c)C_z 로 수동 수정(추가 연산 없음). 사다리꼴 적분 이산화 + soft trust-region.

**3. 비판적 검토.**
- 핵심 볼록완화(||T||≤Γ)의 정확완화(exact relaxation)는 비선형 동역학+다중 비선형 경로제약 하에서 증명되지 않고 "광범위한 수치실험으로 경험적 확인"에 그친다. 저자도 명시하나, TAES/TASE 기준에서는 이론적 공백.
- Problem T의 feasibility 논의(Sec IV-D)가 형식적 서술 수준이며 정리·증명이 아니다. 무한대 외란 시 자세제약 위반 가능성을 "입력단 명령제한"으로 회피한다는 실무적 봉합.
- 양력보상이 영각 α_T가 작다는 가정(1차 Taylor)에 의존하나, Fig.8에서 α_T가 13deg까지 도달하며 가정의 경계에 근접. 큰 α_T 영역의 오차 정량 분석이 약하다.
- 비교 baseline(Case B/C/D)이 대체로 자체 ablation 또는 단일 해석적 법칙[39]이며, 최신 외부 6-DOF SCP 유도(Szmuk[22], Sagliano[26])와의 직접적 정량 비교는 없다.

**4. 시뮬/실험.** Falcon-9 1단급 모델, MATLAB 6-DOF 시뮬(i7 데스크톱), TVC 2차·스로틀 1차 액추에이터. 계획 SCP 5회 반복 0.215s 수렴, 추종 평균 12ms·최대 34ms(4Hz 갱신, 약 20배 여유). Case A1: 착륙 위치오차 0.320m, 속도 0.115m/s, 계획대비 연료 21.5kg 절감. GPOPS-II로 계획 정확도 검증(t_c=3.88s 일치). 500-sample MC(초기조건·풍·C_L/C_D ±15% 분산): 495 중 476 성공 = 96%+, 평균 ||r_f||=1.469m, ||v_f||=0.693m/s. Case C(양력 미보상)가 최악, 제안 양력보상의 효과 입증. (all sim)

**5. Contribution(delta).** 출판년(2024) SOTA 대비: (a) 계획·추종을 모두 궤적최적화로 동시 사용하는 하이브리드 구조, (b) TVC 제어루프 특성을 양력계수 수동수정으로 반영해 연산부담 없이 충실도 격차 축소(신규), (c) 공력하중한계를 의사결정 변수/제약으로 포함한 새 착륙유도 정식화. Szmuk[22]의 6-DOF SCP나 Wang[19]의 MPC 대비 5-DOF 단순화로 onboard 실시간성을 확보한 점이 차별점.

**6. Limitation·Future work.** 저자: 실제 항전탑재 시 연산시간 수배 증가 가능, 점화 후 재계획 실패 사례 존재(추종으로 보완). 추가로 내가 보는 한계: 정확완화·feasibility의 이론적 증명 부재, 외부 6-DOF baseline 부재, 단일 차량/HIL·실비행 검증 없음(데스크톱 sim only), roll 채널·다엔진은 가정으로 배제.

**평가:** 76/100 (strong) · 한줄평: RLV 착륙유도의 실시간 dual-SCP 정식화로 양력보상·실시간성이 돋보이는 견실한 core-G&C 기여이나, 핵심 완화의 이론증명과 외부 SOTA 비교가 빠진 점이 flagship과의 거리.

---

## (2025, JGCD, Biased-Proportional-Navigation-Based Predictor–Corrector Landing Guidance for Reusable Launch Vehicle: Yang)

**1. 핵심 아이디어.** 공력(양력·항력·측력)이 무시 불가한 대기권 내 재사용 발사체 1단 연착륙 유도 문제를 다룬다. 핵심은 편향 비례항법(BPN)을 사용해 받음각 α와 옆미끄럼각 β를 상태의 함수로 변환(속도 방향이 착륙장에서 수직이 되도록)함으로써 3개 제어변수(추력 크기, α, β)를 추력 크기 단일 변수로 축소하는 것이다. 이어 연료최적 점화시점과 추력 프로파일을 이론적으로 규명하여 동력하강 단계를 단일 파라미터(점화시점/추력크기) 근찾기 문제로 환원, 임베디드 실시간 구현이 가능한 저연산 예측-수정(predictor–corrector) 유도 알고리즘을 제안한다.

**2. 알고리즘.** LOS 좌표계에서 횡·종 운동을 분리해 BPN 유도식 ω_yL=4λ̇_e cosλ_a − 2(ϱ̇/ϱ)(λ_e−γ_f), ω_zL=4λ̇_a − 2(ϱ̇/ϱ)(λ_a−ψ_f) (식 27)을 유도하고, 여기서 원하는 γ̇_d, ψ̇_d를 추출해 식 (1)의 동역학과 등치시켜 α̂=−(γ̇_d mV cosψ + mg cosγ)/(T−C_L^α q̄S_ref), β̂ 유사식 (36)으로 받음각·옆미끄럼각 명령을 얻는다. 동력하강 단계는 간소화 최적제어문제 (38)에서 Pontryagin 최대원리로 bang-bang 구조(Lemma 1)와 coast–burn 구조(Proposition 1: T=0 [t0,t_ig], T=T_max [t_ig,t_f])를 증명하고, 단일 미지수 τ(phase1=t_ig, phase2=T_c)에 대해 동역학 적분으로 f(τ)=e_2^T(x0+∫F dt)=0의 근을 secant법으로 찾는다. 외란 보상은 종방향 가속도 오차에 대한 PID ΔT=K_P e_a+K_I∫e_a+K_D ė_a로 추력 명령을 보정한다.

**3. 비판적 검토.**
- **α̂ 명령식 (36)의 분모 T−C_L^α q̄S_ref가 점화 시점마다 반드시 0을 통과한다.** 저자 본인이 명시한다(p.1598): "엔진 off이므로 T−C_L^α q̄S_ref<0. 점화 후 추력이 충분히 크므로 >0. **discontinuity가 존재한다.** 제약 (7)이 없으면 α̂(t_ig⁻)와 α̂(t_ig⁺)의 부호가 반대가 된다." 처방은 각속도 제한 (7)에 의한 포화뿐이다 — 특이점을 해소한 것이 아니라 덮은 것이고, 점화 근방에서 BPN 법칙이 실현되지 않는 구간의 지속시간과 그 연료·종말정확도 대가에 대한 분석이 전혀 없다. MC가 C_L^α를 3σ 20%로 흔드는데, 이 분모의 영점 위치가 바로 그 파라미터에 걸려 있다.
- 특이점 임시처방이 셋이다: (i) 식 (28)의 ε — ϱ→0에서 식 (27) 발산 회피용으로 z에 더하는 임의의 작은 양수(조준점을 옮기는 것인데 종말정확도 영향 분석 없음), (ii) 예측 적분 중 y≤0이면 α=β=0 강제, (iii) 위의 각속도 포화. 셋 다 정당화·오차분석이 없다.
- 이론적 최적성(Prop.1)은 간소화 문제 (38)에서만 성립한다. (38)은 1D 수직강하(ẏ=−V)·k_D와 g를 상수로 고정·양력/측력 없음·지구자전 없음·(p−p_e)A_e 없음·추력제약을 [0,T_max]로 완화한 모델이고, 저자도 "분석 전용이며 풀 의도가 아니다"라고 못박는다(p.1596). 구현은 전 3D 동역학 (1)에서 돈다. 초록의 "theoretical analysis establishes fuel-optimal ignition time"은 claimed-vs-implemented 갭이다. 다만 Assumption 2를 검증 가능한 조건(Prop.2, Fig.4: ξ>1.3)으로 환원한 것은 성실한 작업.
- 횡·종 분리 + 속도 일정 가정으로 BPN 운동학을 유도하고, 식 (25) λ_ee≈λ_e와 식 (35) 소각근사·지구자전항 무시를 α̂/β̂ 역산에 쓰면서 정작 전파는 그 항들을 유지한다. 근사 오차 경계 없음. secant 수렴 보장도 없음.
- **BPN+로켓착륙은 이미 [24](Xiong et al., Acta Astronautica 2022)가 했다** — 저자 본인이 서론에서 "[24]가 BPN으로 비선형 착륙 동역학을 분리·간소화하여 SCO 수렴을 개선했다"고 인용한다. PID 종방향 가속도 보상은 저자 자신의 [28], 점화시점 온라인 결정은 [21,41,42], 1-파라미터 예측-수정은 [32–39]. 구성요소가 전부 차용이고 delta는 조합 자체로 한정된다(그 조합은 유효하다 — §5).
- 추력 가정에 대한 오해 주의: 원문은 "not gimbaled **independently**"(p.1593)로, 짐벌 부재가 아니라 짐벌을 유도단 독립 제어입력으로 두지 않는다는 뜻이다. 식 (1)에서 추력은 T cosα cosβ / T sinα / T cosα sinβ로 3축 분배되어 자세로 조향된다. 실제 RLV 짐벌 가동범위가 수 도 수준(cos5°≈0.996)이라 추력≈동체축 정렬은 정량적으로 타당하고, 콘 내 자유지향을 허용하는 LCvx 계열보다 오히려 더 물리적이다. **공격 지점은 "짐벌을 안 쓴다"가 아니라 "BPN이 뱉는 α·β를 유한 권한·유한 대역폭 짐벌이 따라갈 수 있는지 검증이 없다"이다** — 하필 위의 특이점을 지나는 그 순간에.

**4. 시뮬/실험.** 4개 미션(종·횡 divert 상이, x0=−1.9~−3.9 km, y0=11 km, V0=500 m/s, γ0=−65°) 명목 시뮬레이션. m0=26000 kg, v_ex=2700 m/s, T_max=400 kN, T_min=160 kN, 마진 δ=60 kN, n̄q_max=2.3e5 deg·Pa, η_max=15°, V_f=1 m/s 등 파라미터 제시(단 PID 게인 K_P/K_I/K_D 미공개, 공력계수는 Fig.7 그림뿐). GPOPS-II 연료최적해 대비 6.1~7.5% 초과(2910.9 vs 2707.8 kg 등) → 근사 최적 달성, 수직착륙에 유리한 작은 α·β 생성. secant법 4·4·4·3회 반복으로 0.31~0.40 ms(i5-13600KF **데스크톱**, C언어, 적분스텝 0.5 s, 초기추정 τ=10 s, **수렴판정 0.1 m**), 유도 1 Hz/제어 100 Hz.
- **baseline 문제:** GPOPS-II는 오프라인 최적제어 solver, 즉 *최적성 기준선*이지 경쟁 유도법이 아니다. 논문의 주장 delta가 "SCO 대비 ms급"인데 SCO 계열과의 head-to-head가 **0건**이다. 특히 최근접 선행 [24](BPN+SCO)가 바로 옆에 있는데 비교하지 않는다.
- **touchdown miss가 물리적 결과가 아니다:** MC는 별도 초기조건(Table 3: x0=−2.6, y0=10 km, V0=450 m/s)에서 3000회. 3σ = C_D·C_L^α 20%, ρ·p 15%, V_w 10 m/s, α_w·β_w 4.5°, **x0/y0/z0 0.7 km**, V0 20 m/s, γ0/ψ0 3°. 결과(Table 6): √(x_f²+z_f²) 평균 **4.70e-5 m**, 최대 **3.27e-4 m**. 초기위치를 700 m 흔들어 착륙오차 0.05 mm — 1500만분의 1 축소다. 이는 강건성 입증이 아니라 **항법오차 0·자세동역학 0·액추에이터 0**의 서명이다. α·β를 매 제어주기(100 Hz) 현재 *참값* 상태로부터 식 (37)로 직접 계산하고(저자 권장, p.1597) 플랜트가 유도의 내부모델과 동일하니, 사실상 완전상태피드백 추격법의 수치 수렴잔차를 보고 있는 것. 실제 F9급 착륙정확도는 미터 오더이며 nav/TVC가 지배한다. 결론부의 "land precisely and safely under various uncertainties"는 모델링된 범위를 넘는 주장.
- 유익한 부산물: Fig.14의 점화시점 상태분산(속도 140~230 m/s, 고도 1500~2300 m)은 점화조건 사전지정이 위험하다는 논문의 *동기*를 실증한다 — 이건 설득력 있다.
- 굽힘모멘트 (5)·TAOA (6)·각속도 (7) 제약은 실제로 모델링됨(구현 현실성 가점). HIL/실HW 없음(루브릭상 감점 아님).

**5. Contribution(delta).** 선행 SCO 계열([16–18,21–23,25,26])은 점화시점을 외부 지정으로 두거나 광범위 선형화로 처리했고, 온라인 결정은 [21,41](SCO)·[42](Lu, PDG)에 국한. 예측-수정 계열([32–39])은 **단일 입력을 요구**한다는 알려진 제약 때문에 3-입력 착륙에 직접 적용 불가였다(저자가 서론에서 명시). 실질 delta는 **BPN의 단일입력 축소가 정확히 그 blocker를 푼다**는 것 — α,β를 상태함수화 → 추력크기 1개만 남음 → coast–burn 구조 증명으로 t_ig가 단일 근찾기 파라미터가 됨 → 4회 반복·0.4 ms. 깔끔하고 실제로 유효한 착상이다.
단, delta의 폭은 좁다: BPN+로켓착륙은 [24](Acta Astro 2022)가 선점, PID 보상은 자기 [28], 온라인 점화는 [21,41,42], 1-파라미터 PC는 [32–39]. 새것은 조합과 그 조합이 푸는 blocker의 식별이며, 저자는 계보에 대해 정직하다.

**6. Limitation·Future work.** 저자 인정: 점화 상태분산(Fig.14) 때문에 사전지정은 임무 실패 가능 → 온보드 결정 필요. 내가 보는 한계: (i) **점화 시 α̂ 분모 영점통과 특이점을 각속도 포화로 덮고 분석 없음** — 저자 스스로 discontinuity 존재를 서술, (ii) 특이점 임시처방 3종(ε, y≤0→α=β=0, 포화) 전부 미정당화, (iii) 이론 최적성이 1D 간소화 문제 (38) 한정이라 구현된 3D 동역학 (1)에 대해 미보장(초록의 claimed-vs-implemented 갭), (iv) **경쟁 유도법 baseline 0건** — 특히 최근접 [24] 미비교, (v) 3-DoF 점질량: 자세동역학·짐벌 권한/대역폭·항법오차 부재 → miss 4.7e-5 m는 추상화 산물, (vi) 검증 포락선 협소(단일 기체, V0=450~500 m/s 한 점 근방), (vii) secant 수렴·폐루프 안정성 형식 보장 없음, (viii) 타이밍이 데스크톱 i5(비행 컴퓨터 아님 — 단 30× 감속 가정해도 ~12 ms로 1 Hz엔 충분).
Future: **α̂ 특이점의 원리적 해소**(포화가 아닌 정식화 수준), [24]/[28] 대비 동일조건 head-to-head, 짐벌 권한·항법오차를 넣은 6-DoF MC(진짜 miss 분포 산출), BPN 분리근사 오차의 정량 경계화.

**평가:** 72/100 (good) · 한줄평: 예측-수정법의 "단일 입력 요구"라는 알려진 blocker를 BPN 입력축소로 푼 착상이 깔끔하고 0.4 ms·연료 6~7% 갭이라는 실측도 견실하나, 구성요소 전부가 차용([24]가 BPN+착륙 선점)이고 최적성 증명이 구현 동역학에 전이되지 않으며 경쟁 유도법 baseline이 0건이고 대표수치 miss 4.7e-5 m가 항법·자세·액추에이터 부재의 산물이라, flagship이 아니라 견실한 good.

> **재교정 2026-07-15 (82→72, v1.3 앵커):** 이전 점수는 원문 미확인 상태의 요약 기반. 원문 정독 결과 (a) p.1598의 점화 특이점 자백, (b) [24] Xiong 2022이 BPN+로켓착륙 선점, (c) Table 6의 miss 4.70e-5 m 평균 / 3.27e-4 m 최대가 확인되어 C1 4.5→3.5, C2 4→3.5, C4 4→3.5, C5 4.5→3.5, C7 4→3.5. C3(4)·C6(4)·C8(3.5) 유지. 이전 리뷰의 "추력 비짐벌" 표현은 원문 "not gimbaled independently"의 오독이라 §3에서 정정.

---

## (2025, JGCD, Fast Desensitized Optimal Control for Rocket-Powered Descent and Landing: Robbiani et al.)

**1. 핵심 아이디어.** Seywald의 Desensitized Optimal Control(DOC) 이론을 로켓 동력하강·착륙 문제에 적용하되, 표준 DOC의 치명적 단점(민감도 행렬 전파로 인한 미분방정식 수 2n_x² 급증)을 해소하는 계산 효율적 "Fast DOC" 정식화를 제안한다. 핵심은 최종 상태에 대한 민감도 중 지배적 성분만 골라 축소차원 벡터 Λ(t)로 전파하는 "dominant sensitivity" 개념, LQR/Riccati 기반 피드백 게인의 궤적 동시 최적화 통합, 그리고 추력 포화를 다루는 "marginal DOC coefficient" ν의 도입이다. 결과적으로 화성 착륙 벤치마크에서 정확도 손실 없이 표준 DOC 대비 CPU 시간을 약 30배 단축한다.

**2. 알고리즘.** 비용함수는 Mayer 항에 민감도 적분항과 피드백 게인 항을 가중 결합: J = φ(x(t_f),t_f) + α(∫Σα_k‖Λ(t)_k‖₂ dt + βΩ_Ω 항). 민감도 행렬 S(t) 동역학 Ṡ=A(t)S, S(t_0)=I 대신, 최종상태 한 성분에 대응하는 행 결합 벡터 Λ(t)만 역방향 전파: Λ̇=-(A_R+ηB_R K)^T Λ, Λ(t_f)=Λ_f(특정 위치에 1을 가진 영벡터). 피드백 게인은 affine 법칙 u(t)=u*(t)+K(t)(x(t)-x*(t)), K=-R^{-1}B_R^T P, P는 Riccati 미분방정식 -Ṗ=PÃ_R+Ã_R^T P-PB̃_R R^{-1}B̃_R^T P+Q로 동시 전파. 추력 포화 처리: η(t)=4(u*-u_min)(u_max-u*)/(u_max-u_min)², 그리고 η̃(t)=νη(t)로 게인 마진(약 1dB→ν=2) 내에서 피드백 복원.

**3. 비판적 검토.**
- 이론적 신규성의 상당 부분은 Seywald(DOC), Makkapati(reduced-order/dominant), 민감도-공분산 연결[50] 등 선행 기계장치의 조합·정제이며, 비선형 안정성에 대한 형식적 보장이 없다. ν 상한 기준(게인 마진)은 선형화 근사에 기반하며 저자 스스로 "비선형 경우 형식적으로 성립하지 않는다"고 명시한다.
- 베이스라인이 모두 내부 재구현(noDOC, Standard/Alternative/Efficient DOC)이라 외부 독립 비교가 없다. CPU 30배 단축은 동일 코드베이스 내 비교이므로 구현 최적화 편향 가능성을 배제하기 어렵다(특히 Alternative DOC가 예상과 달리 Standard보다 느린 점도 저자가 "원인 불명, 추가 조사 필요"로 남김).
- 불확실 파라미터를 추력 단일 파라미터로 한정하고 위치 피드백만 적용(질량은 측정 불가 가정)하는 등 시나리오 의존적 단순화가 많아 일반성 주장이 다소 약하다.
- 25.24s CPU는 표준 DOC 대비 극적 개선이지만 여전히 온보드 실시간과는 거리가 있고, 저자도 convex 기반 최신 기법과의 격차가 남아있음을 인정한다.

**4. 시뮬/실험.** 화성 동력착륙 단일 시나리오군(초기 1900/800/3100 m, 속도 40/20/-50 m/s, n_T=3 엔진, γ=27°). IPOPT NLP + Radau pseudospectral collocation + Adigator AD. 각 1500회 Monte Carlo(초기 위치 100m, 속도 10 m/s 3σ 섭동, 추력 2% 3σ 바이어스+시변 노이즈). 성능 목표: 수평 위치 10m, 수직 위치 1m, 수직 속도 1 m/s(3σ) 이내. α/ν/β 스윕으로 민감도-연료 트레이드오프 정량화. CPU 비교(Table 4, i5-1250P, 15회 평균): Standard DOC 811.44s(49+49 상태), Alternative 997.90s, Efficient DOC 160.31s(49 상태), Fast DOC 25.24s(7 상태) — 표준 대비 약 32배 단축. MC가 모든 케이스에서 폐루프 비선형계 안정성 확인.

**5. Contribution(delta).** 출판년(2025) 기준 SOTA 대비 delta: (i) 표준 DOC의 2n_x² 추가 상태를 dominant sensitivity로 n_Λ·n_x(본 문제 7개)로 축소하여 DOC를 항공우주 궤적최적화에 실용화 — Makkapati류 축소기법보다 더 공격적이고 물리적으로 해석가능한 행 선택. (ii) marginal DOC coefficient ν로 bang-bang max-min-max 포화 구간에서도 피드백 권한을 게인 마진 내에서 복원하는 새 메커니즘. (iii) 민감도-공분산 관계[50]에 기반한 a priori 성능지수로 값비싼 사후 MC 전 설계 가이드 제공. 표준 DOC와 동등 정확도를 ~30배 빠르게 달성.

**6. Limitation·Future work.** 저자: 튜닝이 case-dependent(다른 초기조건/강한 비선형성에 대한 일반화 미검증), 추력 민감도 정식화의 정확한 최종 표준편차 예측 불가, convex 기반 최신 가이던스 대비 계산 격차 잔존. 추가로 내가 보는 한계: 형식적 안정성/강건성 보장 부재(선형화 게인마진 휴리스틱 의존), 외부 독립 베이스라인 부재, 6-DoF/공력 미포함 점질량 모델, 단일 시나리오군 검증으로 통계적 일반화 근거 제한적. Future work으로 6-DoF·복잡 모델 확장 시 Fast DOC의 상대 이점이 커질 것이라 주장하나 실증은 미래 과제.

**평가:** 73/100 (good) · 한줄평: DOC를 항공우주 궤적최적화에 실용화한 견고한 in-domain G&C 기여이나, 신규성이 선행 DOC/LQR/축소기법의 정제·조합 수준이고 형식적 보장·외부 베이스라인이 부족하다.

---

## (2009, AIAA GNC, Optimal Terminal Area Strategies and Energy-Tube Concept for a Winged Re-Entry Vehicle: de Ridder·Mooij)

**1. 핵심 아이디어.** 무동력 유익 재진입체(HORUS-2B)의 종말영역에너지관리(TAEM) 구간에서 수직면 최적 전략(최대 사거리·최대 강하)을 특성화하고, 이를 온보드 유도에 활용할 수 있는 요소로 추출한다. 핵심 기여는 "에너지 튜브(energy-tube)" 개념으로, 특정 목표 사거리를 오버슈트 없이 비행 가능한 고도·속도 조합 집합(최소 요구 에너지 경계와 최대 에너지 경계 사이의 단면)을 정의한다. 단일 최소 에너지 값이 모든 초기상태에 대해 충분하지 않다는 점을 보이며, 고도/속도 분포까지 함께 명시해야 함을 강조한다.

**2. 알고리즘.** 점질량 수직면 운동방정식(Eq.1-6)에 에너지 고도 E_h = h + V^2/(2g)를 상태변수로 사용(Eq.19). 에너지 소산율 dE_h/dR = -(D/mg)(1/cosγ)(Eq.21) -> 최대 사거리는 각 에너지 준위에서 항력 최소화(드래그 밸리 추종)로 얻음. 준정상 활공(Eq.12-16): -tanγ = C_D/C_L, 최대 L/D 받음각 프로파일과 등가속도(등 동압) 비행. 최적해는 유전알고리즘(GA)으로 받음각 제어벡터를 에너지 고도 격자에서 탐색. 유도법칙은 PD 동압 추종: α = α* + K_p(q_ref - q_dyn) + K_d·q̇_dyn (α*=7.5°, q_ref=8242 Pa), 게인은 robust design으로 다중 초기상태에 대해 동시 최적화(Table 5, 게인 스케줄링).

**3. 비판적 검토.**
- 이론은 전적으로 고전 이론(Bryson 에너지상태 근사, Vinh 대기비행 최적궤적, 정상활공 극곡선)의 정확한 임포트·재유도이며 새로운 정리·증명은 없다. PD 법칙(Eq.22)은 단순 제시 수준.
- 검증이 시뮬레이션 단일 기체에 국한되고 외부 baseline이 없다. 비교는 내부(정상상태해 vs GA최적해, 사거리 2-6% 개선)에 그치며 몬테카를로·통계 처리가 없다.
- GA가 완전 수렴하지 않아 궤적에 진동이 남는다고 저자 스스로 인정하나, 권고한 지역 최적화 정제는 실제 적용하지 않았다.
- 초음속 최대 사거리에 대해서는 단일 게인집합 robust design이 실패하여 정상상태해로 후퇴했고, 에너지 튜브의 온보드 계획/HAC 알고리즘은 "개발 중"으로 남아 실시간성·복잡도 분석이 부재하다.
- 환경 불확실성(바람·밀도)과 선회비행은 다루지 않고 향후 과제로 미룸 -> 종말영역 실제 적용에는 핵심 요소가 빠져 있다.

**4. 시뮬/실험.** 전부 (sim), HORUS-2B 단일 기체(L 25 m, S 110 m^2, m 26029 kg). 아음속 3개·초음속 3개 초기점(Table 3)에 대해 GA 최적 vs 정상상태 사거리 비교: 예) Startpoint 2 정상상태 34.88 km -> GA 35.97 km; Startpoint 4 134.63 -> 138.93 km(개선 2-6%). 최대 강하(Table 4): Startpoint 1 스피드브레이크 미사용 16.56 km / 풀 사용 14.92 km. Robust design 게인(Table 5)으로 초기 에너지 10 km, V 115-220 m/s 영역 다수 초기상태 궤적 검증(Fig.14-15). 에너지 튜브 단면은 목표 사거리 34.88/100 km 및 15/30/100 km에 대해 도시(Fig.16-17). Baseline: 자기참조(정상상태해)뿐.

**5. Contribution(delta).** 2009년 기준 선행 RLV-TAEM 연구(Kluever&Horneman, Hull et al., Da Costa/Büchner)는 고정 또는 단일 기준 동압 프로파일·HAC 재배치에 기반했다. 본 연구의 delta는 (i) 최대 사거리·최대 강하 양쪽 한계를 에너지 공간에서 명시적으로 특성화하고, (ii) "단일 최소 에너지 값으로는 불충분, 고도·속도 조합까지 명시 필요"라는 통찰을 에너지 튜브 단면으로 형식화한 점이다. 다만 사용된 방법(에너지상태 근사·최소항력 활공·GA·PD 동압추종)은 모두 출판년 이전 확립 기법으로, 기여는 새 알고리즘이 아니라 능력 특성화의 개념적 재구성에 해당한다.

**6. Limitation·Future work.** 저자 인정: GA 미수렴 진동(지역 최적화 권장), 초음속 게인 단일집합 실패(MRAC 권장), 에너지 튜브 온보드 구현 미완, 바람·밀도·선회비행 미반영, 발사 중단/패러포일 시나리오 미탐구. 추가로 평가자 관점: 외부 baseline·MC·정량적 강건성 지표 부재, 실시간 계산비용 미정량, 단일 기체 일반화 한계, PD 동압추종의 안정성·성능 보장 부재.

**평가:** 58/100 (fair) · 한줄평: 고전 에너지상태 이론을 RLV-TAEM에 견실히 적용해 "에너지 튜브"라는 유용한 능력 특성화 개념을 제시했으나, 새 방법론·외부 baseline·온보드 실증이 없어 핵심 G&C 도메인 내 견고한 개념연구 수준.

---

## (2014, JGCD, Entry Guidance: A Unified Method: Lu)

**1. 핵심 아이디어.** 저-양항비(캡슐)부터 고-양항비(극초음속 활공체)까지, 그리고 궤도 진입과 준궤도(suborbital) 진입을 단일 baseline 알고리즘으로 통합 처리하는 수치 predictor-corrector 진입 유도법을 제안한다. baseline은 에너지(e)를 독립변수로 하여 뱅크각 크기 프로파일 하나(scalar σ₀)만을 미지수로 두고 종말 사거리/에너지 조건을 만족시키는 단순·강건한 1-파라미터 근찾기 문제로 환원한다. 그 위에 고도율(altitude-rate) 피드백 보상항을 목적별로 augment하여 phugoid 진동 제거, equilibrium glide 천이, 그리고 가열률·하중계수·동압의 부등식 경로구속을 모두 처리한다.

**2. 알고리즘.** 운동방정식은 무차원·구형 회전지구 3DOF, 독립변수 e=1/r−V²/2. baseline: |σ(e)|=σ₀+(e−e₀)/(e_f−e₀)(σ_f−σ₀), 종말오차 z(σ₀)=s(e_f)−s_f*=0을 Gauss–Newton(=수정 Newton–Raphson) σ₀^{k+1}=σ₀^{k}−λ_k z(σ₀^{k})/[∂z/∂σ₀]로 갱신, 에너지 부족 시 f=½z²의 최소점에서 정지하여 항상 해를 산출. Augmentation: L cos σ_cmd = L cos σ_base − k(ḣ−ḣ_ref). phugoid 억제는 ḣ_ref=ḣ_QEGC (equilibrium glide), 부등식 구속은 시간척도 분리+예측제어로 ḣ_ref를 설정 — a(t+δ)≤a_max에서 u_γ=sin γ_ref를 역산(U_a 등), sin γ_ref=max{sin γ, U_a, U_Q̇, U_q̄}로 다중 구속 동시 처리. 최종패스는 항법 계측 g-load(a)를 직접 사용해 모델 오차 보정.

**3. 비판적 검토.**
- 형식적 안정성/수렴 보증이 없다. Gauss–Newton의 superlinear 수렴, 예측제어의 구속만족은 "δ>0이면 보장된다(완전 실현 가정 시)" 수준의 논증이며, 닫힌루프 안정성·종말정확도에 대한 정리·증명은 제시되지 않는다.
- baseline의 1-파라미터(σ₀) + 선형 σ(e) 파라미터화는 강건성/단순성의 대가로 자유도가 제한적이며, 저자도 aerocapture 등에서는 1-자유도가 과도하게 제약적일 수 있음을 인정한다.
- 핵심 기계(Newton–Raphson 수렴, 시간척도 분리, QEGC)는 인용·차용(Nocedal–Wright, [39], [36])이며 본 논문의 자체 증명은 아니다.
- 불확실성 처리는 fading-memory 필터로 L/L*, D/D*를 보정하는 휴리스틱이며, 별도 공식적 강건성 정리는 없다. δ, k₀ 등 게인은 "수치시험으로 쉽게 정한다"는 식의 튜닝 의존.

**4. 시뮬/실험.** 전부 폐루프 3DOF (sim), 1976 US Standard Atmosphere, 유도 1 Hz 호출. 세 기체: CEV(L/D≈0.28, 8383 kg), X-33(중간 L/D≈0.9), CAV-H(고 L/D≈3.5). 궤도진입(Table 1)+CAV-H 준궤도 3미션(Table 2) 검증, 종말 사거리 정밀도 모든 케이스 1 km 이내. 정량: CAV-H 첨두 가열률 7022.7→4704.5 kW/m²(33%↓), 첨두 하중 3.5→1.56 g(55%↓), 첨두 동압 158→68.5 kN/m²(57%↓); X-33 가열률 1097.7→772.7 kW/m², 하중 2.8→2.3 g; CEV 예측 load-relief로 첨두하중 5.4 g→3.0/4.0 g 구속, 미스 1.5 km 이하. baseline 비교는 자기-비교(보상 유/무) 및 Apollo load-limiter [1]·QEGC-bound [31]에 대한 정성 우위 논증으로, 외부 알고리즘과의 동일조건 정량 벤치마크는 없음.

**5. Contribution(delta).** 출판년(2014) 기준, 진입유도 군이 캡슐(Apollo계)·셔틀(추종형)·predictor-corrector로 파편화되어 있고 특히 predictor-corrector에서 부등식 경로구속(가열·하중·동압)의 신뢰성 있는 일반적 시행수단이 오랜 미해결 난제였다. 본 논문의 delta: (i) 저·중·고 L/D + 궤도/준궤도를 mission-dependent 조정 없이 포괄하는 단일 baseline, (ii) 고-L/D phugoid 진동 제거 및 중-L/D 평형활공 천이를 altitude-rate 보상으로 해결, (iii) 시간척도 분리+예측제어 기반 altitude-rate ḣ_ref 설계로 predictor-corrector 내에서 부등식 경로구속(특히 캡슐 하중·고L/D 가열/하중)을 능동적·정밀하게 시행 — Apollo load-limiter나 QEGC-bound의 reactive/heuristic 한계를 넘어선 점이 핵심 진전이다.

**6. Limitation·Future work.** 저자 인정: CEV·X-33의 잔여 phugoid/altitude loft는 trajectory control과 altitude의 coordination 부족에서 기인하며 본 설계 목표 밖; α가 최대양력에 걸리면 보상 여지 소멸; ḣ_ref 완전추종 불가로 a≤a_max에 소폭 위반 가능; 게인은 수치튜닝 의존. 내가 보는 한계: 형식적 수렴·안정성·구속만족 정리 부재, 전수 Monte-Carlo(분산·항법오차 통계) 부재로 강건성 주장이 정성적, 외부 알고리즘 대비 정량 벤치마크 부재, 온보드 실시간성은 1 Hp 호출·"동등한 계산자원" 진술 수준이며 임베디드/HIL 타이밍 실측은 없음.

**평가:** 86/100 (flagship) · 한줄평: 진입유도의 통합 프레임을 제시하고 predictor-corrector의 부등식 경로구속이라는 장기 난제를 실효적으로 해결한, 핵심 G&C 도메인의 분야 형성급 논문(velocity 32.6/yr, 셔틀/RLV 진입유도 표준 참조).

---

## (2017, AIAA Space Planes Conf, Launch abort trajectory optimisation for reusable launch vehicles: Toso·Maddock)

**1. 핵심 아이디어.** 복합사이클 추진 양력체(lifting-body) 스페이스플레인의 상승 중 추력 상실(abort) 상황을 가정하고, 무추력 활공(glide) 재진입으로 도달 가능한 downrange/crossrange 한계를 궤적 최적화로 도출한다. 상승궤적을 먼저 최적화한 뒤, 그 위의 50개 abort 지점을 시작조건으로 하여 각 지점에서 최대 활공 거리를 구한다. 결과를 "시작 비에너지(specific orbital energy) vs 도달거리" 곡선으로 정리해 abort 능력 지도를 만든다.

**2. 알고리즘.** 3DOF 점질량 동역학(ECEF, 상태 x=[h,λ,θ,v,γ,χ], J2/J3/J4 중력, ISA 대기, X-34/Lazarus 공력 DB). 최적제어문제를 direct transcription + multiple shooting으로 이산화(phase별 n_E 세그먼트, n_C Chebyshev 제어노드, 노드 사이는 piecewise cubic Hermite, 4차 고정스텝 RK 적분). 해법기는 MATLAB fmincon의 SQP(단일목적). 초기추정은 Latin Hypercube Sampling 풀에 완화된 제약·3배 큰 시간스텝으로 multistart, 이후 인접 abort 지점으로 warm-start 연쇄. 비용함수: 상승=최종 payload 최대화, abort=Haversine downrange 또는 최종 위도(crossrange) 최대화. 제약: |a_{x,z}|≤6g0, max(Q)≤20 kPa, AoA∈[-4,24]°, bank∈[-80,80]°.

**3. 비판적 검토.**
- 방법론적으로 신규성이 없다. multiple-shooting direct transcription + SQP(fmincon) + multistart는 모두 표준 도구의 조합이며, 저자 고유의 정리·증명·수렴보장이 전혀 없다. 본질적으로 엔지니어링 응용 연구다.
- 검증이 빈약하다. 차량 1종, abort 50점에 대한 단일 케이스 분석뿐이고 외부 baseline(타 기관 방법/결과)과의 비교, Monte Carlo, 통계적 처리가 없다. 선행연구 참조(ref 1-3)도 대부분 동일 그룹 자기인용이라 독립적 baseline이 부재하다.
- 불확실성 처리가 사실상 없다. 공력 DB 외삽(Mach 도메인 밖에서 최근접 경계값 고정), 대기/추진 모델 오차, 분산(dispersion)에 대한 robustness 분석이나 보장이 전무하다. abort라는 안전-임계 시나리오를 다루면서 nominal-only 해만 제시한다.
- crossrange 분석에서 "faulty behaviour"(고에너지 재진입 시 spiral 진입)를 노드 구성을 임의로 둘로 나눠 회피했다고 서술하는데, 이는 수치적 인공물에 대한 ad-hoc 대처로 해의 신뢰도를 약화시킨다.
- 실시간/온보드 관점이 아니다. multistart fmincon은 저자 스스로 "computationally expensive, prohibitive"라 밝힐 만큼 오프라인 배치용이며, warm-start도 다수 지점의 일괄 평가를 위한 것이지 비행 중 재계산을 의도하지 않는다.

**4. 시뮬/실험.** 전부 시뮬레이션(sim). 명목 상승: payload 1000 kg를 200 km 원궤도 투입, m_glow=90035 kg, 최종 m_p=999.933 kg, 분리된 공기흡입/로켓 추력 T_SL,max=3.894 MN, T_vac,max=4.984 MN. Abort: 고도 1 km<h0<108 km, 속도 271<v0<6327 m/s 범위의 50점, 6초 등간격. 대표 5점(A~E) 제시(예: E점 h=107.65 km, v=6327.9 m/s, m=30638 kg). 결과: downrange 최대 약 12,500 km(E점), crossrange 최대 약 3,950 km(E점, bank 제어 시)로 비에너지에 거의 선형 비례하는 front 곡선. 외부 baseline 없음, MC 없음, HIL 없음.

**5. Contribution(delta).** 선행(동그룹 ref 1-3의 상승/payload 최적화)을 abort 시나리오로 확장하여, 상승궤적 위 임의 지점에서의 무추력 활공 도달능력(downrange·crossrange)을 비에너지 축으로 매핑하는 "abort 능력 지도"를 제시한 점이 delta다. warm-start 연쇄로 다수 abort 지점을 효율적으로 푸는 워크플로도 실무적 기여다. 다만 2017년 출판 시점 기준으로도 spaceplane abort/활공 거리 분석과 direct-shooting+SQP 궤적 최적화는 이미 확립된 영역이어서, novelty는 새 방법이 아니라 특정 차량에 대한 분석 적용에 그친다.

**6. Limitation·Future work.** 저자: skip 재진입은 구조 하중·재사용 수명·온보드 제어/안정성 문제가 있어 더 엄격한 가속/하중 제약이나 후처리가 필요(다만 그러면 suboptimal), 본 연구는 최대 능력 평가에 한정. 추가로 내가 보는 한계: (i) robustness/불확실성 정량화 부재로 안전-임계 abort 결론의 신뢰성 제한, (ii) 외부 baseline 부재, (iii) 6DOF·실제 비선형 제어/자세 동역학 미반영(3DOF 점질량), (iv) 공력 DB 외삽 처리의 임의성, (v) 온보드 실시간성 미확보. Future work로 dispersion 기반 MC, 실시간 가능한 convex/수치 가이던스, 6DOF 검증이 필요.

**평가:** 43/100 (poor) · 한줄평: 표준 SQP·multiple-shooting 도구로 수행한 단일 차량 abort 활공거리 분석 연구로, core-G&C 주제이지만 방법 신규성·검증·불확실성·온보드성이 모두 약하고 인용영향(0.4/yr)도 미미하다.

---

## (2019, JSR, High-Precision Adaptive Predictive Entry Guidance for Vertical Rocket Landing: Zhang·Liu·Liu)

**1. 핵심 아이디어.** 재사용 발사체(RCV)의 대기권 진입 단계에서, 그리드핀으로 받음각 α와 옆미끄럼각 β를 동시에 제어하는 predictor-corrector 진입 유도법을 제안한다. 전통적 BTT(bank-to-turn) 기반 셔틀/화성탐사선 유도와 달리, RCV는 양력 대비 항력이 큰 화살형 기체이므로 σ(뱅크각) 대신 α·β를 직접 제어 변수로 삼는다. 핵심 기여는 onboard 모델과 실제 대기 환경의 불일치(공력계수·밀도 오차)를 두 개의 가속도 편차계수 K_D, K_L로 묶어 실시간으로 적합(fitting)함으로써 predictor의 방법론적 오차를 줄여 미터급 종말 정밀도를 얻는 것이다.

**2. 알고리즘.** 회전 타원체 지구 위 3자유도 운동방정식(식 1-6, J2·코리올리·원심 가속도 포함). 공력가속도는 [X,Y,Z]^T = (1/m)[C_D qS, C_L αqS, C_L βqS]^T, q=½ρV². 제어 프로파일은 시간 선형 램프로 α=-α_k(t-t_s) (|α|≤α_max에서 포화), β도 동형. 적응 적합: onboard 측정 가속도와 binding 모델값의 비로부터 K_D, K_L을 매 스텝 산출(식 16). Predictor-corrector는 종말 경도·위도 (θ_f, φ_f)를 현재 α_k, β_k로 1회 예측하고, 야코비안 J=[∂θ/∂α_k, ∂θ/∂β_k; ∂φ/∂α_k, ∂φ/∂β_k] (식 20)를 구성해 J[dα_k; dβ_k]=[Δθ; Δφ] (식 21)을 elimination으로 2-3회 뉴턴 반복하여 α_k, β_k 갱신.

**3. 비판적 검토.**
- 적합 방법이 K_D, K_L을 "한 스텝 전후 속도·궤도각 일치"(식 17-18)로 역산하는데, 비행 초기 q<50(저동압) 구간에서는 공력이 미약해 적합이 작동하지 않음을 저자 스스로 인정(Fig.4-5). 즉 적합 유효 구간이 제한적이고, 측정 잡음에 대한 필터가 없어 결론에서 "filter가 필요하다"고 후술한다 — 즉 제안 기법 자체가 잡음에 취약함을 시인.
- 제어 프로파일이 단순 시간 선형 램프(2개 스칼라 α_k, β_k)로 매우 제약적이다. 단 두 자유도로 종말 위치 2개 제약(θ_f, φ_f)만 맞추며, 종말 속도·비행경로각은 직접 제약하지 않는다(RCV는 nonlifting body라 충분한 제어력이 없다고 변명). 따라서 "high-precision"은 위치 한정이고 속도·자세 종말조건은 보장되지 않는다.
- 이론적 보증 부재: 뉴턴 반복의 수렴성·유일성·안정성에 대한 증명이 전혀 없고, 야코비안 가역성도 가정뿐이다. 1000회 MC 통계가 유일한 근거.
- K_D, K_L이라는 lumped 보정은 밀도·C_D·C_L·질량 오차를 하나의 스칼라로 합산하므로, 서로 다른 오차원이 상쇄·왜곡될 수 있는 ill-posed 가능성. 식별 가능성(identifiability) 논의가 없다.
- 외부 baseline 비교가 없다. "전통 predictor-corrector보다 낫다"는 정성적 주장만 있고, 동일 시나리오에서 기준 알고리즘을 직접 돌린 정량 비교가 없다(Lu의 unified method[5], Wang[4] 등 인용만).

**4. 시뮬/실험.** 전량 (sim). 1000회 Monte Carlo, 분산: ρ±20%, C_D±20%, C_L±20%, m±10kg, V±50m/s, X±5000m (Table 1). 발사장 회수와 해상 회수 두 시나리오. 종말 셔트다운점 편차(Table 3): 발사장 회수 X 평균 -9.9m/분산 1.7, Z 평균 15.7m/분산 21; 해상 회수 X 평균 13.8m/분산 39.6, Z 평균 -10.6m/분산 56.3. 즉 미터급 정밀도 주장. baseline 정량 비교는 없음(self-claim). 실비행/HIL 없음.

**5. Contribution(delta).** 출판년(2019) 기준, predictor-corrector 진입 유도(Lu[5], Wang[4] 2017, Kozynchenko[9])를 화성 lander/셔틀에서 RCV 수직회수로 이식하고, σ(뱅크) 제어를 α·β 그리드핀 제어로 치환한 점이 delta. 추가로 onboard 모델-실환경 불일치를 K_D, K_L 두 계수의 실시간 적합으로 보정하는 단순한 적응층을 결합. SpaceX Falcon 9 회수 맥락에 진입유도를 맞춘 응용적 신규성은 있으나, 방법론 골격(predictor-corrector + Newton on terminal constraints)은 기존 것의 재구성에 가깝다. Acikmese 류의 lossless convexification 같은 field-shaping 기여와는 거리가 멀다.

**6. Limitation·Future work.** (저자) 저동압 구간 적합 불가, 측정잡음 필터 미적용, α_max 포화로 종말부 비행안정성 영향; 향후 reference trajectory 생성에 수치 predictor-corrector 활용. (추가) 종말 속도·자세 제약 미보장, 외부 baseline 부재로 "high-precision" 주장이 상대적 우위로 입증되지 않음, 수렴·강건성 이론 부재, K_D/K_L lumping의 식별가능성 미검증, 재현 파라미터(전 6식의 기준궤적·binding table)는 일부만 제시되어 재현성 낮음(코드 비공개).

**평가:** 56/100 (fair) · 한줄평: RCV 수직회수에 진입 predictor-corrector를 α·β 그리드핀 제어로 이식하고 간단한 2계수 적응층을 더한 견실한 도메인-적합 응용이나, 외부 baseline·이론적 보증·종말 속도제약이 빠져 영향력은 제한적(velocity 1.6/yr).

---

## (2019, Eng. Optimization, Direct trajectory optimization framework for VTVL reusable rockets: two-stage case study: Ma·Wang·Shao·Song·Biegler)

**1. 핵심 아이디어.** 2단 VTVL 재사용 로켓의 전체 비행(상승+1단 회수 강하)을 두 단을 별도 모델링한 2-phase 궤적 최적화 문제로 정식화하고, 유한요소 Radau collocation으로 NLP로 변환해 IPOPT로 푸는 직접법 프레임워크를 제시한다. 핵심 기여는 복잡 제약(글라이드슬로프·동압·visual accel.·단분리 interior-point 제약)이 만드는 좁은 feasible 영역에서 NLP의 수렴을 돕는 "internal-external-growth" 초기화 전략이다. RTLS(발사장 귀환)와 RTDS(드론십 귀환) 두 시나리오로 검증한다.

**2. 알고리즘.** 점질량 동역학: ṙ=v, v̇=-μr/||r||³+T/m+D/m, ṁ=-||T||/(g0·Isp). 목적함수 J=-m2^(2)(tf2) (2단 잔여연료 최대화). 제약: 글라이드슬로프 (r1^(2)-rf1)ᵀrf1 ≥ ||r1^(2)-rf1||·||rf1||·cosθgs, 1단 추력 Tmin≤||T||≤Tmax(throttle, 9엔진), 동압 ½ρ||v||²≤qmax, 단분리 연속성 r/v/m 연결. 전치: 각 유한요소에서 상태/제어를 K+1 Radau점 Lagrange 다항식으로 근사 → collocation 방정식 Σ(dLj/dτ)Zij-hi·f(Zik,Uik)=0. IPOPT는 barrier 문제 min f(X)-ε·Σln(X^(i)) s.t. c(X)=0를 damped Newton + filter line-search로 풀고 ε를 초선형 감소. 초기화 5단계: (1) 경로제약 없는 저밀도 해 → (2) 전체 저밀도 해 (primal/dual 전달) → (3) 저장 → (4) 고밀도 collocation에 primal+dual 보간 초기값 → (5) 출력.

**3. 비판적 검토.**
- 자체 이론 기여는 없다. collocation↔이산화 Euler-Lagrange의 KKT 일치, IPOPT barrier 수렴 등 핵심 기계는 모두 인용(Biegler, Wächter-Biegler, Kameswaran-Biegler)된 것이고, 본문은 그 적용이다.
- 결정론적·공칭 해만 다룬다. 분산·몬테카를로·불확실성·강건성 분석이 전무하다. 재사용 로켓 회수는 본질적으로 환경/모델 불확실성이 지배적인 문제인데 이를 다루지 않아 실용 강건성 주장이 약하다.
- 모델 단순화가 과하다(진공추력 고정, CD·Aref·ρ 단순화, 양력 무시, 추력=속도 반대방향 아님에도 자세동역학 없음). 회전·자세·공력모멘트가 빠진 점질량 모델은 강하 회수의 핵심 난점(자세제어·재진입 가열·바람)을 포착하지 못한다.
- baseline이 자기참조뿐이다. "conventional initialization은 수렴 실패, 제안법은 수렴 성공"이 유일 비교이며, 외부 방법(pseudospectral, 다중사격, convex/LCvx)과의 정량 비교가 없다.
- 검증은 ode45 재전파 일치와 메쉬세분 수렴(37~45 요소)으로 self-consistency는 보였으나, 최적성/전역성 보장이나 외부 진리값 대비는 없다.

**4. 시뮬/실험.** AMPL+IPOPT 3.8.0, tol 1e-8, i7-4710MQ. 시나리오 2개(RTLS, RTDS). RTLS: 단분리 131.4s, 2단 궤도투입 453.4s, 1단 착륙 569.7s, 1단 추력 bang-bang, 고밀도 해 계산 1.9s/115 iter. RTDS: 단분리 134.6s, 착륙 528.4s, 1.6s/90 iter, 드론십 위경도 (28.0°,1.0°). 동압·visual accel.이 제약(qmax 80kPa, accmax 10.0g0) 내 만족 확인. 메쉬 세분(1단 11~20, 2단 16~25 요소)에서 해 일치로 수렴 정당화. baseline=공칭 초기화(수렴 실패)뿐, MC·외부방법 비교 없음 (모두 sim).

**5. Contribution(delta).** 출판년(2019) 기준, 단일단 powered-descent(Acikmese convex/LCvx, Liu 2017) 및 자기 선행 달착륙 collocation(Ma 2016b/2018)을 넘어 2단 로켓의 상승-회수를 단분리 interior-point 제약으로 결합한 단일 2-phase 정식화를 제시하고, primal+dual을 함께 보간하는 internal-external-growth 초기화로 복잡 제약 NLP의 수렴난을 완화한 점이 delta다. 새 방향을 여는 방법론은 아니고, 확립된 collocation/IPOPT 도구를 재사용 로켓 전 과정에 적용·통합한 점진적 엔지니어링 기여다.

**6. Limitation·Future work.** 저자: 점질량 단순화 모델이며 임의의 비행환경/임무제약은 등식·부등식으로 확장 가능한 open framework라고 명시. 내가 보는 한계: (i) 불확실성·강건성·온라인 적용성 부재(1.6~1.9s offline 설계 도구이며 onboard 실시간 주장 없음), (ii) 자세동역학·공력모멘트·바람 미반영으로 회수 강하의 실제 난점 미포착, (iii) 외부 baseline 부재로 효율/최적성 우위 정량 미입증, (iv) 인용속도 3.6/yr로 영향력은 제한적. Future: 6-DoF/강건 최적화, convex 워밍스타트와의 결합, 실시간 재계획.

**평가:** 61/100 (fair) · 한줄평: 정식화·수치적 정합성은 견실하나 자체 이론·불확실성·외부 baseline이 빠진 공칭-결정론적 직접법 적용 연구로, 2단 회수 전과정을 통합한 점은 in-domain core-G&C로서 의미 있으나 영향력은 보통.

---

## (2019, JGCD, Guidance of Reusable Launchers: Improving Descent and Landing Performance (DESCENDO): Simplício·Marcos·Bennani)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 하강·착륙(D&L) 유도를 위한 폐루프 볼록 최적화 알고리즘 DESCENDO(DEScending over extended envelopes using successive CONvexification-based Optimization)를 제안한다. 먼저 단순 CTV(구속 종단속도) 유도로 추진제 질량·최대 동압·착륙속도 간 트레이드오프 맵을 구축해 성능 한계를 분석하고, 이를 바탕으로 효율 중심(Jerez/FORCES, 공력 무시)과 최적성 중심(Szmuk, 일정밀도·저고도 한정) 사이의 "중간 지점"을 노리는 successive convexification 기반 유도를 설계한다. VEGA 대표 경량 수직이착륙 부스터 벤치마크에서 DRL(드론십)·RTLS(발사장 복귀) 전 영역 회수 시나리오를 폐루프로 검증한다.

**2. 알고리즘.** 연료 최적 궤적 생성을 SOCP로 캐스팅. 비볼록성 두 가지를 처리: (1) 질량소모 동역학은 z(t)=ln m̂(t) 좌표변환으로 lossless convexification, (2) 추력 크기·방향 제약은 w(t)=T/m̂, σ(t)=‖T‖/m̂ 변수 도입 + ‖w[k]‖≤σ[k] 완화(z[N] 최대화 시 등호). SOCP 1: 공력 무시, min Σσ[k]=max z[N], 경계조건·이산 동역학(식13)·방향제약 w_z≥‖w_xy‖/tanθ_max·추력 한계 T_min/m̂≤σ≤T_max/m̂·추력률 제약(식19). SOCP 2: 항력 가속 a[k]=w[k]+ĝ−d*[k]v[k] (d*=½ρ*S_ref C_D*‖v*‖/exp z*)로 공력을 선형화 보강하고, 트러스트 영역 제약(TRC) ‖w−w*‖≤η_w, 비행경로(원뿔) 제약 r_z≥(r̂_z/‖r̂_xy‖)‖r_xy‖ 추가, 목적함수 max z[N]−w_η Ση_w. 유도 스텝마다 SOCP 1 한 번 + SOCP 2를 N_p회 반복(successive convexification 루프), MATLAB+CVX+ECOS 구현, f_gui=10Hz 폐루프.

**3. 비판적 검토.**
- Lossless/successive convexification의 핵심 이론(완화의 타당성·수렴성)은 모두 인용·수입이며, RLV 확장 영역에서 완화가 tight한지에 대한 자체 증명은 없다. SOCP 2의 공력 선형화 수렴도 경험적으로만 제시.
- baseline이 저자들의 선행 벤치마크[24]의 CTV 유도뿐이다. Szmuk·Jerez 등 가장 가까운 선행 알고리즘과의 직접 정량 비교가 없어 "중간 지점"이라는 핵심 주장(효율 대 최적성)이 수치로 입증되지 않는다.
- Monte Carlo 분산 해석이 없다. 임무 프로파일에 dispersion이 언급되나 바람·공력·초기조건 산포에 대한 정량 강건성 평가는 부재하며 공칭 폐루프 검증에 머문다.
- 완벽 자세제어 가정으로 6-DOF 회전 동역학·자세-궤적 결합을 분리했다. RLV의 가장 어려운 부분(CP가 CG 앞→공력 불안정) 영향이 유도 단에서 빠져 있다.
- N, f_gui, θ_max, w_η 등이 pattern search로 고정 튜닝되었고 그 민감도·일반화는 미평가(저자도 실세계 적용 전 필수라 명시).

**4. 시뮬/실험.** VEGA 대표 1100kg급 페이로드 비행성 경량 부스터의 비선형 6-DOF 벤치마크[24]에서 DRL·RTLS 두 시나리오 (sim). 공력계수·질량소모·CP/CG 이동 모델링. 정량: DRL은 N_p=2에서 CTV 대비 추진제 6.9% 절감; RTLS는 N_p=2에서 16.4% 절감, m_rec 8.06t·Q_max 50.1kPa·v_f 1.63m/s로 CTV(8.37t·60kPa·2m/s) 대비 310kg·16% 동압 동시 개선. ECOS 코어 솔버시간 100s 회수분에 대해 N_p={0,1,2}에서 각각 0.6·1.3·2.2s, 저고도 공력루프 차단으로 ~30% 추가 단축. baseline=자체 선행 CTV 유도(외부 SOTA 비교 없음), MC 없음.

**5. Contribution(delta).** 출판년(2019) 기준 delta: (1) successive convexification을 RLV의 확장 비행영역(재진입~터치다운 전구간, 가변 공기밀도, 공력 감속)으로 확장—Szmuk[25]의 일정밀도·저고도·저속 한정 가정을 제거. (2) Jerez[26](공력 무시, 효율 우선)와 Szmuk(최적성 우선) 사이의 명시적 중간 지점 설계. (3) DRL/RTLS 완전 회수 시나리오에 대한 폐루프 검증(저자 주장 "아직 정식 연구된 바 없음"). (4) CTV 트레이드오프 맵 기반 성능 한계 분석 방법론. 새 방법론 창시가 아닌, 확립된 볼록화 기법의 RLV 전영역·폐루프 적용 확장.

**6. Limitation·Future work.** 저자: 완벽 자세제어 가정(자세제어·그 최소화는 robust load prediction/relief로 별도 연구 진행 중), t_f를 최적화 변수로 두지 않고 고정(CTV와 공정 비교 위함), N 고정, 실세계 적용 전 f_gui·N 영향의 철저한 평가 필요. 내가 보는 한계: 외부 SOTA 알고리즘 대비 정량 비교 부재로 "중간 지점" 주장 미검증; MC·형식적 강건성 보장 없음; 자체 baseline; 회전 동역학·공력 불안정성을 유도 단에서 분리; FORCES/CVXGEN 언급뿐 실제 임베디드/비행 시연 없음.

**평가:** 70/100 (good) · 한줄평: RLV 하강 전영역에 successive convexification을 폐루프로 확장하고 온보드 솔버시간을 명시적으로 제시한 견실한 in-domain G&C 논문이나, 외부 baseline·Monte Carlo 부재와 수입된 핵심 이론으로 인해 flagship에는 못 미친다.

---

## (2020, CCC, Full-stage Reentry Trajectory Optimization for Reusable Launch Vehicle: Wu·Tian·Li)

**1. 핵심 아이디어.** 120km에서 지상까지의 RLV 재진입을 IRE(초기재진입+활공), TAEM(말단 에너지관리), A&L(접근·착륙)의 3단계로 나누어 각 단계 특성에 맞는 궤적최적화 기법을 단계별(stage-by-stage)로 적용하고, 각 단계의 종단상태를 다음 단계의 초기조건으로 넘겨 다중-세그먼트 전환점에서의 상태 불연속을 해소하는 full-stage 전략을 제안한다. IRE는 가변 trust-region을 갖는 sequential convex programming(SCP), TAEM은 기준 동압(dynamic pressure) 프로파일 기반 받음각 설계와 지상궤적 추종 뱅크각 설계, A&L은 quasi-equilibrium glide(QEG)와 지수감쇠 하강 서브페이즈를 secant법으로 접속하는 방식이다.

**2. 알고리즘.** 3DOF 병진운동방정식(식1-6)에 제어변수를 u=σ̇로 재정의해 ẋ=f(x)+Bu(식17-19)로 affine화 → 1차 Taylor 선형화 + trust-region ‖x−x*‖≤ε(2차 SOCP, 식20) + 사다리꼴 이산화(식21-23)로 convex 문제 P2 구성, YALMIP+SeDuMi로 SCP 반복 풀이. 실제/예측 성능지수 η',η와 차분 Δη',Δη 비교로 trust region 갱신. TAEM: 동압-고도 3차다항 q̄_ref(h)=Q0+Q1h+Q2h²+Q3h³(식24)을 운동방정식 미분(식25-27)과 비교해 C_D→α 산출, 뱅크각은 σ=K_v·Δψ_AC 등 지상궤적 추종. A&L: QEG에서 α_gliding1, 지수하강 tanγ=tanγ_TD·e^{−ks}(식28)에서 α_gliding2 도출, 접속고도 h_F를 Gauss-Newton/secant로 결정.

**3. 비판적 검토.**
- 핵심 수치기법(SCP 선형화, trust-region SOCP, 사다리꼴 이산화)이 모두 인용 선행연구(Lu&Liu 2013, Wang&Grant 2017)에서 차용된 표준 절차이며, 수렴성·실현가능성에 대한 자체 증명이나 정리는 없다(C1 약함).
- 가변 trust-region 갱신 규칙, TAEM의 3차 동압 프로파일 선택, A&L 접속고도의 secant법 등 설계선택이 휴리스틱이며 정당화·민감도 분석이 없다.
- 검증이 단일 명목(nominal) 시나리오 1회뿐이고, 외부 baseline 비교나 Monte-Carlo·분산해석이 전혀 없어 "reliably"라는 주장에 대한 정량적 뒷받침이 부족하다.
- 계산시간(2.8GHz MATLAB)이 명시되지 않아 SCP의 실시간성 잠재력 주장이 검증되지 않았다.

**4. 시뮬/실험.** 단일 명목 시나리오 (sim): x0=[120km,7600m/s,…], xf=[0km,200m/s,…], m=9.21e4kg, S=250m², 제약 Q̇max=1e7, qmax=3e3, nmax=10. 3차원 궤적·에너지-고도/속도/경도/위도/비행경로각/헤딩각 곡선(Fig.1-7)으로 전 단계 연결의 평활성과 명목 착륙점 도달을 정성적으로 제시. 정량 오차지표·MC·baseline 비교 없음.

**5. Contribution(delta).** 출판년(2020) 기준 선행은 대부분 단일 단계(IRE 또는 TAEM 또는 A&L)에 한정된 반면, 본 논문은 세 단계를 하나의 full-stage 파이프라인으로 연결하고 전환점 상태 불연속을 종단-초기 상태 인계로 봉합한 점이 delta다. 다만 각 서브기법(SCP, QEG, 지수하강)은 기존 방법의 조합·차용으로 방법론적 신규성은 점진적(incremental)이다.

**6. Limitation·Future work.** 저자는 명시적 한계 언급이 거의 없다. 내가 보는 한계: (i) 불확실성·교란·강건성 부재(명목 전용), (ii) 외부 baseline·MC 부재로 우월성 미입증, (iii) 실시간/온보드 적용성에 대한 계산복잡도·타이밍 분석 부재, (iv) 자체 이론보증(수렴·feasibility) 없음, (v) 단일 차량·단일 궤적으로 일반화 미검증.

**평가:** 49/100 (weak) · 한줄평: full-stage 단계연결 아이디어는 실용적이나, 기법이 모두 차용·명목검증 1회·baseline/MC 부재로 in-domain G&C 저널 기준엔 미달하는 코어-GNC 컨퍼런스 수준 기여.

---

## (2021, AST, Recovery trajectory planning for the reusable launch vehicle: Cheng·Jing·Gao)

**1. 핵심 아이디어.** 재사용발사체(RLV) 1단의 전체 회수 과정(6단계: FM·CC·UAUD·UAPD·EAUD·VL)에 대한 궤적계획을 다룬다. 두 가지 기여를 제시한다. (1) 수정 궤적보정법(RTCM): 수직착륙(VL)단의 점화점을 수평면상으로 이동시키고 정상상태 풍장(preset steady wind field)을 궤적계획에 도입하여, 대기권 내 비추력 하강단에서 최대 법선 공력하중을 줄이고 착륙정확도를 개선한다. (2) PSO(입자군집최적화)와 다항식유도법칙(PGL)을 결합한 복합 궤적계획법으로 전체 회수문제의 최적화를 수행하며, 기본 PSO의 조기수렴을 막기 위해 재초기화 메커니즘을 개선한다.

**2. 알고리즘.** 점질량 LCS 운동방정식 r˙=v, v˙=(P+R)/m+g, m˙=-||P||/Isp. 최적화변수 6개 {Δv_B,1z, Δv_B,2x, Δv_B,2y, α^EA, β^EA, h^EA}를 PSO(n=6, m=50, iter=20)로 탐색. 입자갱신 v_id(t+1)=w·v_id+c1·r1·(p_id-x_id)+c2·r2·(p_gd-x_id), 선형감소 관성가중 w=w_max-(w_max-w_min)t/t_max, 시변 가속계수 c1·c2. 적합도 F=|Δm|+K_q·q_max+(q·η_w)_max+K_EA·||r_err^EA||+K_VL·(||r_err^VL||+||v_err^VL||). 재초기화 규칙(33): D_xi≤ε 또는 D_vi≤ε이면 속도 재초기화. VL단은 PGL로 a_f=a0+a1·t_go+a2·t_go², t_go는 4차식의 Newton 반복 t_go*=t_go0-f/f' (3회 반복)로 계산, 추력명령 P_c=m(a_f-12ρ0/t_go*²-6(v0+v_f)/t_go*-g_l-a_Rl).

**3. 비판적 검토.**
- 본질적으로 휴리스틱 최적화(PSO) 기반이며 수렴성·최적성에 대한 형식적 보장이 전혀 없다. 이론적 정리·증명 없이 좌표변환·점질량 EOM·Tsiolkovsky·Newton 반복 등 교과서 수준의 기계적 유도에 머문다.
- 핵심 설계상수들이 임의적이다. 기대 종단위치 계수 t_E=-5, 적합도 가중치 K_q=10⁻², K_EA=10³, K_VL=10⁵를 "동일 자릿수로 맞추기 위해" 선택했다고만 기술하며 정당화가 부족하다.
- baseline이 자기정의다. Case a("전통적 보정법")는 저자가 구현한 비교군이며, 외부/실제 baseline이나 타 최적화기(convex/직접법 등)와의 정량 비교가 없다.
- α^EA를 약한 음수로 두어 종단속도 v_fx^EA를 양수로 만들어 자연감속을 유도한다는 핵심 메커니즘은 합리적이나, 부호 정렬에 의존하는 정성적 논거로 정량적 민감도 분석이 빠져 있다.
- PGL을 달착륙(Apollo LM, Chang'E-3) 유도에서 가져온 import이며, 전체 회수문제 결합 자체가 기여이지 PGL·PSO 각각은 기존 기법이다.

**4. 시뮬/실험.** Falcon 9 기반 파라미터(m0=35t, S_ref=10.75m², Isp=400s). 3개 케이스(a/b/c, PWF·MIP 조합)와 3000회 독립 Monte Carlo. 풍장은 고도별 정규/균일분포로 모델링(상층풍 평균 19~129m/s, 지표 0~100m N(10,(5/3)²)). 정량결과: 60km 점화로 최대 동압 56.88% 감소(sim), EAUD 최대 법선공력하중 15.35% 수준으로 감소(sim), 궤적계획 평균 8.6010분 소요(5회 평균). 종단편차 히스토그램(Fig.5,6)·평균표(Table 9,10) 제시. MIP 적용 시 소비연료 8055.0→7835.7kg 감소. baseline은 자기구현 Case a.

**5. Contribution(delta).** 출판년(2021) 기준, RLV 1단 전체 회수단계를 단일 최적화문제로 묶어 PSO+PGL 복합계획법으로 푼 점, 그리고 VL 점화점 수평이동+정상풍장 도입으로 공력하중과 착륙정확도를 동시에 다룬 RTCM이 delta다. 다만 구성요소(시변계수 PSO[26], 재초기화[27,28], PGL[13-15])는 모두 선행 import이며, 새 G&C 방법론을 여는 수준이 아니라 알려진 부품의 공학적 조합·점진적 개선이다.

**6. Limitation·Future work.** 저자: 법선 공력하중을 더 줄이려면 하중경감제어를 자세제어에 도입하는 방향이 더 매력적이라고 언급(향후과제). 내가 보는 한계: (i) PSO의 전역최적성·반복적 재현성 보장 부재(8.6분 오프라인 계획으로 온보드 실시간성 미흡), (ii) 6자유도·실제 공력DB·엔진 동역학 미반영(점질량 가정), (iii) 외부 baseline·통계적 유의성 검정 부재, (iv) 풍장 모델이 정상상태 가정에 강하게 의존하여 비정상 돌풍 강건성 미검증.

**평가:** 56/100 (fair) · 한줄평: RLV 전체 회수궤적을 PSO+PGL로 묶은 견실한 도메인내 G&C 공학논문이나, 휴리스틱 의존·자기 baseline·이론적 보장 부재로 표준 수준에 머문다.

---

## (2023, IMechE G, Multi-phase and dual aero/propulsive rocket landing guidance using successive convex programming: Lee·Lee)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 연료 최적 연착륙 유도를 6-DOF 모델 기반으로 정식화하고, 추력벡터제어(TVC)와 공력제어(AC) 핀을 동시에 사용하는 dual aero/propulsive 제어를 도입한다. AC 핀이 자세 제어용 모멘트를 분담해 TVC가 자세 제어에 낭비하는 추력(T·sin δ_T 손실)을 줄여 연료를 절감한다는 시너지 효과가 핵심이다. 나아가 공력 유도 단계(AC만)와 수직 착륙 단계(TVC+AC)를 하나의 비볼록 자유종단시간 문제로 묶어 연속 상태트리거 제약(STC)으로 다단계화하고, 이를 SCVX(successive convexification)로 SOCP 부문제 열로 변환해 푼다.

**2. 알고리즘.** 병진/회전 6-DOF: ṙ=v, v̇=F_I/m+g, ṁ=−T/(g0 Isp), q̇=½Ω·q, ω̇=I_B^{-1}(M_B−ω×I_Bω). 총력/모멘트 F_I=C_{I/B}(T_B+A_B), M_B=r_{p,B}×T_B+M_{B,aero}로 AC가 힘과 모멘트에 모두 기여. 목적함수 J=−m(t_f). 정규화시간 τ∈[0,1]과 시간팽창계수 η=t_f로 자유종단시간을 고정시간화(x'=ηf), 1차 테일러 선형화로 x'≈F_x x+F_u u+F_η η+R, 사다리꼴 이산화. 다단계는 연속 STC ζ(x)=−min(f_trigger(x),0)·c(x)=0로 고도 h_L 기준 추력하한·노즐각 제약을 if-then 없이 표현. 노즐각/추력하한의 비볼록 제약은 소각근사+lagging으로 선형화, 가상제어 슬랙(v1,v2)·가변 신뢰영역(ξ) 페널티로 부문제 실현가능성 확보(P3: min −m_{Nk}+w_ξ ξ+w_v v).

**3. 비판적 검토.**
- 이론적 신규성보다 기존 기법의 결합·적용에 가깝다. SCVX(refs 15–17,21), STC(ref 21), lagging(ref 23), 가상제어/신뢰영역(refs 5,24), 정규화시간(ref 17) 모두 차용이며, 저자 스스로 "수렴은 아직 이론적으로 증명되지 않았다"고 명시한다. 자체 수렴/최적성 보장은 없다.
- 검증이 전부 시뮬레이션이고 단일 차량 모델·공칭 동역학에 한정된다. 비교 baseline은 SQP(자체 구현)와 TVC-only(내부 ablation)뿐으로 외부 공개 벤치마크가 없다.
- Monte-Carlo(300회)가 초기 위치 분산만 다루고, 공력계수/질량/돌풍 등 모델 불확실성·외란은 포함하지 않아 robustness 주장이 제한적이다. AC 효용이 공력계수표에 강하게 의존하는데 그 표가 제시되지 않아 재현·일반화가 어렵다.
- 소각근사 정당화를 위해 slew 한계의 80%만 사용하는 등 근사 마진 처리는 합리적이나, 그만큼 제약 활용도가 보수적이다.

**4. 시뮬/실험.** i7-8700, MOSEK(IPM), N_k=150. 다단계 착륙: t_f=15.16s, 연료 252.44kg, 6회 반복으로 수렴, 부문제당 0.2–0.25s. Dual vs powered 비교(표4): TVC-only 13.16s/263.54kg vs TVC+AC 12.12s/247.40kg → 연료 16.14kg(약 6%) 절감, 자세각·각속도 변동도 작음 (모두 sim). SQP 비교(표5): 제안법 247.40kg/1.5s vs SQP 246.39kg/129.34s — 거의 동일 연료를 ~86배 빠르게 (sim). MC 300회(Y_{I,0}~U[10,80], Z_{I,0}~U[−80,−10]): 전 케이스 착륙 수렴, 초기조건 분산에 대한 robustness 시연 (sim).

**5. Contribution(delta).** 출판년(2023) 기준 가장 가까운 선행은 Liu(ref 20, 3-DOF에서 공력제어를 위치/속도 변경용 힘으로만 사용)와 Szmuk 계열 6-DOF powered landing(refs 16–17, TVC만). 본 논문의 delta는 (1) 6-DOF에서 AC를 자세 모멘트 생성에 명시적으로 활용해 TVC 자세제어 추력손실을 보상하는 dual aero/propulsive 정식화, (2) 공력 유도+수직 착륙을 연속 STC로 단일 비볼록 문제로 통합한 multi-phase 정식화, (3) AC 핀·노즐각의 최대 slew rate 제약을 SCVX에 편입한 점이다. 모두 기존 SCVX RLV 라인의 분명한 확장이나, 구성요소는 확립된 기법의 조합이다.

**6. Limitation·Future work.** 저자: 수렴 미증명, 공력 유도 단계가 약 2s로 짧고 단계가 길어지면 절감폭 증가 여지, custom SOCP 솔버 시 추가 가속 기대. 내가 보는 한계: 모델 불확실성·외란·항법오차 미포함으로 실제 폐루프·HIL 검증 부재, 공력계수표 미공개로 재현성 저하, 외부 공개 baseline 부재, 단일 시나리오군. Future: 공력/질량 불확실성 robustness, 임베디드 실시간 구현·HIL, 폐루프(샘플링 재계획) 검증.

**평가:** 70/100 (good) · 한줄평: 확립된 SCVX·STC 도구를 견실하게 결합해 6-DOF dual aero/propulsive·다단계 RLV 착륙을 정식화하고 명확한 타이밍 근거를 제시한 유능한 in-domain G&C 논문이나, 자체 이론·외부 baseline·불확실성 검증이 얇아 영향력은 중간 수준이다.

---

## (2024, JGCD, Fast and Robust Optimization of Full Trajectory from Entry Through Powered Descent: Lu·Sandoval·Davami)

**1. 핵심 아이디어.** 고질량 화성 EDL(진입-하강-착륙) 임무의 진입~동력하강 전 구간을 하나의 하이브리드 최적제어(HOC) 문제로 보고, 추진제 소모를 최소화하는 end-to-end 궤적을 빠르고 신뢰성 있게 최적화한다. 새 솔버를 만드는 대신 기존 진입 유도(FNPEG)와 연료최적 동력하강 유도(G-POLAR), 적응형 PDI(동력하강 개시) 로직을 결합한 이중수준(bilevel) 최적화로 문제를 단일 변수(진입 종단 뱅크각 σ_f) 최적화로 환원한다. 데스크톱에서 약 10~14초 만에 거의 최적의 전 구간 궤적을 얻는다.

**2. 알고리즘.** HOC: minimize J = m0 − m(t_f), 진입 동역학 ẋ_e=f_e(x_e,σ)와 동력하강 동역학 ẋ_p=f_p(x_p,u_p), 천이는 PDI 시점 t_PDI에서 x_PDI = q(x_e(t_PDI)). 외부(상위) 최적화: golden-section search로 σ_f∈[σ_min,σ_max] 1변수 최소화. 내부(하위) 최적화: 주어진 PDI에서 G-POLAR(간접법)가 연료최적 해를 ~2초에 산출. 진입 궤적은 FNPEG 폐루프 시뮬레이션으로 생성, PDI는 적응형 PDI 로직(예측 동력하강의 지상거리가 실제 잔여 지상거리를 초과하는 순간)으로 결정. 뱅크각은 에너지 e=1/r−V²/2에 대해 선형 프로파일 |σ|=σ0+(e−e0)/(e_f−e0)(σ_f−σ0), 1차 액추에이터 σ̇=−(σ−u)/τ (τ=10s)와 σ̇ 율제한 적용. 가정 1~3 하에서 Proposition으로 bilevel 해 존재 및 golden-section 수렴을 보장.

**3. 비판적 검토.**
- 수렴/존재 보장은 ODE의 파라미터 연속의존성과 golden-section의 단봉성 가정에 의존한다. Corollary 자체가 비용함수의 단봉성을 전제하므로 전역최적이 아닌 국소최적 수렴만 보장되며, 저자도 다중 극소 가능성을 인정한다.
- "near-optimal"의 핵심 동력(저차원 σ_f 파라미터화의 선형 뱅크 프로파일, 적응형 PDI의 근최적성)은 정량적 오차 상한 증명이 아니라 정성적/경험적 논거로 정당화된다. 벤치마크 대비 ~5% 추진제 초과로 실증되긴 하나 이론적 bound는 없다.
- FNPEG·G-POLAR·적응형 PDI를 모두 기존 자기 인용 알고리즘에 의존하므로, 본 논문의 독립 기여는 "조립 프레임워크"와 수렴 분석에 집중된다. 핵심 빌딩블록의 성능이 결과를 좌우한다.
- 정규화되지 않은 해가 지표면 grazing의 비현실적 궤적을 내는 점을 솔직히 드러낸 것은 좋으나, 정규화 계수 k 및 h_min 선정은 문제 의존적이며 자동화되어 있지 않다.

**4. 시뮬/실험.** 화성 인간급 EDL용 MRV(총질량 61,838 kg) 모델. 단일 명목 사례에서 bilevel 해 m_prop=8,899 kg(약 12초), 정규화 벤치마크 8,462 kg(범용 PSOC 도구 약 2000~4700초, 1/400 시간), 비정규화 8,825 kg는 비현실적. 추진제 질량분율 14.5%로 문헌[1]의 비최적 ~28~30% 대비 약 50% 감소. EI 분산 3000 Monte Carlo: 평균 8,881 kg, σ_f* 12~29 deg, CPU 13.88s; 고충실도 Mars GRAM 대기 적용시 평균 9,067 kg, CPU 22.23s; 동력하강에 공력 포함시 평균 추진제 추가 762 kg 감소(8,129 kg, 진공 대비 −8.6%), 전 사례 지정 착륙점 수 미터 이내 착지. baseline=state-of-the-art pseudospectral OCP 도구(DIDO 계열)+knotting. 모두 (sim).

**5. Contribution(delta).** 출판년(2024) 기준, 진입과 동력하강을 동시에 푸는 end-to-end 화성 EDL 연료최적화에서 기존은 QCQP 변환(Ref[12], 2D·행성자전 무시)이나 신경망 학습용 데이터셋(Ref[13]) 수준에 머물렀다. 본 논문은 운용 유도 알고리즘(FNPEG+G-POLAR+적응형 PDI)을 폐루프 시뮬레이션으로 엮어 무한차원 HOC를 1변수 bilevel 최적화로 환원하는, 직접/간접법 어디에도 속하지 않는 비전통적 프레임을 제시한다. 새로운 대형 SW 없이 기존 온보드 유도 자산만으로 전 구간 최적화가 가능하다는 점, 범용 도구 대비 2~3 orders of magnitude 속도 향상과 신뢰성 있는 수렴을 결합한 점이 핵심 delta다.

**6. Limitation·Future work.** (저자) 전역최적 보장 없음·단봉성 가정 의존, G-POLAR 공력 미고려로 동력하강 해는 엄밀히는 연료최적 아님(폐루프로 제약만 충족), 추진-공력 연성 데이터 부재. (평가자) 정규화/h_min·k 튜닝의 수동성, σ_f 선형 프로파일의 최적성 손실에 대한 정량 bound 부재, FNPEG/G-POLAR 비공개로 재현성 제약, HIL/비행 실증 부재로 온보드 실시간성은 데스크톱 타이밍 논거에 그침. 향후 고충실도 모델의 반복 호출형 재최적화(현재조건 기반)와 비행 SW 통합이 자연스러운 확장.

**평가:** 80/100 (strong) · 한줄평: 기존 운용 유도 알고리즘을 영리하게 재조립해 화성 EDL 전 구간 연료최적화를 초 단위로 신뢰성 있게 푸는, 실용적 임팩트가 분명한 정통 core-G&C 기여; 다만 독립적 이론 신규성보다 프레임 통합과 강건한 실증에 무게가 실린다.

---

## (2024, ASR, A novel re-entry trajectory design strategy enforcing inequality and terminal constraints in height-velocity plane: Mishra·Sushnigdha)

**1. 핵심 아이디어.** 날개형 재사용발사체(RLV)의 재진입 궤적을 고도-속도(h-v) 평면에서 설계하는 방법을 제안한다. 경로 제약(가열률·동압·하중계수)과 준평형활공조건(QEGC)으로 진입 회랑을 구성하고, 회랑 내부에 선형-로그-선형(LLL) 3구간 다항식으로 기준 궤적을 만든 뒤, 메타휴리스틱 최적화(ISSR)로 단 2개의 미지 파라미터(V1, V2)만 탐색하여 종단 range-to-go 오차를 최소화한다. 피드백 선형화로 기준 비행경로각을 추종하고 bank reversal 로직으로 부호를 결정하여 실제 궤적을 생성한다.

**2. 알고리즘.** 회랑: h_min=max(h_q,h_nz,h_Q), h_max=h_QEGC (식 17-20). 기준궤적 (식 21): V1≤v≤V_tran 선형, V2≤v<V1 로그(글라이드 제약 수정형 h_s·log(ρ0 v² S_ref C_L /(2m(gr-v²)))+C), V_f≤v<V2 선형. 1차 연속조건으로 H1,C,H2가 (V1,V2)에 종속되어 미지수 2개로 축소. 목적함수 J=|S_go* - S_rg(V_f)| (식 25)를 ISSR로 최소화(탐색공간을 반복적으로 축소: R(n)=R(n-1)·exp(-R_F/n_max) 등). 추종: dE/dv+kE=0 (E=γ-γ_ref, k<0)로 피드백 선형화하여 수정 뱅크각 β_mod=arccos[((D+g sinγ)L_ref cosβ_ref)/(L(D_ref+g sinγ_ref)) + (kv/L)(D+g sinγ)(γ-γ_ref)] (식 42).

**3. 비판적 검토.**
- h-v 평면 회랑 설계와 QEGC 상한·경로제약 하한 구성은 Shen&Lu, Huifeng 등에서 이미 확립된 틀이며, 본 논문의 핵심 신규성은 "미지수 2개" LLL 파라미터화와 FPA 추종 피드백 선형화로 한정된 점진적 개선이다.
- 비교 baseline이 내부적이다. ISSR을 PSO·GWO와만 비교(자체 최적화기 우열)하고, 궤적 결과 자체를 외부 SOTA 기법(볼록최적화/GPM/기존 h-v 기법)과 정량 비교하지 않는다.
- 피드백 선형화·bank reversal은 Yu&Chen, Shen&Lu에서 도입한 것이며 추종 안정성·수렴에 대한 형식적 증명이 없다. 식 41 유도에서 중력·원심 항을 무시하는 근사가 들어간다.
- V2 지점에서 연속조건 미적용으로 뱅크각이 불연속(bump)이 발생함을 저자도 인정하며, 피드백 선형화로 완화한다고 하나 이 처리의 정량적 영향 분석은 제한적이다.
- MC가 35회로 적고 불확실성 범위가 ±5%(C_L,C_D,ρ)로 비교적 보수적이며, 큰 공력 불확실성·외란에서 한계가 있음을 저자가 명시한다.

**4. 시뮬/실험.** 104305 kg, S_ref 391.22 m² VTHL RLV. 진입 100 km→종단 25 km, 850 m/s. 기본 3 케이스 + 진입/목적지 위경도를 바꾼 3개 신규 케이스 (sim): 종단 고도오차 약 19 m, 속도오차 0(엄밀), range-to-go 오차 0.08-0.88 km(허용 ε 이내). ISSR vs PSO vs GWO 15회 독립실행 (sim): ISSR 평균 J=0.000401, 분산 4.8e-7로 PSO(0.486)·GWO(7.54) 대비 우수. 35회 몬테카를로 (sim): 고도편차 <0.4 km, range-to-go 평균오차 1.69 km(<2 km). OPAL-RT OP4510 controller + real-time simulator로 case 1 실시간 폐루프 HIL (HIL), 샘플링 0.1 s, 오프라인 결과와 일치. 외부 기법과의 정량 baseline 비교는 없음.

**5. Contribution(delta).** 출판년(2024) 기준 선행 대비 delta: (i) Wang et al.(2019)의 사전 정의 속도+미지수 1개 또는 HOP(4 미지수) 대비, LLL 3구간 다항식으로 미지수를 2개로 축소하여 회랑 내포성을 개선하고 최적화 복잡도를 낮춤. (ii) 드래그 프로파일 추종 대신 기준 비행경로각을 직접 추종하는 비교적 단순한 피드백 선형화 추종법 제시. (iii) 실시간 HIL(OPAL-RT)로 온보드 실현성 입증. 다만 h-v 회랑·QEGC·bank reversal·피드백 선형화 자체는 선행 확립 요소로, 전체적으로 기존 방향의 정련에 해당.

**6. Limitation·Future work.** (저자) 큰 공력 불확실성·강한 상태 외란에서 성능 저하; V2 지점 뱅크각 비평활; 고 L/D·완만한 진입각 차량에 주로 적합. (리뷰어) 추종 안정성의 형식적 보장 부재; MC 규모·불확실성 범위가 제한적; 외부 SOTA 궤적 기법과의 정량 비교 누락; ISSR의 전역최적성·수렴 보장 부재(메타휴리스틱 특성상 케이스별 재튜닝 필요); 3-DOF 점질량 모델로 자세동역학·실제 제어면 포화 미반영.

**평가:** 67/100 (good) · 한줄평: h-v 평면 재진입 궤적 설계를 미지수 2개로 단순화하고 실시간 HIL까지 입증한 견실한 in-domain G&C 논문이나, 신규성은 점진적이고 외부 baseline 비교가 부족하다.

---

## (2024, Acta Astro, Endoatmospheric powered descent guidance: Wang·Zhang·Li)

**1. 핵심 아이디어.** 로켓 추진체가 착륙 직전 매우 높은 받음각(bellyflop)으로 활공하며 공력 항력으로 감속한 뒤 자세를 세워 역추진 착륙하는, 대기 내(endoatmospheric) 동력하강유도(PDG) 문제를 다룬다. 기존 대기 PDG는 받음각 20deg 이하만 다뤘으나, 본 논문은 최대 90/180deg의 강한 받음각 기동에서 발생하는 고비선형 동역학을 실시간으로 처리하는 폐루프 유도법을 제안한다. 핵심은 추진제 최적 문제를 최적제어 이론으로 분석한 뒤, 수치적 난점(특이호·throttling)을 제거하도록 두 가지 정규화를 가한 "유도 지향(guidance-oriented)" 궤적생성 문제를 successive convexification으로 매 주기 푸는 것이다.

**2. 알고리즘.** 상태 x=(r_I,V_I,Q,m), 제어 u=(ω_B,T). 운동방정식: ṙ=V, V̇=g+(A+Te_I)/m, Q̇=Q⊗Ω_B/2, ṁ=-T/V_ex. 각속도를 명시적 제어로 두고 ω̇_B는 제외하는 restricted 6-DoF(시간척도 분리 가정). 최적제어 분석으로 Hamiltonian과 costate(λ_r,λ_V,λ_Q,λ_m)를 유도, 각속도 스위칭함수 S_ω, 추력 스위칭함수 S_T로부터 특이호 존재를 규명. 정규화 1: 성능지표에 J'=m(t_f)-∫k_ω ω_B^Tω_B/2 dt 추가 → ω_B = S_ω/k_ω (포화 시 ω_max S_ω/||S_ω||)로 유일 결정, 특이호 제거. 정규화 2: 추력을 3구간 piecewise-constant "zero-max-mid"(T1=0, T2=T_max, T3∈[T_min,T_max])로 매개변수화하여 bang-bang/throttling 회피 및 피드백 마진 확보. 매 주기 시작guess(직전 해/사전설계 nominal)를 기준으로 Problem 2를 convexify→trapezoidal 이산화→SOCP(ECOS)로 풀고, 비동기 궤적생성/시간기반 보간으로 명령 생성.

**3. 비판적 검토.**
- restricted 6-DoF의 정당화(회전·병진 시간척도 분리로 자세추종이 충분히 빠르다는 가정)는 정성적 논변에 머물고, 자세제어계의 1차 지연(τ_ω=0.1s, τ_T=0.05s)을 unmodeled로 두어 실제 추종오차가 유도 성능에 미치는 영향은 정량 분석되지 않았다.
- successive convexification의 수렴성은 본문에서 증명하지 않고 외부 문헌(29,30)에 의존한다. 안전임계 착륙에서 매 주기 SOCP 수렴/타당성 실패 시의 거동(본문은 "타당하지 않으면 출력 미갱신")에 대한 보장이 약하다.
- baseline이 GPOPS 최적 기준해(상한)뿐이고, 경쟁 유도법(예: MPC 기반 ref.16, indirect-direct ref.22)과의 직접 비교가 없어 상대적 우월성 입증이 제한적이다.
- 핀포인트 정밀도에는 못 미치며(바람 외란에 위치오차 잔존), 추진제도 최적 대비 63.7 vs 47.9 kg로 약 33% 손실 — 정규화가 추진 효율을 명시적으로 희생한다.

**4. 시뮬/실험.** 지구 대기, 총질량 2000kg, S_ref=1m², T_max=40kN/T_min=24kN, V_ex=2700m/s, ω_max=30deg/s. (1) 최적기준 비교(sim): GPOPS 최적해 대비 받음각 85.7deg에서 낙하속도 48m/s vs 180deg에서 145.5m/s로 공력감속 효과 확인, 유도 궤적이 zero-max-mid 추력·평활한 각속도 생성, 계산시간 주기당 일반적으로 0.03s 미만(i7-8809G, ECOS). (2) Monte Carlo 1000회: 위치±100m, 속도±20m/s, 질량±3%, 공력계수±15%, 밀도±5%, 추력±3%, 바람 0–10m/s 분산. 최종오차 downrange<20m·crossrange 수 m, 최종자세 ~1/3 deg, 연료 ~63.7kg, 비행시간 17.0–27.4s. 절제실험: 중간추력(3구간) 제거 시 최종 속도/위치 분산이 크게 증가 → 피드백 마진 효과 정량 입증. baseline=최적기준해 + with/without intermediate-thrust ablation.

**5. Contribution(delta).** 출판년(2024) 기준 선행 대기 PDG(ref.12–16)가 받음각 20deg 이하에 국한된 데 비해, 본 논문은 받음각 90/180deg의 bellyflop 영역을 다루는 첫 대기 PDG 유도 프레임워크를 제시한다. delta: (i) 각속도를 제어로 두고 ω̇_B를 배제한 restricted 6-DoF 모델, (ii) 최적제어 분석에 근거한 각속도 2차 정규화로 특이호를 원리적으로 제거(임의 S_ω에 대해 ω_B 유일 결정), (iii) 종래 burn-coast-burn 대신 zero-max-mid 3구간 추력 매개변수화로 마지막 중간추력 구간이 피드백 마진을 확보. 단순한 SOCP 재구현이 아니라 분석 기반의 새로운 정형화.

**6. Limitation·Future work.** 저자: 핀포인트 미달(주로 바람 외란), 외란 추정/궤적추종 피드백법으로 보강 필요; 자세제어 지연 unmodeled. 추가로 보는 한계: restricted 6-DoF 가정의 정량적 타당성 검증 부재, SCvx 수렴/타당성 실패에 대한 안전 보장 미흡, 추진 효율 약 33% 손실, 경쟁 유도법 대비 직접 baseline 부재, 실기/HIL 검증 없이 데스크톱 i7 타이밍에 그침.

**평가:** 78/100 (strong) · 한줄평: bellyflop형 고받음각 대기 동력하강이라는 신규·시의적 G&C 방향을 최적제어 분석에 기반한 정규화와 실시간 SCvx로 견실하게 정형화했고, 경쟁 유도법 baseline·실기 검증 부재가 약점인 강한 core-gnc 논문.

---

## (2025, AIAA SciTech, Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-Time Compound State-Triggered Constraints: Uzun·Açıkmeşe·Carson)

> 주의: 제공된 메타데이터(CJA, "Optimal feedback guidance with disturbance rejection...", Chen·Zhang·Li)와 실제 PDF 내용이 일치하지 않는다. 실제 PDF는 Uzun·Açıkmeşe·Carson의 AIAA SciTech 2025 논문(UW-ACL)이며, 본 평가는 지시대로 실제로 읽은 PDF 내용을 채점한 것이다.

**1. 핵심 아이디어.** 6자유도 동력하강유도(PDG)에서 시간·논리 사양(STL의 부분집합인 compound state-triggered constraints, STC)을 연속시간(continuous-time)에서 만족시키는 SCP 프레임워크를 제시한다. 평활(C1-smooth)하고 sound·complete한 일반화 평균 기반 강건성 척도(D-GMSR)로 논리/시간 사양을 매끄럽게 파라미터화하고, 이를 연속시간 successive convexification(CT-SCvx)과 결합해 노드 사이에서도 제약이 만족되도록 보장하며 수렴성을 유지한다. 구현 코드를 공개했다(UW-ACL/CT-cSTC).

**2. 알고리즘.** (i) STC를 D-GMSR의 conjunction/disjunction 평활 함수(가중 기하평균·멱평균)로 파라미터화: ∧ⁿ(fᵢ≥0) ⟺ Σ max(0,yᵢ)²=0 형태의 비음(nonnegative) 등식으로 표현 → 외부 벌점함수 불필요. (ii) 경로제약을 연속시간 적분 ∫₀^tf 1ᵀqc(0,gₓ)+...dt=0 로 reformulate하고 상태 y를 증강. (iii) time-dilation s(τ)=dt̃/dτ 로 자유종단시간→고정종단시간 변환. (iv) multiple-shooting으로 동역학 정확 이산화, LICQ 완화(εLICQ). (v) 비볼록 제약을 L1 정확 벌점화 후 prox-linear(수렴 보장 SCP)+적응 가중치 갱신(Algorithm 1)으로 풀이. 부분문제는 CVXPY+ECOS/MOSEK.

**3. 비판적 검토.**
- 논문의 핵심 정리(D-GMSR의 soundness·completeness [75], prox-linear 수렴성 [33], CT-SCvx [55])는 대부분 인용/수입이며, 본 논문 고유의 새 증명은 제한적이다. 즉 두 기존 도구의 결합·적용이 주된 기여.
- "robustness measure"의 robustness는 STL 사양 만족 여백을 의미하며, 외란/불확실성 강건성이 아니다. 외란 제거나 분산해석은 다루지 않는다(메타데이터가 가리키는 disturbance-rejection 논문과 혼동 주의).
- 수치검증이 단일 시나리오 1건뿐이고 Monte-Carlo·외부 baseline·통계가 전무하다. 연속시간 만족·수렴성을 정성적으로 보일 뿐 정량 성능(반복수, 해 시간, 최적성 갭) 표가 없다.
- 실시간성을 동기로 강조하나 본 논문 자체에는 솔버 실행시간·복잡도 측정이 없다. 온보드 가능성은 인용된 선행(PIPG, CT-SCvx 실시간 데모)에 의존한다.

**4. 시뮬/실험.** 6-DoF 가변추력 로켓 단일 동력하강 시나리오(sim). 초기질량 100,000kg, 건조 85,000kg, r_i=(200,200,500)m, v_i=(0,0,-50)m/s, Isp=330s, K=15 노드, 종단시간 초기추정 21s. 약 5.72s에 속도·기울기각이 임계 이하로 떨어져 추력 제약 전환, 11.07s에 LOS 제약(h₂trig=200m) 활성, 15s에 h₁trig=100m 이하로 김벌·속도·각속도·tilt·glideslope 제약 활성. 모든 조건이 노드 사이 연속시간에서 만족됨을 시연. 정량 baseline 비교·MC 없음.

**5. Contribution(delta).** 출판년(2025) 기준, STL 부분집합인 compound STC(논리 ∧/∨ + 트리거)를 SCP 안에서 **연속시간 만족** 보장으로 부과하는 일반 정식화는 선행 STC 연구([37–39])나 정수제약([40,41])에서 다뤄지지 않았다. D-GMSR(평활·sound·complete)을 PDG STC 파라미터화에 처음 적용하고 CT-SCvx와 결합해 locality/masking 문제 없이 수렴 보장과 노드간 만족을 동시 달성한 점이 선행 대비 delta. G-FOLD/SPLICE 계보의 자연스러운 일반화.

**6. Limitation·Future work.** (저자) eventually·until 등 시간연산자의 연속시간 정식화 미해결, 사양 구조를 활용한 전용 솔버 미설계. (내가 보는 한계) 검증이 단일 시나리오로 빈약하고 MC·외부 baseline·실행시간 측정 부재, 불확실성/외란 강건성 미평가. 고유 이론 기여보다 기존 도구 결합 성격이 강해 novelty가 합성적(synthetic)이다.

**평가:** 72/100 (good) · 한줄평: 코어 G&C(6-DoF 동력하강) 논문으로 이론적 토대와 코드 공개가 탄탄하고 STL/STC의 연속시간 SCP 통합이라는 유용한 일반화를 제시하나, 단일 시나리오·무 baseline·실행시간 미측정으로 실증이 얇다.

---

## (2025, IJASS, Convex Optimization Based Landing Guidance for Reusable Orbital Rockets: Lee·Jung·Lee)

**1. 핵심 아이디어.** Starship형 "reusable orbital rocket"(벨리플롭 후 급속 flip 기동)의 착륙을 대상으로, 회전 동역학을 포함한 6-DOF 다단계(4-phase) 연료최적 착륙유도 문제를 정립한다. 궤적계획(trajectory planning)과 궤적추종(trajectory tracking)의 이중 최적화 구조를 채택하여, 비행 전 한 번 SCP로 기준궤적을 산출하고 비행 중에는 MPC로 추종 명령을 생성한다. 추종 명령 형태에 따라 추력벡터를 직접 쓰는 방법 1과 Euler각+별도 자세제어기를 쓰는 방법 2의 두 유도법을 제안한다.

**2. 알고리즘.** 병진(r˙=v, v˙=F/m+g)·회전(q˙=½Ω·q, ω˙=I⁻¹(M−ω×Iω)) 6-DOF에 타원체 공력모델 A_B=−½ρS_A‖v‖C_A v_B 적용. 계획문제 P1ᵖ: maximize m(t_f), free-final-time 다단계, 비볼록 최소추력(20)·TVC(22)·glide-slope(26)·tilt(24) 제약. 추종문제 P1ᵗ: LQ tracking objective J_lqt=‖x_err(t_f)‖²_WF+∫(‖x_err‖²_WQ+‖u˙‖²_WR). SCP화: 시간 dilation σ_j로 정규화 후 1차 Taylor 선형화(42)→사다리꼴 이산화(48,49)→virtual state J_vs=w_vs‖x−x_vs‖+trust region J_tr로 인공 비실현성 완화. 추력명령 T_c=‖T_B‖, TVC δ_y,c=atan(T_Bx/T_Bz), δ_p,c=asin(−T_By/‖T_B‖). ECOS SOCP 솔버 사용.

**3. 비판적 검토.**
- 이론적 새로움보다 통합·확장 성격이 강함: 6-DOF SCP 착륙([16,17,18,26]), MPC 추종([24-27,29]), virtual state/trust region([30])이 모두 선행 존재하며, 본 논문은 [30]의 3-DOF pitch-only를 3D 6-DOF로 확장하고 이중구조를 [29]에서 6-DOF로 끌어올린 조합이다.
- SCP 수렴은 저자도 "analytical proof of strict convergence 부재"를 명시 — 수렴/최적성에 대한 자체 보장 없음. 최적성 검증은 SQP와의 수치 일치로만 간접 확인.
- 시나리오가 단일 명목궤적(고도 700→0m, 약 15s)에 한정. flip 타이밍이 연료에 미치는 영향이 핵심 동기인데, flip 타이밍 자체에 대한 정량적 trade study가 없다(자유 final-time이 알아서 결정).
- 자세제어기는 외부 3-loop([35]) 차용이며 자세 폐루프 안정성 분석은 없음 — 방법 2의 lag 보상 필요성을 본인이 언급하나 정량화 안 됨.
- Assumption 2(CM/CP 고정)는 <20s 비행이라 합리적이나, 연료 7t 소모(초기 100t 중)로 질량·관성이 크게 변하는데 I_B가 m에 따라 갱신되는지(식 8-10) 명확하지 않다.

**4. 시뮬/실험.** 6-DOF 비선형 시뮬(sim). 명목: 착륙위치오차 방법1 0.161m/방법2 0.075m, 착륙속도 0.124/0.096 m/s, 연료 6995/6920 kg. 외부 비교 — (a) SQP로 연료최적성 교차검증(SCP 6959 vs SQP 6986.8 kg, 거의 동일), (b) 선행기법 [26](재계획만, tracker 없음) 대비: [26]은 착륙속도 ~9.856 m/s로 대폭 열위(Table 4). MC 600샘플(IC/wind/aero/TVC/CM/CP 각 100, Table 5): 방법2 ‖r(t_f)‖ 평균 0.118m(SD 0.106), 방법1 0.218m(SD 0.122). 민감도분석(Table 7): wind가 FPE 최대 기여(방법1 0.320m). 추종 ~8-50ms, 계획 ~680ms(i5 desktop).

**5. Contribution(delta).** 2025 SOTA 기준 delta는 (i) flip 기동을 포함한 6-DOF 3D 다단계 연료최적 착륙문제의 정식화([30]의 pitch-only 2D 확장), (ii) 6-DOF로 끌어올린 planning+tracking 이중 최적화 구조([29]의 3-DOF 한계 극복), (iii) 추종명령 형태에 따른 두 유도법 비교 및 민감도 분석. 6-DOF SCP·MPC 자체는 신규가 아니므로 novelty는 명백하나 점진적(incremental) — 새 방향을 여는 방법이라기보다 알려진 구성요소의 도메인 특화 통합.

**6. Limitation·Future work.** 저자: throttle lag로 인한 초기 추력 오버슈트(Fig.10/11d) → 최적화에 lag 반영 필요; wind가 주 오차원 → 추가 대응기법 필요; 방법2의 자세제어 lag 보상(command shaping). 추가로 내가 보는 한계: 단일 명목궤적·단일 차량 파라미터로 외부 일반화 근거 부족, SCP 수렴 보장 부재, 하드웨어/HIL 부재, 코드·데이터 비공개(Data availability: not available), flip 타이밍 최적화 trade study 부재.

**평가:** 73/100 (good) · 한줄평: Starship형 6-DOF flip 착륙을 SCP 계획+MPC 추종 이중구조로 견고하게 정식화하고 MC·민감도·외부 baseline까지 갖춘 탄탄한 in-domain G&C 논문이나, 구성요소가 모두 선행 기반이라 기여는 점진적이고 자체 이론보장은 없다.
