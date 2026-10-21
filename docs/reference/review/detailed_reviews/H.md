# Thread H 비판 리뷰 — Control / tracking & stabilization (RLV-specific)

> RLV/launcher/reentry **전용 폐루프 제어**(추종·안정화) 논문. 루브릭 v1.3로 일관 채점. 비-Elsevier 자동확보분 18편(Elsevier 31편은 별도 수동 수집 후 추가).

---

## (2023, JGCD, Unified-Loop Structured H-Infinity Control for Aerodynamic Steering of Reusable Rockets: Sagliano et al.)

**1. 핵심 아이디어.** DLR/JAXA/CNES의 재사용 발사체 실증기 CALLISTO의 순수 공력 하강 구간(고도 약 25~5 km, 약 60 s)에 대한 자세+위치 통합 제어를 structured H-infinity(LTI 고정구조)로 설계한다. 핵심은 전통적 외측-내측(궤적-자세) 캐스케이드를 제거하고, 피치/요 채널에서 위치·속도·자세·각속도를 하나의 PID형 제어법칙으로 묶는 "unified-loop" 구조다. 고도를 스케줄링 변수로 한 게인 스케줄링과 mu-분석·misalignment 행렬·1000-케이스 6-DoF MC로 강건성을 검증한다.

**2. 알고리즘.** 롤은 K^phi(s) = (k_i/s + k_p) e_phi + k_d e_{wx} (분리 루프 유지). 피치/요는 통합 제어법칙 K^theta(s) = (k_{i,z}/s + k_{p,z}) e_D + k_{p,vd} e_{vD} + k_{p,theta} e_theta + k_{d,q} e_{wy} (적분기로 정상상태 오차 0). 합성은 T/S/KS 혼합감도 문제 min gamma = ||P_{w->z}||_inf 를 hinfstruct(Apkarian-Noll 비평활 최적화)로 PID 템플릿에 투영해 풀고, 게인은 고도에 대해 선형 보간 k(h)=k_{i-1}+(k_i-k_{i-1})(h-h_{i-1})/(h_i-h_{i-1}). 강건성은 mu(M11)<1(robust stability)와 robgain 기반 robust performance로 평가.

**3. 비판적 검토.**
- 합성 엔진(structured H-infinity, hinfstruct, 비평활 최적화)은 전적으로 기존 도구를 도입한 것이며, 새 이론·증명은 없다. 기여의 본질은 모델링(부록 A의 상세 해석적 선형화)과 루프 통합 아키텍처에 있다.
- 여러 합성점에서 gamma>1로 robust performance 기준(eq.30)을 만족하지 못해, 임의적인 1.25*gamma 완화 기준으로 평가한다. 이는 성능 보장이 형식적으로 충족되지 않음을 의미하며 저자도 인정한다.
- 외부 경쟁 기법과의 비교가 없다. baseline은 사실상 저자의 선행 캐스케이드 설계[28]이며, 통합 루프가 더 큰 안정 여유를 준다는 주장도 자기 비교에 의존한다.
- 전부 시뮬레이션(고충실도 6-DoF)이며 비행/HIL 결과는 없다. CALLISTO 실비행 검증은 향후 과제로 남는다.
- 슬로싱·휨(bending) 모드를 명시적으로 제외하고 안정 여유로 흡수 가능하다고 가정 — 실제 유연체 발사체에서는 검증이 필요하다.

**4. 시뮬/실험.** 롤·피치 각각 15개 합성점에서 LTI 합성. 선형 분석: Nichols·감도·여유를 궤적 604점에서 평가(요구치 GM>=6 dB, PM>=30 deg, DM>=50 ms 모두 충족; 피치 최소 GM~10 dB, PM~47 deg, DM~140 ms). mu-분석: LFT로 ~20여 개 공력/관성/질량/밀도 섭동 주입, robust stability mu<1 만족(롤·피치). 게인스케줄 궤적 전체: usample로 18,724개 perturbed 폐루프 안정 여유 확인. 비선형: 1000-케이스 MC(질량 20%, 관성 12.5%, 공력계수 6%, 바람 0~50 m/s 등) 모두 정규화 오차 요구 범위 내. baseline=내부 선행연구[28](self). (전부 sim)

**5. Contribution(delta).** (1) CALLISTO의 순수 공력 하강 구간에 대한 file-backed 고충실도 공력 DB와 결합한 6-DoF 해석적 선형화 — 동체/핀 기여의 비분리 결합 반영. (2) 출판년(2023) 기준 launcher용 structured H-infinity는 이미 확립(VEGA, Ares-I 등)되었으나, 자세+병진을 단일 PID형 루프로 통합해 명시적 외/내 루프 분리를 제거한 점은 "저자가 아는 한 재사용 로켓 공력 제어에 처음" 적용 — 가장 가까운 선행[28]에 대한 명확한 진전. (3) misalignment 행렬을 통한 게인스케줄 법칙의 plant/controller 불일치 강건성 검증 절차.

**6. Limitation·Future work.** 저자: gamma>1로 robust performance 미충족(C_z,theta·C_m,theta·밀도 rho가 지배적 불확실성 — RCS 병용으로 완화 제안), 바람은 동역학에 직접 침투하는 외란으로 완전 제거 불가, 슬로싱·휨 미포함, 실비행 미검증. 내가 보는 한계: 새 제어이론 기여 부재(도구 적용 중심), 외부 baseline 부재로 통합 루프의 우월성 주장이 자기 비교에 한정, 온보드 실시간성에 대한 정량적 timing/복잡도 분석 없음(저차 구조 논거만 제시).

**평가:** 72/100 (good) · 한줄평: 실제 재사용 발사체 실증기의 미탐구 공력 하강 구간에 대한 견실하고 검증이 충실한 G&C 응용·아키텍처 기여이나, 합성 이론은 도입형이고 외부 baseline·실비행이 없어 분야 형성급에는 못 미친다.

---

## (2024, JGCD, Robust Space Launcher Control with Time-Varying Objectives: Biertümpfel et al.)

**1. 핵심 아이디어.** 발사체 대기권 상승 구간은 질량·공력·중심이 급변하는 본질적으로 시변(time-varying)·유한구간(finite horizon) 문제인데, 기존 접근(LTI gain-scheduling, LPV)은 무한구간·동결점 가정으로 이 성질을 놓친다. 본 논문은 유한구간 LTV 시스템에 대한 관측기 기반 혼합감도(mixed-sensitivity) 합성을 발사체 상승 피치 제어에 적용하고, 상승 궤적을 따라 변하는 제어 목표(추종 vs 하중경감 vs 드리프트)를 시변 가중함수로 직접 인코딩한다. 물리적으로 동기부여된 투명한 가중치 설계로 튜닝 노력을 줄이고, 고정 구조의 구현 용이한 제어기를 산출한다.

**2. 알고리즘.** 기준 궤적 주위 비선형 EOM을 수치 선형화하여 유한구간 LTV 플랜트 P (식 1)를 얻음. 제어 목표를 가중 폐루프의 유도 L2[0,tf] 노름 최소화 min_K ||F_l(G,K)||_{2,[0,tf]} (식 4)로 정식화하고, 고정 구조 관측기 기반 제어기 K=[A+BF+LC, L; F, 0] (식 5)를 단방향 결합 Riccati 미분방정식 두 개로 합성(Theorem 1, 식 6·7): 관측기 RDE는 L=-ZC^T, 상태피드백 RDE는 γ 의존이라 bisection으로 최소 γ 탐색. 4-블록 혼합감도(식 20): S=(I+P_uK)^{-1}, 가중필터 W_e(감도/외란억제, 적분증강), W_u(제어노력/롤오프 ω_ro=25 rad/s), 스케일링 V_e/V_u/V_d. 스케줄링 변수로 시간 대신 비중력속도(nongravitational velocity)를 채택해 추력 프로파일 변동에 강건.

**3. 비판적 검토.**
- 합성 정리(Theorem 1)의 증명은 저자들의 선행 ACC 2022 논문[8]에 위임되어 본문에는 결과만 인용 — 본 논문 자체의 이론적 기여는 정식화·증명보다 적용·가중치 설계에 있다.
- 외부 baseline과의 정량 비교가 없음. 고전 H∞ 굽힘필터 제어기(14 states)를 정성적으로만 언급하고, 동일 시나리오에서 추종오차·하중·연산시간을 수치로 맞대결하지 않아 "더 낫다"는 주장이 정량 근거를 결여.
- 강건 보장은 형식적 worst-case가 아니라 1024 꼭짓점 + 37 풍속의 Monte Carlo로 경험적 확인 — 본 LTV 프레임의 형식적 robust 합성/검증은 별도 논문[7][15] 소관이라 본 논문 내 보장은 nominal 합성 + MC 검증 수준.
- 합성은 비풍속(no wind model in synthesis), 풍외란은 sensitivity 요구로만 다룸 — 합리적 선택이나 외란 모델과 합성의 분리가 보수성에 미치는 영향은 정량화되지 않음.

**4. 시뮬/실험.** 16.5~103 s 상승 구간, Vega급 가요성 발사체(3 굽힘모드 + tail-wags-dog + 2차 TVC 동역학)의 고충실도 비선형 Simulink 시뮬. (a) Nominal: VV05 실측 풍 + Dryden 난류필터로 가진 — 피치오차 θ_err<0.5 deg, |q̄α| 한계 대비 2배 이상 여유, 드리프트율<20 m/s, TVC<5.5 deg 포화 미달, 모든 요구 충족 (sim). (b) Robust: Table 3 불확실성(추력 ±1%, 공력계수/l_GP ±10%, 모드주파수·TVC ±5%, 지연 30 ms) 1024 꼭짓점 × 37 고유 난류 ≈ 38,888 runs(sim) — θ_err 최대 1.30 deg, 분리 시점 요구 충족, 드리프트율 최대 21.7 m/s, 하중 안전계수 최저 1.2, TVC 소비 최대 25.3 deg·s(한계 250의 1/10). 외부 baseline·real/HIL 없음. 합성 연산 ≈241 s(데스크톱).

**5. Contribution(delta).** 출판년(2024) 기준, 관측기 기반 유한구간 LTV 혼합감도 합성[8]을 발사체 대기권 상승 제어에 처음 적용하고, 상승 궤적을 따라 변하는 제어 목표를 물리 동기 시변 가중치로 직접 인코딩한 점이 delta. LPV[5]가 가진 무한구간·임시방편 스케줄링의 보수성을 우회하고, 단방향 결합 RDE로 양방향 결합 RDE(spectral radius 조건 점검 필요) 대비 수치적으로 효율적이며 고정·투명 구조(적분증강+Luenberger+롤오프증강)를 제공. 다만 핵심 합성 기계장치 자체는 선행 자가-문헌의 수입이라 novelty는 "신규 방법"보다 "신규 적용+목표 인코딩" 수준의 명확한 점진적 진보.

**6. Limitation·Future work.** 저자: 풍속 모델 미포함(외란억제로 대체), 시간 대신 비중력속도 스케줄링이 후기 단 분리·연소시간 변동에 더 적합. 내가 보는 한계: (i) 외부 baseline 정량 비교 부재로 "고전 H∞ 대비 우월" 주장이 미검증; (ii) 형식적 robust 보장 없이 MC에 의존; (iii) 단일 발사체·피치 평면(요 동일 가정)에 한정, 다축 결합·연료 슬로싱 미포함; (iv) onboard 실시간/임베디드 실증 없음(합성은 오프라인). Future: 형식 worst-case와의 결합, HIL/실비행 검증.

**평가:** 74/100 (good) · 한줄평: 발사체 상승 제어라는 핵심 G&C 문제에 유한구간 LTV 합성을 견실히 적용하고 고충실도 MC로 검증한 양질의 in-domain 논문이나, 핵심 정리의 자가-인용과 외부 baseline 부재가 상한을 누른다.

---

## (2022, TAES, Robust Multiple Model Predictive Control for Ascent Trajectory Tracking of Aerospace Vehicles: Cao et al.)

**1. 핵심 아이디어.** 광역 비행 포락선과 강한 비선형성·공력 불확실성·큰 초기 편차를 동시에 갖는 항공우주 비행체(ASV, GHAME 극초음속 모델)의 상승 궤적 추종을 위해 강인 다중모델 예측제어(RMMPC)를 제안한다. gap-metric으로 비중복 선형모델 집합을 구성하고, 각 부영역에 polytopic 불확실성 모델과 LMI 기반 강인 MPC(오프라인 설계/온라인 최적화 분리)를 두며, guardian map(GM) 기반 능동 스위칭 규칙으로 모델 간 전환 안정성을 보장한다. 큰 초기편차에 대해서는 과도 궤적(transient trajectory) 보정 모듈(TRT)로 추종 명령을 온라인 재구성해 작동기 포화를 회피한다.

**2. 알고리즘.** 종방향 6상태 비선형 모델 X=[x,h,V,γ,α,q,m]에서 출발한다. (i) 모델집합: 포락선을 Δv·Δh로 격자화→야코비 선형화→gap metric δ̃(Γi,Γj)로 부포락선 경계·공칭점 선정→이산 선형모델 Γm. (ii) 불확실성: 30% 공력 불확실성 ω를 TP 모델로 polytope [Ãm,B̃m]=Σ σmj[Ãmj,B̃mj] (Σσ=1) 표현. (iii) 추종오차 확장계 X̄=[X;e], 피드백 u(k+i)=Km X̄. Lyapunov V=X̄ᵀP X̄ 하에 강인 안정 성능조건 V(k+1)-V(k)< -X̄ᵀQ̄X̄ - uᵀR̄u를 LMI (20)으로 변환(Lemma 1), Theorem 1에서 Km=YG⁻¹로 합성하고 입력제약·종단제약을 추가한 LMI 최적화 (29)/(46)로 J=δ+χ0+ℓ 최소화. (iv) Lemma 4는 GM 안정영역 Ω(Fig.3(d): η, ζ, ωn)에 극배치하는 LMI (47)-(49) 추가. (v) Lemma 5/6 + Theorem 2: GM 기반 스위칭 규칙(제어가능구간 Φm 중첩, 상태가 3/4 Φm 초과 시 Km→Km+1 전환)으로 전영역 폐루프 Ω-강인안정. (vi) TRT: 과도궤적 ẋd=Vd cosγd 등 (65)-(67)로 (xr-xd) 오차를 k1,k2 이득으로 지수 수렴시켜 명령 재구성, 합성입력 u=um+Km(t)X̄d.

**3. 비판적 검토.**
- 이론적 골격(LMI/Lyapunov 합성)은 견고하나 핵심 도구가 대부분 인용 수입이다: gap metric(Koenings[28]), guardian map(Saydy[37], Xiao[39]), TP 모델링[34], Cuzzola-Geromel RMPC[23], 종단가중[41]. 본 논문 고유 기여는 이들을 ASV 상승추종에 통합·재배열한 부분으로, 증명도 주로 Schur 보수·대수 변환이다.
- 온라인 계산성능 주장과 표(Table II)가 다소 상충한다. 단일주기 계산시간 0.23s가 샘플링 시간 0.1s를 초과하며, 저자 스스로 LMI 수가 문제크기에 지수적으로 증가한다고 명시한다. 실시간성을 강조하지만 실제로 0.1s 주기 내 연산을 보장하지 못한다.
- 검증이 단일 비행체(GHAME)·단일 공칭궤적·단일 불확실성 수준(30%)에 국한된다. λ=0.45, 가중행렬 Qe·R̄, k1·k2 등 다수 설계변수가 trial-and-error로 선정되어 일반화·튜닝 부담이 크다.
- TRT 과도궤적 설계 (66)은 선형 1차 오차동역학을 가정해 종단시점 도달을 보장하나, 작동기 포화 회피는 정성적(Fig.10-11) 주장에 그치고 포화 회피에 대한 형식적 보장은 없다.

**4. 시뮬/실험.** 모두 (sim). GHAME 종방향 모델, 30% 공력(L,D,My) 불확실성, 포락선 V∈[400,850]m/s, h∈[9,12]km를 31×10 격자화 후 gap metric으로 30개 부포락선 도출. 추종범위 (0km,9.6km)→(11.7km,10.05km), t∈[0,20s], 종단 수평·고도 오차 ≤10m 요구, |δe|≤20°, 0.2<φ≤1.4. 1000회 Monte Carlo로 외부 baseline SL-RMPC[22]·PDL-RMPC[23] 대비: 초기편차 e1에서 RMMPC가 SL-RMPC보다 강인성·정밀도·연료소모 우수, PDL-RMPC와 추종성능 유사하나 단일주기 계산 0.23s vs 1.14s(약 5배 단축), 총 시뮬 22.05s vs 49.4s. 더 큰 초기편차 e2에서는 PDL-RMPC가 20s 내 종단 정밀도(10m) 미달, TRT 스킴만이 만족. 폐루프 극분포(Fig.12)로 GM 스위칭 유무에 따른 Ω_obj 내 극 배치 차이 확인.

**5. Contribution(delta).** 출판년(2022) 기준 delta: (a) gap-metric 기반 비중복 선형모델 자동구성 알고리즘(반복실험식 모델집합 구성 대비 중복 제거), (b) "오프라인 설계+온라인 최적화" 분리로 RMPC 온라인 LMI 부담 경감(PDL-RMPC[23] 대비 약 5배 단축), (c) 수동 스위칭[16] 대비 GM 기반 능동 스위칭으로 전환 안정성을 형식 보장, (d) 큰 초기편차에 대한 온라인 과도궤적 보정(TRT)으로 작동기 포화 완화·가용 초기영역 확장. 개별 구성요소(MMPC, RMPC, gap metric, guardian map)는 모두 선행 존재하므로 새 방향 개척이라기보다 ASV 상승추종을 위한 통합적 진전이다.

**6. Limitation·Future work.** 저자: 예측구간 확장 시 LMI 수가 지수증가하여 계산부담 급증. 내가 보는 추가 한계: (1) 단일주기 연산(0.23s)이 샘플주기(0.1s)를 초과해 실시간 주장과 모순—실제 온보드 임베디드/HIL 검증 부재. (2) 검증이 단일 모델·궤적·30% 불확실성에 한정, 통계적 처리(분산·신뢰구간)는 MC 횟수만 제시. (3) 다수 설계파라미터의 휴리스틱 선정으로 재현·이식성 저하, 코드·데이터 비공개. (4) TRT의 포화 회피는 정성적. Future: 임베디드 실시간 구현/HIL, 6자유도 측방 동역학 확장, 파라미터 자동 튜닝.

**평가:** 75/100 (strong) · 한줄평: gap-metric·guardian-map·TRT를 결합한 ASV 상승추종용 강인 MMPC의 견실한 통합 연구로 핵심 G&C 도메인에 직접 부합하나, 구성요소가 대부분 인용 수입이고 실시간성 주장이 표와 상충하며 검증이 단일 시나리오에 국한된다.

---

## (2026, JGCD, Integrated Launcher Guidance and Control via High-Order Fully Actuated System Approach: Sun et al.)

**1. 핵심 아이디어.** 궤도 투입(orbit insertion) 단계 발사체의 병진-회전 결합 동역학과 추력벡터제어(TVC)에 의한 부족구동(underactuation)을 통합 처리하기 위해, 전통적 분리형(guidance/attitude 분리) 구조 대신 고차 완전구동 시스템(High-Order Fully Actuated System, HOFAS) 프레임워크를 제안한다. 자세 동역학을 위치 추적 루프에 직접 내장하여 중간 자세 기준(reference attitude) 생성을 제거하고, 결합·캐스케이드 비선형 동역학을 단일 4차 HOFAS 모델로 표현한다. 재귀적 제어법칙으로 시스템을 디커플된 선형 부시스템들로 변환하고, 모든 부시스템에 걸쳐 매개적 고유구조 지정(parametric eigenstructure assignment)을 수행한다.

**2. 알고리즘.** 상태를 ξ=[ξ1=r_x; ξ2=[r_y;r_z]; ξ3=γ], η=[ψ;φ;ψ̇;φ̇;m]로 분할. ξ1은 추력 스로틀로 직접(2차), ξ2는 (ψ,φ)에 대한 상대차수 2를 이용해 두 번 미분하여 4차 HOFAS로 표현(ξ2^(4)=f20+f21+G12·u1·u2), ξ3(롤)는 잉여 제어자유도로 안정화. 재귀 제어법칙: u1=G1^{-1}(-f1-A1^{0~1}e1^{0~1}+ξ1,ref^(2)), u2=(G12 u1)^{-1}(-f20-f21-A2^{0~3}e2^{0~3}+ξ2,ref^(4)), u3=-f3-G3 u2-A3^{0~1}e3^{0~1}+ξ3,ref^(2). 이는 ė_i^{0~n-1}=Φ(A_i^{0~n-1})e_i^{0~n-1}의 선형 폐루프 오차동역학을 산출. 피드백이득 A는 Lemma 2를 통해 고유구조 매개변수 σ로 표현되고, 추적오차 적분 비용 J(σ)=∫Σe_iᵀW_i e_i dt를 자세각/추력/서보각/잔여질량 제약(40b–g) 하에서 제약전사(constraint transcription)+PSO/GA 혼합 휴리스틱으로 최적화.

**3. 비판적 검토.**
- 핵심 수학 기반(HOFAS 매개적 고유구조 지정, Lemma 1)은 Duan 시리즈[26–40]에서 그대로 임포트되며, 본 논문 고유의 정리는 Lemma 2 수준. 폐루프 안정성은 선형 오차동역학으로 환원되나, 불확실성/외란 하 형식적 견고성 보장은 없고 전적으로 몬테카를로에 의존한다.
- baseline이 저자들 자신의 선행 FAS 연구[44]뿐이다. 외부 SOTA(예: convex/SOCP 기반 IGC, MPC)와의 비교가 없어 "우월성" 주장이 self-baseline에 한정된다.
- 특이점(ψ,φ=±π/2 및 u1≠0) 회피가 명령 제한(command limiting)이라는 휴리스틱 제안과 "정상 운용에서 일반적으로 만족" 가정에 머문다. 과도 구간 위반에 대한 정량 분석/증명이 없다.
- 공력은 설계상 무시(Kepler 궤도 가정). 실제 대기권 상승단 전체가 아니라 IGC 단계(궤도 투입 근방)만 다루므로 적용 범위가 좁다.
- 단일 명목 시나리오 + 측정잡음 MC뿐이며, 모델 파라미터 불확실성(추력 편차, 관성 오정렬, 추력-자세 정렬오차)은 다루지 않는다.

**4. 시뮬/실험.** 중국 LM-2F 2단 파라미터(m0=1.518×10⁴ kg, F_t,max=1.0×10⁵ N, I_sp=2751.8 m/s, δ∈[-0.15,0.15] rad) 기반 (sim). 초기오차 e0=[-100,50,-100] m, ė0=[-10,-10,-5] m/s. 명목 비교: HOFAS vs FAS[44] 동일 에너지 조건. HOFAS가 위치/속도 추적에서 더 빠른 수렴·진동 감소, 서보각이 ±0.15 rad 한계 내 유지(FAS는 0.169 rad로 일시 위반, 피치 -1.111 rad 대 HOFAS -0.210 rad로 약 40% 측면오차 안정화 개선). 500회 MC(측정잡음 Δr 1σ≈3.3 m 등) 결과 최종 위치오차 ±1 m, 속도오차 ±0.4 m/s 내, 1σ: e_x 0.109 m, e_y 0.192 m, e_z 0.129 m. 제어입력 계산시간 1 cycle당 <1 ms 보고. baseline=저자 자신의 FAS[44](self-baseline).

**5. Contribution(delta).** 선행 대비 핵심 delta는 (1) 자세를 위치추적 루프에 직접 내장한 통합 4차 HOFAS 모델로 중간 자세기준 생성을 제거, (2) 캐스케이드 2차 FAS[44]를 단일 고차 형식으로 통합하여 전 자유도에 걸친 동시 고유구조 지정, (3) 물리·구동기 제약을 반영한 피드백이득 공동 최적화. 다만 출판년(2026) 기준 HOFAS 자체는 이미 정립된 프레임워크(twistors[41], 우주선 fly-around[42], 부족구동 안정화[43])이고, 본 문제(발사체 궤도투입 IGC)는 저자 선행[44]의 정련(refinement)에 해당하여 novelty는 점진적이다.

**6. Limitation·Future work.** 저자: 특이섭동(singular perturbation) 이론으로 고차 미분신호의 이중시간척도 처리 강화, 견고성 향상을 future work로 언급. 내가 보는 한계: 외부 baseline 부재, 형식적 견고성 보장 부재, 모델 불확실성 미고려, 특이점 회피의 비증명적 처리, 공력 무시·IGC 구간 한정으로 인한 적용범위 협소, quaternion 자세표현 채택 제안은 미구현.

**평가:** 71/100 (good) · 한줄평: 발사체 궤도투입 통합 G&C를 HOFAS로 깔끔히 정식화하고 측정잡음 MC까지 수행한 견실한 in-domain 논문이나, 임포트된 핵심 기계·self-baseline·점진적 novelty로 strong에는 못 미친다.

---

## (2023, Nonlinear Dynamics, RLV integrated guidance and control based on adaptive high-order sliding mode: Li et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 재진입 단계를 대상으로 유도와 자세제어를 분리하지 않고 단일 루프로 통합한 IGC(integrated guidance and control) 구조를 제안한다. 미지의 외란과 액추에이터 고장(가산 고장 + 효율 손실)이 존재해도 외란 상한값을 몰라도 되도록, 시변(time-varying) 배리어 함수 기반 적응 이득을 고차 슬라이딩 모드 제어(HOSMC)에 결합한다. 추적 오차를 초기 시점부터 규정된 영역 안에 가두고 유한시간 수렴을 보장하는 것이 핵심 주장이다.

**2. 알고리즘.** 고도/속도/측면각(s_h=h−h_ref, s_v=v−v_ref, s_β=β−β_ref)을 출력으로 잡고 입출력 선형화로 상대차수 9(=4+3+2)를 도출하여 전체 시스템을 컴팩트형 Ṡ = A + δBM + D (식 7)로 변환, 이를 n차 적분체인 (10)–(11)로 재기술. 적응 이득은 시변 배리어 함수 K(V(z),V(z0),t) = V(z)/(f(V(z0),t)−V(z)) (식 13)로 설계되어 V(z)가 경계로 갈수록 이득이 커지고 0으로 갈수록 작아져 과대추정을 방지. 피드백 제어법칙 ũ(z,t) = k1·ũ0(z) + k2·K·sign(ũ0(z)) (식 14)이며, 공칭 제어기 ũ0와 Lyapunov 함수 V는 Hong [36]/Bhat-Bernstein [35]의 알고리즘에서 가져옴. 최종 제어모멘트 M = B^{-1}(U−A) (식 23).

**3. 비판적 검토.**
- 핵심 안정성 기계(Lemma 1은 Bhat-Bernstein, 공칭 제어기·Lyapunov 함수는 Hong [36])가 외부 인용으로 import되어 있고, Proposition 1의 증명은 "Proof 1과 유사"로 생략되어 자체 기여 증명의 완결성이 다소 약하다.
- 저자 스스로 한계로 명시하듯 "검증된 공칭 제어기에 의존"하며 정확한 수렴 시간을 결정하기 어렵다. 즉 유한시간 보장은 형식적이나 실제 수렴 시간은 초기편차·튜닝 파라미터에 좌우되어 설계 가이드가 부족하다.
- 검증이 단일 섭동 시나리오에 그치고 Monte Carlo가 없어, "외란 상한 미지" 강건성 주장이 통계적으로 뒷받침되지 못한다.
- baseline이 동일 그룹의 선행 SGC 방법(Tian et al. [15])으로 self/group 비교에 가깝다. 외부 IGC 기법(backstepping, MPC 등)과의 정량 비교가 없다.
- 효율 손실 δ와 가산 고장 ΔM이 부드러운 사인 형태로만 주입되어 급격한 고장 전이 상황은 시험되지 않았다.

**4. 시뮬/실험.** X-33 파라미터(I_xx=434270 등, cl0=−0.207 등), 샘플링 1ms. 시작점 h0=80km, v0=7800m/s, γ0=−1°. 초기편차 [−500m,100m/s,1°], 가산 고장 ΔM=10^6·[0.3sin(0.1t),1+0.3cos(0.1t),0.3cos(0.1t)], 효율 손실 δ=0.9+0.1cos(0.1t). 단일 시나리오(sim), MC 없음. 제안 IGC 대 선행 SGC [15] 비교: 측면각 오차 ~10^-16(deg)로 사실상 0, 고도/속도 추적오차가 SGC 대비 작고 고차 도함수까지 수렴, 제어모멘트 ~10^5 Nm 수준의 연속 입력. 외부 baseline·정량 표 없음.

**5. Contribution(delta).** 출판년(2023) 기준, 시변 배리어 함수 적응 이득을 단일 루프 IGC의 HOSMC에 적용한 점이 델타. Laghrouche et al. [30]의 조각별 배리어 함수가 요구하는 검출 메커니즘(detection mechanism)을 시변 배리어로 제거하고, 초기 시점부터 V(z)를 규정 영역 f(V(z0),t) 안에 유지. [28]/[32]의 IGC 대비 외란 상한 지식 없이 효율 손실·고장 하에서 유한시간 수렴을 보장한다고 주장. 다만 기여의 본질은 기존 HOSMC·배리어 적응·Hong 공칭 제어기의 재조합으로 점진적 진보 성격이 강하다.

**6. Limitation·Future work.** 저자: 검증된 공칭 제어기 의존, 정확한 수렴 시간 결정 곤란 → 향후 fixed-time 제어와 결합 예정. 추가로 내가 보는 한계: MC·외부 baseline 부재로 강건성의 통계적 근거가 약하고, 측정 잡음·지연·B 행렬 가역성/제어 포화 같은 실제 구현 제약이 다뤄지지 않으며, Assumption 1(전 상태 가측)이 항법 오차를 배제해 실기 적용성에 거리가 있다.

**평가:** 67/100 (good) · 한줄평: 외란 상한 미지·액추에이터 고장 하 RLV 단일루프 IGC에 시변 배리어 적응 HOSMC를 결합한 견실한 도메인 내 G&C 논문이나, 기여가 기존 요소의 재조합에 가깝고 단일 시나리오·자체 baseline으로 검증이 얕다.

---

## (2016, IJRNC, Adaptive-gain multivariable super-twisting sliding mode control for reentry RLV with torque perturbation: Dong et al.)

**1. 핵심 아이디어.** 재진입 RLV 자세제어를 외부루프(α,β,σ)·내부루프(p,q,r)의 다중시간척도 이중루프 구조로 나누고, 각 루프에 적응이득 다변수 super-twisting(AMSTW) 슬라이딩모드 제어기를 설계한다. 불확실성·교란의 경계가 미지인 상황에서, 적응 알고리즘으로 제어이득을 과대추정 없이 자동 조절하여 채터링을 억제하고 유한시간 추종을 보장한다. 특히 가산(additive) 외란뿐 아니라 미지 경계의 곱셈(multiplicative) 토크 섭동 (1+ε)M 까지 처리하는 것을 강조한다.

**2. 알고리즘.** 외부루프: ω_c = R⁻¹(Θ̇_d − α₁ σ_Θ/‖σ_Θ‖^{1/2} + ϖ_Θ), ϖ̇_Θ = −(β₁/2) σ_Θ/‖σ_Θ‖, 적응이득 α̇₁ = ω₁√(γ₁/2) (σ_Θ>σ_T일 때, 아니면 0), β₁ = 2ε₁α₁. 내부루프: M = I₀(ω̇_d + I₀⁻¹ΩI₀ω − α₂ σ_ω/‖σ_ω‖^{1/2} + ϖ_ω), ϖ̇_ω = −(β₂/2) σ_ω/‖σ_ω‖, α̇₂ = ω₂√(γ₂/2)·sign(‖σ_ω‖−μ) (α₂>α_m), β₂ = 2ε₂α₂. ω_c는 1차 필터(τ_ω ω̇_d + ω_d = ω_c)를 통과시켜 미분 폭발을 회피. 안정성은 z=[σ/‖σ‖^{1/2}, ϖ] 좌표의 Lyapunov 함수 V = z^T P z + 적응이득 오차항으로 V̇ ≤ −η₀√V 형태를 유도해 유한시간 수렴을 증명.

**3. 비판적 검토.**
- 핵심 STW Lyapunov 기구(P 행렬 양정치, V̇ ≤ −η₀√V 구조)는 Moreno/Shtessel [35,36,25]에서 거의 그대로 도입한 것으로, 자체 기여는 다변수·이중루프로의 확장에 한정된다(C1 일부 감점).
- 이중루프 전체의 유한시간 안정성은 다중시간척도 특성에 의존한다고 하나, 그 분리(separation) 정당화를 [37]에 떠넘기고 본문 증명은 부재(Remark 5의 한 줄). 시간척도 분리가 성립하는 조건이 정량적으로 제시되지 않는다.
- R 비특이성을 "재진입에서 β≈0이므로 det(R)≈1"로 단순 가정하나, 과도구간(β_e가 −10° 이상 발생, Fig.4)에서는 이 가정이 깨질 수 있어 R⁻¹의 robustness가 불명확.
- 외부루프 적응법칙(17)은 σ_Θ가 임계값 σ_T를 넘으면 α₁이 단조 증가만 하므로(감소항 없음), σ_Θ=σ̇_Θ=0 도달 전까지 이득이 한 방향으로 누적된다. 잡음·잔류 추종오차가 σ_T를 지속 초과하면 이득 표류(drift) 위험이 있으나 분석 없음.
- 곱셈 섭동 처리를 신규성으로 내세우나, 수식상 (1+ε)는 b₂=1+ε로 흡수되어 결국 유계 미지함수 ζ=Δ_M+ḃ₂ϖ_ω로 묶이므로, 가산 외란 처리와 구조적으로 본질적 차별점이 크지 않다.

**4. 시뮬/실험.** 고충실도 6-DOF X-33 모델(질량관성 I₀, X-33 공력데이터 [40]) 단일 시나리오, 재진입 2000s. 초기 v=25,600 ft/s, Θ(0)=[0.22,−0.2,−0.5]ᵀ rad. 가산외란 Δd(크기 κ̄)와 곱셈섭동 ε=0.1[1+sin(πt/125)], 모델불확실성 ΔI=10% I₀ 주입. Case 1: κ̄=10⁶에서 α 수렴 ≈수초 내, β·σ 추종 양호, 채터링 없음. Case 2: κ̄=10² 추가, 수렴 <1s. 정량 비교는 [32,38]보다 "더 작은 제어토크로 더 빠른 추종"이라는 서술적 주장뿐, 동일 그림 내 외부 baseline 곡선 비교·MC·통계 없음. 외란/불확실성 1세트, 단일 초기조건.

**5. Contribution(delta).** 출판년(2016) 기준: (i) Shtessel·Plestan의 단입력 적응이득 STW [35,36]를 다변수(multivariable) MIMO로 확장한 첫 시도라고 주장; (ii) RLV 재진입에서 가산뿐 아니라 미지경계 곱셈 토크섭동까지 동시 처리; (iii) 외란관측기·퍼지근사 같은 보조기구 없이 미지 경계하에서 과대추정 회피. 가장 가까운 선행([35,36] univariate, [32] smooth 2차 SMC+관측기)과 비교하면 명확한 점진적 진보이나, 방법론적 토대는 기존 STW Lyapunov 이론에 직접 의존.

**6. Limitation·Future work.** 저자: 본문은 자세제어에 한정, 유도(guidance)와의 통합은 향후 과제. 내가 보는 한계: (a) 이중루프 시간척도 분리의 정량 증명 부재; (b) 외부 baseline과의 동일조건 정량비교·MC·통계 부재로 우월성 주장이 서술 수준; (c) 적응이득 단조증가법칙의 표류 위험·실측잡음 영향 미검토; (d) 실시간성/연산복잡도·구현(HIL/임베디드) 분석 없음; (e) R 비특이성 가정의 과도구간 robustness 미검증; (f) 곱셈섭동 신규성이 수식상 가산항으로 흡수되어 실질 delta가 제한적.

**평가:** 66/100 (good) · 한줄평: 적응이득 super-twisting을 다변수·이중루프 RLV 재진입에 충실히 확장한 견실한 in-domain G&C 논문이나, 기존 STW 이론 의존도가 높고 외부 baseline 정량비교·MC가 없어 핵심 기여가 점진적이다.

---

## (2021, IJRNC, Robust model predictive control for hypersonic vehicle with state-dependent input constraints and parameter uncertainty: Hu et al.)

**1. 핵심 아이디어.** 흡기식 극초음속 비행체의 피드백 선형화 모델에서는 실제 입력 제약(엘리베이터, 연료/공기비)이 가상 입력으로 변환될 때 상태 의존(state-dependent) 비선형 제약으로 바뀌는데, 본 논문은 이 문제를 정면으로 다룬다. 파라미터 불확실성을 경계 외란을 갖는 폴리토픽 LPV 오차모델로 표현하고, 상태 의존 가상 입력 제약을 다변량 다항식으로 근사한 뒤 sum-of-squares(SOS) 기법으로 볼록 행렬 SOS/LMI 조건으로 캐스팅한다. 무한지평 RMPC min-max 최적화로 가상 제어칙을 얻고, norm-bounding으로 외란이 있는 오차상태도 불변집합에 머물도록 보장한 뒤 역계산으로 실제 입력을 복원한다.

**2. 알고리즘.** 핵심 구조는 다음과 같다. 가상 입력 v0 = M0(x,p0) + N0(x,p0)u, N0 가역 가정. 가상 제어칙은 무제약 피드백 h(ze)=Hze와 보조 피드백 k(ze)=Kze의 볼록결합: Δv0 = (1-λ)Hze + λKze = Uze, λ∈[0,1]. 상태의존 제약 Δv0min/max는 1차 다항식 Δv0 = β0 + Σ βi z_ei로 최소자승 피팅. 게인 H,K는 Theorem 1의 Optimization problem 2 (min γ s.t. Q>0, Schur 보수로 얻은 LMI (35)-(40) + SOS 제약 (41)-(42))로 동시에 산출, H=YhQ^-1, K=YkQ^-1. 안정성 증명은 Lyapunov Vn=ze^T P ze, ΔV ≤ -||ze||_E^2 - ||Δv0n||_F^2를 i=0..∞ 누적하여 J∞≤γ; 외란 항은 Lemma 5 norm-bounding ((a+b)^T G(a+b) ≤ (1+δ)a^T Ga + (1+1/δ)b^T Gb)으로 (1+δ1)θ1 + (1+1/δ1)ξ ≤ 1 조건 하에 불변집합 유지. 실제 입력 u = N0^-1(x,p0)(-M0(x,p0) + (Δv0+v0m)).

**3. 비판적 검토.**
- 이론적 골격(Lemma 2 입력제약 등가변환[ref27], Lemma 3-4 SOS 불변집합/제약[ref27], Lemma 5 norm-bounding[ref35], SOS 정의[ref34])이 대부분 인용·수입된 보조정리이고, 본 논문의 기여는 이들을 극초음속 상태의존 제약 문제에 조합·적용한 것. 증명은 깔끔하나 새 기계장치는 적다.
- 상태의존 제약을 1차 다항식으로 피팅한 것이 핵심 단순화인데, Remark 1에서 저자 스스로 정확도-계산량 트레이드오프를 인정. 강한 비선형 제약을 affine으로 근사하면 제약 경계 근처에서 보수성/위반 가능성이 생길 수 있으나 이에 대한 오차 정량화가 없다.
- 외란 경계 W(||w||^2 ≤ W)와 θ1, ξ, δ1을 default parameter로 두는데, 이들 선정 절차와 보존성(feasibility 영향)이 논의되지 않는다.
- 불변집합 보장은 형식적으로 제시되나, 불확실 파라미터 50%(m, I) 변동 범위 전체에 대한 실증 검증(MC)이 없어 worst-case 보장이 단일 궤적으로만 뒷받침된다.

**4. 시뮬/실험.** AFRL 종방향 극초음속 모델(ref3,33), 단일 기준궤적: 초기 q̄=2000 lb·ft^-2, h=85,000 ft, Mach 10/220s까지 상승 후 h_ref=115,000 ft, V_ref=10,115 ft/s. 실입력 제약 -20°≤δe≤20°, 0≤Φc≤1.5. 불확실 파라미터 8종(m,I 50%, β 3% 등, Table 1). Baseline = 전통 RMPC(상태의존 제약 미고려, 자체 구현, 단일·내부 baseline). 정량결과: 두 제어기 모두 추종 성공하나 SOS-RMPC가 전환점에서 채터링 범위가 작고 더 빠른 안정화, 전통 RMPC는 가상입력이 상태의존 제약을 위반→실제입력도 위반(Fig 3,4). MC·통계·외부 baseline·실시간 타이밍은 모두 부재 (모두 (sim)).

**5. Contribution(delta).** 출판년(2021) 기준, 피드백 선형화 기반 극초음속 제어에서 가상 입력 제약이 본질적으로 상태 의존이라는 문제를 명시적으로 SOS로 처리한 점이 가장 명확한 delta. 선행 RMPC-HV(ref25 parameter-dependent RMPC, ref26 weighted tube-RMPC)는 제약을 실입력 직접 또는 간접 처리하거나 anti-windup으로 우회했고, SOS-HV(ref31)는 RMPC와 결합하지 않았다. 본 논문은 RMPC+SOS를 결합해 상태의존 가상제약을 LMI로 캐스팅하고 무한지평 robust 보장까지 묶은 점이 기여. 다만 각 구성요소는 기존재하며 영리한 조립에 가깝다(파괴적 신규성은 아님).

**6. Limitation·Future work.** 저자: 강체 모델만 다루고 탄성 모드 무시→탄성 극초음체로 확장 예정. 추가로 내가 보는 한계: (i) 1차 다항식 피팅의 근사오차가 보장에 미치는 영향 미정량, (ii) 50% 파라미터 변동에 대한 MC 부재로 robust 주장과 실증 사이 간극, (iii) 온라인 연산은 가벼우나(게인 오프라인) 명시적 타이밍/복잡도 분석 없음, (iv) 단일 시나리오·단일 내부 baseline으로 일반화·우월성 입증이 약함.

**평가:** 65/100 (good) · 한줄평: 극초음속 피드백선형화에서 간과돼온 상태의존 입력제약을 SOS-RMPC로 형식화한 견실한 in-domain G&C 논문이나, 수입 보조정리 의존과 단일궤적·MC부재의 얕은 검증이 상한을 누른다.

---

## (2025, AIAA SciTech, Incremental Nonlinear Dynamic Inversion Flight Control for the DLR Reusability Flight Experiment ReFEx: Gäßler·Robens)

**1. 핵심 아이디어.** DLR의 재사용 발사체 시험기 ReFEx의 대기권 재진입 비행 단계를 위한 비행제어 설계를 제시한다. 넓은 Mach·받음각 영역, 빠른 동압 변화, 제어면 교차결합, 제어 반전(control reversal) 및 특이점(singularity)이라는 난제를 다루기 위해, 외부 NDI 자세루프 + 내부 NDI 각속도루프의 종속(cascaded) 구조에 INDI 기반 제어배분(control allocation)을 결합하고, 특이점 부근의 과대 편향을 막는 damped-least-squares(DLS) 기법으로 보강한다.

**2. 알고리즘.** 외부루프 NDI: omega_c = G1^{-1}(phi_dot_c - G2[chi_dot; gamma_dot]), 의사제어 phi_dot_c = K_P,phi·phi_err + phi_dot_ref (FF+FB). 내부루프 NDI: M_c = J_est·omega_dot_c (cross-term 무시), omega_dot_c = K_P,omega·omega_err + omega_dot_ref. 제어배분은 INDI 1차 Taylor 전개: omega_dot ≈ omega_dot_0 + J^{-1}·B·Δδ, B = p_dyn·A_ref·L_ref·∂C/∂δ. DLS 최적화 min_Δδ(||ΔC - C_δ·Δδ||^2 + λ^2||A(δ_0+Δδ)||^2), 해석해 Δδ = (C_δ^T·C_δ + λ^2·A)^{-1}(C_δ^T·ΔC + λ^2·A·δ_0). λ^2은 최소특이값 σ_min이 임계값 ε 미만일 때 [1-(σ_min/ε)^2]·λ_max^2로 활성화되어 측방향 편향만 0으로 감쇠.

**3. 비판적 검토.**
- NDI·INDI·DLS 모두 선행 문헌(Sieberling, Acquatella, Springer Handbook of Robotics)에서 가져온 기성 도구이며, 기여는 ReFEx 특유 난제(반전·특이점)에 대한 통합·적용에 집중된다. 새로운 정리·안정성 증명은 없다.
- 강건성·특이점 통과 안정성에 대한 형식적 보장이 없고, 100회 Monte Carlo 경험적 결과로만 robustness를 주장한다. 출판 기준(TAES/TASE)에서는 약점.
- 외부 baseline 비교가 전무하다. gain-scheduling 또는 순수 NDI 대비 정량 비교가 있었다면 INDI+DLS 채택의 정당성이 훨씬 강해졌을 것이다(논문 [50] 같은 비교가 인용만 됨).
- 정량 결과가 그림의 오차 시계열로만 제시되고, 표 형태의 RMS·최대오차·실패율 통계가 없어 결과 검증이 어렵다.
- INDI가 미분된 각가속도 신호에 의존하는데(저자도 인정), 현재는 Bessel 필터만 적용. 위상지연·노이즈 영향에 대한 정량 분석이 부족.

**4. 시뮬/실험.** 고충실도 6-DoF MATLAB/Simulink 환경, 100회 Monte-Carlo(sim). 전 재진입 궤적(Mach ~5.3→0.3, 받음각 -36°→+8°, bank 변화 >180°). 섭동 대상: MCI/CoM, AEDB, 센서·구동기 동특성·장착오차, 환경·대기·바람, 초기조건, 상승궤적, 열변형. 결과: bank·받음각·옆미끄럼각 오차가 제어반전·특이점·롤기동 구간(첫 100초, 최고 동압 영역)에서 가장 크지만 유계로 유지됨을 정성적으로 제시. 외부 baseline 없음(자기검증). 실제 비행은 2026년 예정.

**5. Contribution(delta).** 출판년(2025) 기준 NDI/INDI의 RLV 재진입 적용 자체는 신규가 아니다(선행: Acquatella 2020, Simplício 2023, Juliana). delta는 (i) ReFEx의 비대칭 카나드 롤 제어반전·측방향 제어 특이점이라는 구체적 물리 난제를 식별하고, (ii) 특이값 트리거 DLS 제어배분으로 측방향 편향만 선택적 감쇠하여 포화·불안정 없이 특이점 영역을 통과하도록 통합 설계한 점. 실 비행 프로그램 대상의 엔지니어링 통합 기여로, 신규 방법론보다 적용·통합 성격이 강하다.

**6. Limitation·Future work.** 저자: INDI의 미분 각가속도 의존성을 complementary filter로 완화 예정. 추가(내가 보는 한계): 형식적 안정성/강건성 보장 부재, 외부 baseline 비교 부재, 정량 통계 미제시, 재현성 낮음(AEDB·MOPS 게인 비공개), 온보드 실시간 타이밍 벤치마크 미제시(40Hz 아키텍처는 명시).

**평가:** 61/100 (fair) · 한줄평: 실 DLR 비행 프로그램의 견실한 재진입 제어 통합 설계로 도메인 관련성은 높으나, 방법론 신규성·형식적 보장·외부 baseline·정량 통계가 약해 G&C 학술 기여로는 중위권.

---

## (2020, JGCD, Optimal Tuning of Adaptive Augmenting Controller for Launch Vehicles in Atmospheric Flight: Trotta et al.)

**1. 핵심 아이디어.** 발사체(LV) 대기권 비행용 적응증강제어(AAC; baseline PID/PD 제어기 출력에 곱셈형 forward-loop 적응 게인 k_T를 더하는 NASA SLS 계열 구조)의 파라미터를 수동 trial-and-error 대신 체계적으로 자동 튜닝하는 두 가지 최적화 방법론을 제안한다. (1) RDO/MC: Monte Carlo 캠페인으로 worst-case 성능지표 J_MC를 평가해 유전알고리즘(GA)으로 최소화. (2) RDO/min-max: aero GM=0 및 rigid GM=0의 두 limiting corner case만 시뮬레이션해 J를 정의하고 Nelder-Mead로 푸는 경량화 버전. 목적은 자세오차(‖Δθ‖∞)와 횡방향 공력하중(‖q̄α‖∞HP)을 동시에 줄여 LOV(loss-of-vehicle) 사건을 감소시키는 것이다.

**2. 알고리즘.** 적응법칙(Eq.10): k̇_a = ((k_max−k_a)/k_max)·a_AAC·e_r² − α_AAC·k_a·y_s − β_AAC·(k_T−1), 여기서 e_r=θ_INS−θ_r은 reference model 오차, y_s는 spectral damper 출력(H_LP·y_HP², Eq.11–12), 마지막 항은 leakage(k_T→1 유도). 튜닝 대상 x=[a_AAC, α_AAC, β_AAC, ω_c^HP, ω_c^LP], 경계 k_0=GM_aero, k_a_max=GM_rigid−k_0. RDO/MC(Eq.15): min_x { max_{c∈C} J_1^(c) + max_{c∈C} J_2^(c) }, |C|=100. RDO/min-max(Eq.16): 동일 형태이나 corner set C*={c_A-GM, c_R-GM} 두 케이스만 사용. J_1=‖Δθ‖∞, J_2=‖q̄α‖∞HP. 외란은 step gust(Eq.5, t_a=70s, V_a=5, V_b=−30 m/s).

**3. 비판적 검토.**
- 적응법칙 구조(Eq.10/14)와 LV 모델, BC, notch 필터 모두 선행연구(Refs.2,4,8,11–13)에서 그대로 가져온 것으로, 본 Note의 본질적 기여는 "튜닝 방법론"에 국한된다. AAC 자체나 안정성 이론에 대한 새로운 분석/증명은 없다.
- 최적화 도구(GA, Nelder-Mead)는 모두 표준 off-the-shelf이며 noisy 목적함수에 대한 형식적 수렴/최적성 보장이 제시되지 않는다. "global optimality는 random-restart로 추구"라는 서술은 휴리스틱 수준.
- 두 방법(RDO/MC vs RDO/min-max)의 결과가 "indistinguishable"(Table 2,4; Fig.4b,6)이라는 점은 min-max 경량화의 정당성을 보이지만, 동시에 정교한 MC 최적화가 실익이 거의 없음을 시사 — 기여의 핵심 셀링포인트가 약화된다.
- min-max corner case 선택(aero GM=0, rigid GM=0)이 진짜 worst case를 포착한다는 보장은 직관적 논거(Ref.8 인용)에 의존하며 형식적 증명이 없다. 비선형 적응 동역학에서 corner가 내부 worst case를 지배한다는 보장은 일반적으로 성립하지 않는다.
- 단일축(symmetric pitch/yaw) 선형 시변 모델, aerodynamic damping·TWD 무시 등 모델 단순화가 누적되어 절대 성능 수치의 신뢰도는 제한적(저자도 conservative라 인정).

**4. 시뮬/실험.** VEGA급 medium LV(liftoff 120,000 kg) 선형 시변 모델, liftoff→1단 분리(고도 60 km)까지. max-q̄(t=72s) 조건 중심. MC 1000회, 파라미터 scattering A_6/K_1/ω_BM ±30%, a_1/a_3/a_4 ±10%(Table 3), Dryden 난류 + step gust. 정량 결과(Table 4, 1000회 L∞ 평균): BC 대비 q̄α −13%(13.41→11.79 kPa/deg), Δθ −8%(4.45→4.05 deg), ż/z는 거의 불변. LOV 관점(Fig.6): RDO/min-max 적용 시 안전한계 초과 295건 중 270건 방지(≈94%). 계산비용: RDO/min-max ≈15분(Nelder-Mead, serial, i7-9700K), RDO/MC ≈16시간(GA, 병렬). Baseline은 augmentation 없는 BC(self/internal baseline)와 두 튜닝법 상호비교 — 외부 baseline 없음.

**5. Contribution(delta).** 2020년 시점에서 AAC 자체는 SLS(Refs.2–4)·VEGA(Refs.5–8)로 이미 정립된 기술이었고, 본 Note의 delta는 "정량적·재현가능한 자동 튜닝 절차의 부재"라는 선행연구의 공백(저자가 명시한 trial-and-error 문제)을 robust design optimization 프레임으로 메운 것. 특히 16h MC 최적화와 동등한 결과를 15분에 얻는 min-max corner-case 정식화가 실용적 delta. 다만 새로운 제어 이론/구조가 아닌 기존 구조의 파라미터 선정 방법론이라는 점에서 incremental.

**6. Limitation·Future work.** 저자: 단일축 모델, AAC가 bending mode frequency 감소·spectral damper 미조정 시 BC 성능을 오히려 저하/불안정화할 수 있음; AAC를 adaptive notch filter와 통합해 최소 1차 bending mode phase-stabilization 필요. 내가 보는 한계: (a) 외부 baseline(μ-synthesis, H∞ Ref.7 등) 직접 비교 부재로 AAC 튜닝의 상대 우위 정량화 불가; (b) min-max corner의 worst-case 지배성에 대한 형식적 근거 부재; (c) 실제 비행/HIL 검증 없음, 순수 시뮬; (d) 개선폭(−8~−13%)이 모델 단순화 가정 하에서 marginal하여 실 운용 이득 불확실. Future: 실 측정 풍속 프로파일·다축 결합·bending mode 적응 통합.

**평가:** 66/100 (good) · 한줄평: 잘 정립된 AAC 구조에 대한 실용적·재현가능한 자동 튜닝 방법론을 제시한 견실한 in-domain G&C Engineering Note이나, 새로운 제어/이론적 기여가 없고 외부 baseline·형식적 보장이 부족하며 개선폭이 제한적이다.

---

## (2004, JGCD, Integrated Adaptive Guidance and Control for Re-Entry Vehicles with Flight-Test Results: Schierman et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 접근/착륙 단계에서 제어면 고착(locked/jammed) 고장 발생 시 임무를 회복하기 위해, 내부루프 제어 재구성(control reconfiguration), 외부루프 유도 이득 적응(guidance-gain adaptation), 온보드 궤적 재형성(trajectory reshaping)을 하나의 통합 시스템(IAG&C)으로 결합한다. 핵심 차별점은 궤적 재형성을 온라인 최적화 대신 오프라인에서 구축한 궤적 데이터베이스를 PNN(다항 신경망)으로 인코딩해 비행 중 신속 보간(interrogation)하는 OPTG(optimum-path-to-go) 방법론이다. Boeing X-40A 동역학을 모사한 TIFS 가변안정 항공기로 실제 비행시험까지 수행했다.

**2. 알고리즘.** 내부루프: 동적역변환 omega_dot = f(omega,P)+g(P,delta)로 모멘트 상쇄, 제어할당은 Bodson의 혼합최적화 1-노름 LP min_delta(||B delta - d_des||_1 + lambda||W_delta(delta - delta_p)||_1) (rate/position saturation 회피, lambda=0.01), 명시적 모델추종 omega_m/omega_des = K_bw/(s+K_bw)에 PID 보상 + 적분기 안티와인드업 I_AW=K_AW(B delta - d_des). 축 포화 시 기준모델 대역폭 K_bw를 |B delta - d_des|에 따라 축소. 유도: 백스테핑(고도→비행경로각→받음각→피치율) GLAW, 각 루프 비례이득을 피치축 대역폭 K_bwQ의 선형함수로 적응(예: K_H=(1/15)K_bwQ). 최종 플레어: H_dot_c = -H/tau_ff + H_dot_des. OPTG: SQP로 min J=(H_ddot^T W H_ddot)^{1/2} 궤적 최적화(독립변수=다운레인지 X), C_L/C_D 상하한을 critical parameter로 변동시켜 고장 표현, 궤적군을 KG 다항/PNN으로 인코딩, 비행 중 PLP로 달성가능 C_L,C_D 추정 후 보간 + 제약최소자승 계수보정.

**3. 비판적 검토.**
- 공식 안정성/수렴성 보증이 없다. 백스테핑·동적역변환·이득적응을 채택하면서도 폐루프 안정성 정리나 적응 시스템의 수렴 증명을 제시하지 않고, "loop-shaping 고려로 설계"라는 공학적 논거에 의존한다.
- 고장 모델링이 고착(locked-in-place) 제어면에 국한되며, floating/missing/부분작동/손상 표면은 향후과제로 미룬다. 또한 고장 표현을 lift/drag 상하한 변동으로 환원하는 가정은 영리하지만 받음각이 trim 조건에 한정되어야 성립한다.
- baseline이 본질적으로 ablation(재형성 ON vs OFF)이며, 동시대 다른 재구성/재유도 방법(predictor-corrector, shooting 등)과의 정량 비교가 없다.
- 비행시험의 TIFS 모델추종 스케일링(0.3배)으로 인해 센서오차·바람이 3.3배 증폭되어, 일부 실패 사례가 알고리즘 자체보다 시험설비 한계에서 비롯됨 — 저자도 인정.

**4. 시뮬/실험.** Batch: 고충실도 6DOF X-40A 모델, 2518개 다중 표면고장 케이스. 성공 착륙률 총 59%(재형성 OFF)→95%(재형성 ON); 클래스별 예: SB+Rud(both) 63→89%, BF+Flap 57→100% (sim). 예시 케이스 착륙 침하율 -24.6→-7.2 fps (sim). 비행시험(TIFS, Niagara Falls): Phase II 40회, 21개 고장조합, Table 6 — 예 L_Flap(-16)+SB(55) 침하율 -13.4 fps(reshaping)/-17.0 fps(no reshaping)(real), 2표면 고장(L rud -4, SB +70)에서 -21.9→-7.2 fps(real). Baseline=재형성 미적용(self/ablation). Boeing이 MC·worst-on-worst 분산해석 수행(본문 미수록).

**5. Contribution(delta).** 2004년 SOTA 기준, 재진입/착륙 유도의 온보드 궤적 생성은 predictor-corrector(Refs 4,9-11)나 shooting/2점경계값(Refs 7,8)이 주류였고 온라인 최적화의 계산부하·수렴 보장이 난제였다. 본 논문의 delta는 (1) 오프라인 궤적 데이터베이스를 PNN으로 인코딩해 온라인 최적화를 회피하는 OPTG 방법론을 RLV 재형성에 적용, (2) 제어 재구성·유도 적응·궤적 재형성을 단일 통합 아키텍처로 결합, (3) 가변안정 항공기를 통한 실제 비행시험 검증 — 당시 자율 고장허용 RLV 착륙 시스템으로는 선구적인 통합·실증이다.

**6. Limitation·Future work.** 저자: 고착 고장에 국한, 단일 착륙지/시나리오 데이터베이스, Mach 의존성 무시(접근/착륙에서만 타당, TAEM/재진입엔 부적합), TIFS 스케일링 인공물. 추가로: 형식적 안정성/적응 수렴 보장 부재, 외부 baseline 부재, PNN 보간의 오차한계가 정량화되지 않음, OPTG 온라인 연산시간의 명시적 수치 미제시(실시간성은 설계 논거로만 뒷받침).

**평가:** 84/100 (strong) · 한줄평: 제어 재구성·유도 적응·온보드 궤적 재형성을 통합하고 가변안정 항공기로 실비행 검증한 선구적 RLV 고장허용 G&C 시스템 — 형식적 보증과 외부 baseline은 부족하나 실증 강도가 높다.

---

## (2016, JGCD, Structured Singular-Value Analysis of the Vega Launcher in Atmospheric Flight: Simplício et al.)

**1. 핵심 아이디어.** 유럽 경량 발사체 Vega의 대기권 비행 구간(이륙~110초, Mach 5.6) TVC 자세제어 시스템에 대해 구조적 특이값(structured singular value, μ) 기반의 독립적 강건성 검증(V&V)을 수행한다. 발사체의 불확실성(공력계수, 관성, 추력, 굽힘모드, TVC 액추에이터, 시간지연, 바람)을 체계적으로 선형분수변환(LFT)으로 모델링하고, μ를 통해 강건안정성(RS)과 강건성능(RP)을 분석한다. 핵심 주장은 μ 분석이 고전적 주파수영역 안정여유 분석을 일반화·보완하며, 무작위 샘플링(Monte-Carlo) 대비 더 신뢰성 있고 계산 효율적으로 최악(worst-case) 파라미터 조합을 찾아낸다는 것이다.

**2. 알고리즘.** 각 파라미터 불확실성을 x = x_NOM(1 + w_x δ_x), δ_x∈[-1,1] 형태의 곱셈 섭동으로 LFT 표현하고, 모든 블록을 모아 구조화 섭동 Δ_u = diag(Δ_RB, Δ_BM, Δ_TVC, Δ_τ), ‖Δ_u‖_∞≤1 로 구성한다. 폐루프를 M(s)=F_l{P(s),K(s)}로 흡수하고 e_p = F_u{M,Δ_u}d_p. 강건안정성은 μ_Δ(M11(jω))<1 ∀ω 로 판정하며, μ는 직접 계산 불가하므로 상·하한 max_Q ρ(QM11) ≤ μ ≤ inf_D σ̄(DM11 D^{-1}) 로 경계화한다. 강건성능은 가상 성능섭동 Δ_p를 추가해 ‖e'_p d'^{-1}_p‖_∞<1 조건을 μ_Δ̂(N(jω))<1 로 환원한다. μ 하한에서 불안정화 섭동벡터 Δ_u,critic를 추출해 WC 폐루프를 재구성한다.

**3. 비판적 검토.**
- 방법론적 새로움은 제한적이다. μ 분석·LFT 모델링·WC 추출은 모두 기성 이론(Doyle/Packard/Zhou, MATLAB Robust Control Toolbox)의 적용이며, 저자들 자신의 선행 연구(ref 11)와 미사일 오토파일럿 μ 분석(1992) 등 발사체 μ 적용 선례가 존재한다. 본 논문의 기여는 새 방법이 아니라 정밀한 불확실성 원자 모델링·RP 확장·생산 발사체 비행데이터 검증이다.
- frozen-time(고정시점) 국소 μ 분석으로 시변 시스템의 전역 거동을 근사한다. 저자도 이를 인정하며 LPV 기법(ref 27)을 대안으로 언급하나, 본문에서는 이산 시점 동결 분석에 머문다.
- μ는 본질적으로 보수적이며 상·하한 사이에 갭이 존재한다. 실제 WC는 하한과 상한 사이에 있어, 보고된 WC 여유(예: LF GM 2.1 dB, 지연 115 ms)가 진짜 최악인지 불확실하다(저자도 명시).
- baseline 비교가 약하다. 비교 대상은 동일 시스템에 대한 고전 Nichols/MC(10^4 샘플)로, 외부의 경쟁적 강건성 분석 기법과의 정량 비교는 없다.
- 2D 선형 섭동 모델로 roll/pitch-yaw 커플링을 배제했고(향후 과제로 명시), 일부 파라미터 상관(추력-동압)을 단순화했다.

**4. 시뮬/실험.** Vega VV05 임무(2015.6.23) 파라미터 사용. 비행 전 구간(t=5~110 s) 다수 시점에서 μ 분석 수행; t=60 s 고동압 구간을 상세 분석. 고전 분석에서 명목 RB 여유 GM≥6.4 dB·지연 131 ms 확인. μ로 도출한 WC는 LF GM 2.1 dB·지연 115 ms로 축소(sim). 10^4 MC LFT 샘플과 비교해 μ가 단발로 더 심한 불안정 조건을 식별함을 보임(sim). 6-DOF 비선형 시뮬레이터로 시간영역 검증(WC 공력하중 Mach 3 부근 안전포락선 근접, 드리프트·TVC·자세오차 진동 증가). 실제 VV05 비행데이터(real)와 명목·WC를 비교해 사전/사후 분석의 전역 정합성 확인.

**5. Contribution(delta).** 출판년(2016) SOTA 기준: μ 자체는 1980년대 정립, 발사체 적용도 선례 존재. 본 논문의 delta는 (1) Vega 대기권 비행의 강체·4굽힘모드·TVC·지연·바람을 망라한 체계적 LFT 불확실성 모델 구축, (2) RS를 넘어 최대 동압·드리프트·TVC·추종오차에 대한 RP까지 μ로 정량화하고 가중치 설계로 요구사항 트레이드오프 공간을 관리, (3) μ 하한에서 WC 섭동벡터를 추출해 6-DOF 시뮬·실제 비행데이터로 검증한 산업 규모 사전/사후 분석. 새 알고리즘이 아닌, 생산 발사체 V&V에 μ를 end-to-end로 적용·검증한 점진적 응용 기여.

**6. Limitation·Future work.** 저자: roll 및 pitch/yaw 커플링 미반영, 물리적 상관 파라미터(추력-동압)와 LFT의 전역 시간 파라미터화 미흡, μ 상·하한 갭, frozen-time 근사. 내가 보는 한계: 방법론적 신규성 부족(기성 도구 적용), 외부 경쟁 기법과의 정량 baseline 부재, Vega 특정성으로 일반화 영향이 좁음, 데이터/코드 비공개로 재현 곤란.

**평가:** 66/100 (good) · 한줄평: 생산 발사체에 μ 기반 RS/RP V&V를 정밀 LFT 모델·실제 비행데이터로 검증한 견실한 core-G&C 응용 논문이나, 방법 신규성은 제한적이고 baseline이 약하다.

---

## (2022, IEEE TSMC, Adaptive Multivariable Reentry Attitude Control of RLV With Prescribed Performance: Tian et al.)

**1. 핵심 아이디어.** RLV 재진입 자세제어에서 외란의 상한을 모른 채(unknown upper bound) 자세추종 오차를 사전 규정된(prescribed) 성능 함수 내부에 가두는 연속 적응 다변수 제어기를 제안한다. 핵심은 시간가변 비과대추정(non-overestimated) 적응이득으로, 외란 상한 정보 없이 슬라이딩 매니폴드 s를 추적하며 이득의 과대추정을 방지한다. 이는 Obeid et al.(Automatica 2018, [25])의 스칼라 적응 SMC를 지수감쇠 항 도입과 다변수 확장으로 일반화한 것이다.

**2. 알고리즘.** 섭동 이중적분기 x1_dot=x2, x2_dot=u+Δ에 대해 슬라이딩면 s=x1+x2. 제어법칙 u = -x2 - (||s|| / (ε + ||ε0||e^{-t} - ||s||))·sign(s), 여기서 ε0=ε1+||s0||, ε는 임의 양수. 적응이득 K(s,t)=||s||/(ε+||ε0||e^{-t}-||s||)는 s에 따라 자동 조정되어 s→0이면 K→0. Lyapunov V=(1/2)sᵀs+(1/2)K(s,t)²로 ||s||<ε+||ε0||e^{-t}를 증명(Step 1), V1=||x1||로 x1,x2의 규정영역 이탈 불가(Step 2). RLV 적용 시 자세오차동역학 e1_dot=e2, e2_dot=F+R·I⁻¹u+Δ'(F는 완전기지 공칭부)에 u'=-e2-F-(||s||/(ε+||ε0||e^{-t}-||s||))sign(s) 적용 후 u=I·R⁻¹u'로 환산.

**3. 비판적 검토.**
- 검증이 매우 빈약하다. 단일 시나리오(정현파 기준·정현파 외란) 1회 시뮬레이션뿐이며 Monte Carlo, 파라미터 불확실성 스윕, 외부 baseline 비교가 전무하다. 특히 [25] 대비 "개선"을 주장하면서도 [25]나 다른 적응 SMC와의 정량 비교가 없어 delta가 실증되지 않았다.
- 신규성이 점증적이다. 저자 스스로 "[25]에서 기원"한다고 밝히며, 기여는 (i) 지수감쇠 시간가변 항 추가로 검출 메커니즘(detection mechanism) 제거, (ii) 스칼라→다변수 확장이다. prescribed performance·barrier 기반 적응 SMC 자체는 출판년(2022) 시점에 이미 확립된 도구다.
- 증명은 표준 적응 SMC Lyapunov 논증으로, 자체적으로 완결되어 있으나 핵심 기계(prescribed-performance 함수, [25]의 적응구조)를 상당 부분 차용한다. 새로운 이론 장치는 지수감쇠 항이 거의 전부다.
- 외란 모델이 단순(유계·정현파)하고, 자이로 노이즈·측정지연·입력포화·미모델링 동역학에 대한 검토가 없다. 1ms 샘플링이라는 언급 외에 onboard 실시간/임베디드 분석이 없다.
- 다변수 설계의 채터링 억제 이점을 본문에서 주장하나(Remark 3) 정량 지표(예: 제어신호 에너지·채터링 메트릭) 비교가 없다.

**4. 시뮬/실험.** MATLAB/Simulink 단일 시뮬(sim), 1ms 샘플링. RLV 관성 Ixx=434270, Ixy=17880, Iyy=961220, Izz=1131541 kg·m². 외란 Δ=[0.3sin(1.5t); 1+0.3cos(1.5t); 0.3(1+sin(1.5t)+cos(1.5t))]×10⁶. 기준 Θref=[0.5sin t, 0.4cos t, 0.3sin t] rad, 초기 Θ(0)=[0.2,0.2,0.2] rad, Ω(0)=0. 파라미터 ε=0.01, ε1=0. 결과: 자세추종·오차·슬라이딩면 norm이 모두 규정영역 (24) 내부에 확실히 구속됨(Figs.1-6), 제어신호 연속성 확인. 정량 비교 baseline 없음, MC 없음.

**5. Contribution(delta).** 선행 [25](Obeid 2018, 스칼라 barrier 적응 SMC) 대비: (i) 지수감쇠 시간가변 항 도입으로 슬라이딩면이 0 근방에 진입하는 시점을 판정하는 검출 메커니즘 불필요 → 샘플링/노이즈로 인한 비현실성 회피, (ii) 과도+정상상태 성능 동시 보장(([25]는 정상상태만), (iii) 스칼라→다변수 일반화로 RLV 자세제어 적용 및 채터링 억제. 출판년(2022) SOTA 기준 novelty는 기존 적응 SMC + prescribed performance 조합의 점증적 개선으로, 새 방향 개척은 아니다.

**6. Limitation·Future work.** 저자: matched 외란만 처리, mismatched 외란으로의 확장을 future work로 명시. 내가 보는 한계: (a) 검증의 절대적 부족(단일 시나리오·무 baseline·무 MC)으로 TSMC급 실증 기준 미달, (b) 입력포화·측정노이즈·미모델링 동역학 미고려, (c) onboard 실시간 비용·복잡도 정량 분석 부재, (d) 주장된 [25] 대비 우위가 실험으로 입증되지 않음, (e) 강건성 보장이 형식적으로 제시되나 불확실성하 통계적 검증 없음.

**평가:** 66/100 (good) · 한줄평: 이론적으로 깔끔한 RLV 적응 SMC + prescribed-performance 조합이나 점증적 신규성과 단일 시나리오·무 baseline 검증으로 in-domain G&C 저널 논문의 하한 수준에 머문다.

---

## (2018, IJRNC, Reentry attitude control for a reusable launch vehicle with aeroservoelastic model using type-2 adaptive fuzzy sliding mode control: Mao et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV) 재진입 단계의 자세 추종 제어를, 구조 탄성과 강체 운동이 결합된 공력서보탄성(aeroservoelastic) 모델 위에서 설계한다. 먼저 6-DoF 기하 모델에 가정모드법(hypothetical modal method)과 Galerkin법으로 동체 종방향 굽힘 탄성(3 모드)을 결합해 공력서보탄성 모델을 세우고, 백스테핑 기반 2-루프(자세각 외부루프 / 각속도 내부루프) 구조에서 각 루프에 슬라이딩 모드 제어기를 둔다. 비선형 함수 행렬(G_Ω^{-1}F_Ω, F_w^{-1}G_w 등)을 구간 type-2 퍼지 논리 시스템(IT2-FLS)으로 적응 근사하여 추종 성능을 높이고, 가상제어에 1차 필터를 도입해 control saltation(항 폭발)을 완화한다.

**2. 알고리즘.** 제어지향 모델: Ω̇ = G_Ω Ω + F_Ω, ẇ = G_w w + F_w u (strict-feedback). 슬라이딩면 S_Ω = D_Ω(Ω−Ω_d), S_w = D_w(w−w_d). 가상제어 w̄_d = (Ĝ_Ω^{-1})Ω̇_d − (Ĝ_Ω^{-1}F_Ω) − K_{Ω1}S_Ω − K_{Ω2}S_Ω/(||S_Ω||·1+δ), 이후 K_f ẇ_d + w_d = w̄_d 필터 통과. 실제 제어 u = (F_w^{-1})ŵ_d − (F_w^{-1}G_w)w − K_{w1}S_w − K_{w2}S_w/(||S_w||·1+δ). 비선형 항은 두 IT2-FLS의 평균출력 θ^T ξ (좌/우 파라미터 θ_l,θ_r) 형태로 추정하고, 적응법칙은 θ̂̇ = λΓ^+(ξ S/2 − c θ̂), η̂̇ = τΓ^+(−ζ S Ω_d/2 − d η̂) 형태. Lyapunov 후보 W=Σ(½S² + 파라미터오차²/2λ + ½y_f²)로 Ẇ ≤ −ρW+κ를 도출, UUB(균등 유한시간 유계) 증명.

**3. 비판적 검토.**
- 핵심 구성요소(IT2-FLS 적응 근사 + SMC + 백스테핑)는 출판 시점(2018)에 이미 유연 공기흡입식 극초음속기 대상으로 존재(참고문헌 31 Tao 2017, 32 Gao 2015의 type-2 fuzzy SMC). 본 논문의 신규성은 이를 RLV 재진입 + 공력서보탄성 모델에 조합 적용한 것으로 증분적이다.
- 공력서보탄성 결합이 제어 설계에 들어가는 경로가 약하다. 탄성좌표 η_1,η_2,η_3가 공력계수의 인자로만 들어가고, 탄성 동역학(식 6)은 제어기 설계에서 직접 다뤄지지 않고 근사 비선형 항 속에 흡수된다. 즉 "aeroservoelastic 제어"라기보다 탄성을 외란/근사로 처리한 자세제어에 가깝다.
- 경계성 가정(Assumption 1~4)이 다소 강하다. G_Ω, F_w 및 그 도함수의 상·하한 존재를 가정하나, 재진입 전 구간에서 G_Ω의 비특이성(특히 |cos β| 항)이 실제로 보장되는지에 대한 검증이 없다.
- 베이스라인이 자체 구현 SMC 단일 비교로 내부 baseline이며, 외부(타 논문) 방법과의 비교나 몬테카를로가 없다. 정량 우위(각속도 피크 48/24/22 vs 82/58/61 deg/s)는 SMC 게인 튜닝에 의존할 수 있다.
- type-2 FLS + Karnik-Mendel 타입축소의 온보드 연산부담에 대한 시간/복잡도 분석이 없다. 매끄러운 제어신호는 제시하나 실시간성 근거는 부족.

**4. 시뮬/실험.** MATLAB 6-DoF, X-33 차량(I_xx=434,270 / I_yy=961,220 / I_zz=1,131,541 slug·ft², I_xz=17,880), 초기 h=260,000 ft, v=24,061 ft/s, γ=−1.064°, α=12.6°, β=−11.46°. 유도명령 Ω_d=[15°,0°,−46°], Lu(참고42) 궤적. 고정 샘플 0.01 s, 100 s 중 앞 100 step 표시. Case 1 공칭, Case 2 ±10% F_Ω 섭동 + 외란 ΔD_w(10^6 스케일 사인항). 결과: 자세각 약 3 s 내 안정 추종, 각속도 피크 Type-2 AFSMC 48/24/22 deg/s vs SMC 82/58/61 deg/s, 탄성좌표 도함수 약 10 s 내 0 수렴. baseline=자체 SMC(internal), MC 없음. (sim only)

**5. Contribution(delta).** 선행(Tao 2017, Gao 2015의 유연 극초음속기 IT2 퍼지 SMC) 대비 delta: (1) RLV 재진입 자세제어에 공력서보탄성 모델(가정모드법+Galerkin, 3모드)을 명시적으로 결합한 6-DoF 검증, (2) 2-루프 백스테핑 + 가상제어 1차 필터로 control saltation 완화, (3) 좌/우 파라미터 각각에 대한 적응법칙과 완전한 UUB Lyapunov 증명 제시. 2018년 기준 novelty는 새 방법론 개창이 아니라 기존 IT2-AFSMC 구성요소의 재진입-탄성 도메인 조합 및 통합 안정성 증명에 있음.

**6. Limitation·Future work.** 저자: 재진입 단일 단계(TAEM/오토랜딩 미포함), 공칭 곡선맞춤 공력. 내가 보는 한계: 탄성 동역학이 제어 루프에 능동적으로 들어가지 않아 진정한 공력서보탄성 제어라 보기 어려움; 단일 시나리오·자체 baseline·MC 부재로 통계적 robustness 미검증; type-2 FLS 온보드 연산비용 미분석; G_Ω 비특이성 전구간 보장 미증명; 입력포화/액추에이터 동특성(δ_a,δ_f,δ_t) 미반영.

**평가:** 68/100 (good) · 한줄평: 완전한 Lyapunov UUB 증명과 6-DoF 검증을 갖춘 견고하지만 증분적인 재진입 자세제어 논문 — 핵심 IT2-AFSMC 구성은 기존 유연 극초음속기 연구의 RLV-탄성 도메인 조합이며, 자체 SMC baseline과 MC 부재가 영향력을 제한한다.

---

## (2019, IJRNC, Multivariable supertwisting fixed-time approach for RLV re-entry attitude control: You et al.)

**1. 핵심 아이디어.** RLV 재진입 자세제어를 위해 다변수 슈퍼트위스팅(multivariable super-twisting) SMC와 고정시간(fixed-time) 수렴 이론을 결합한 제어 전략을 제안한다. 다중 시간척도(multiple-time-scale) 원리에 따라 자세각 부시스템과 자세각속도 부시스템으로 분할하고, 각 부시스템마다 고정시간 외란관측기와 제어기를 종합 설계하여 초기조건과 무관한 정착시간(settling time) 안에 가이던스 명령을 추종하도록 한다. 핵심 셀링포인트는 정착시간이 초기상태에 독립적이고 채터링이 완화된다는 점이다.

**2. 알고리즘.** 제어대상: Θ̇ = RΩ + d1, Iω̇ = -ΩIω + u + d2. Theorem 1의 일반형 ẋ1 = -k1 x1/||x1||^{1/2} - k2 x1 - k3 x1||x1||^{1/2} + (x2||x2||^2)/||x1||^{1/2} + Δ, ẋ2 = -k4 x1/||x1|| - k5 x1 - k6 x1||x1||^{1/2}을 다변수 STW 고정시간 형태로 제시. Lyapunov V = 2k4||x1|| + k5||x1||^2 + (1/2)||x2||^2 + (1/2)wᵀw (w = k1 x1/||x1||^{1/2} + k2 x1 - x2)로부터 V̇ ≤ -aV^{1/2} - bV^{3/2}를 유도, Lemma 1(Polyakov)로 Tmax = 2/a + 2/b. 자세각 부시스템: 외란관측기 ż0=Rω+z1, z1=k_{Θo1}σ1/||σ1||^{1/2}+...-z2||z2||^2/||σ1||^{1/2}, 가상제어 ωd = R^{-1}(Θ̇d - z1 - w1). 각속도 부시스템: u = I(I^{-1}ΩIω - m1 - n1) 형태로 동일 구조 반복.

**3. 비판적 검토.**
- 고정시간 안정성의 핵심 도구(Lemma 1, fixed-time 개념)는 Polyakov에서 그대로 수입되며, Theorem 2-3는 사실상 Theorem 1로의 환원 증명에 불과하다. 자체 증명은 다변수 STW의 Lyapunov 유계화에 집중된다.
- 외란에 대한 가정 ||Δ|| ≤ ξ||x1|| (원점에서 소멸하는 vanishing 외란)이 다소 제한적이다. d1, d2가 일반적으로 상수 바이어스를 포함하면 이 STW 구조의 가정과 정확히 부합하지 않을 수 있는데 이에 대한 논의가 부족하다.
- "초기조건 독립" 정착시간이 핵심 주장임에도 시뮬레이션은 단일 초기조건만 사용한다. 여러 초기상태에서의 정착시간 불변성을 실증하지 않아 핵심 기여를 직접 검증하지 못했다.
- 채터링 완화를 정성적으로만 언급하고(연속 STW 구조에서 기대되긴 함) 정량 지표(제어신호 미분, 스위칭 주파수 등)가 없다.

**4. 시뮬/실험.** X-33 모델, 단일 재진입 시나리오 (sim). 초기조건 α0=12.60°, β0=-11.46°, μ0=-28.65°, p0=q0=r0=0; 관성 Ixx=434270, Iyy=961200, Izz=1131541, Ixz=17880 slug-ft². 시변 외란 d1, d2(d2 크기 ~10^4) 인가. 결과: 자세각 정착시간 약 1.8 s, 외란추정 수렴 약 0.2 s, 각속도 추종 약 0.5 s. Baseline: Tian et al.(ref 13)의 high-order sliding mode(HSM)와 비교 — 본 기법이 더 빠르고 최대 제어토크가 더 작음을 제시. 외부 baseline 1종, MC 없음, 통계 없음.

**5. Contribution(delta).** 2019 출판년 기준, 다변수 슈퍼트위스팅(Nagesh & Edwards 2014)과 고정시간 수렴(Polyakov 2012, Zuo 2014)을 RLV 자세제어에 결합한 "multivariable super-twisting fixed-time" 형태를 처음 제시한다고 주장. 다만 저자 자신의 선행연구(You et al. 2017, refs 36-38)에서 이미 고정시간 RLV 자세제어, NTSM 고정시간, 균일강건 외란관측기+고정시간 제어기를 다뤘으므로, 실질 delta는 NTSM/스칼라 형태 대신 "다변수 STW" 구조를 채택한 점에 한정된다 — 새로운 방향을 여는 advance라기보다 점진적 조합.

**6. Limitation·Future work.** 저자: 수치 시뮬레이션 검증에 그침. 내가 보는 한계: (i) vanishing 외란 가정의 보수성, (ii) 단일 시나리오·단일 초기조건으로 "초기조건 독립" 주장을 실증 못함, (iii) MC/통계 부재, (iv) 실시간성·연산복잡도·온보드 적용성 분석 전무, (v) 채터링 완화의 정량 평가 부재, (vi) 액추에이터 포화·동특성 미고려. Future: 다중 초기조건·외란 실현에서의 정착시간 불변성 실증, HIL/실시간 타이밍, 포화 처리.

**평가:** 66/100 (good) · 한줄평: 완결된 Lyapunov 고정시간 증명을 갖춘 견실한 in-domain RLV 자세제어 논문이나, 핵심 도구가 수입형이고 저자 선행연구 대비 delta가 다변수 STW 조합으로 점진적이며 단일 시나리오 검증에 머문다.

---

## (2021, AIAA SciTech, Incremental Sliding Mode Control for Aeroelastic Launch Vehicles with Propellant Slosh: Mooij·Wang)

**1. 핵심 아이디어.** 유연(공탄성) 발사체의 피치 자세 추종과 추진제 슬로싱 억제를 단일 입력(스위블 각)으로 동시에 달성하는 과소구동(under-actuated) 제어 문제를 다룬다. 센서 기반 INDI(증분 비선형 동적 역변환)에 슬라이딩 모드 제어를 결합한 INDI-SMC를 이 문제에 적용하며, 비교군으로 LQR, 순수 INDI, FL-SMC(피드백 선형화 기반 SMC)를 둔다. 순수 INDI는 과소구동 슬로싱을 억제하지 못함을 보이고, INDI-SMC가 정합/비정합 불확실성 모두에 강건함을 주장한다.

**2. 알고리즘.** 부분 피드백 선형화 후 피치 동역학은 θ̈ = ν + ε_indi (Eq.15). 가상 입력 ν를 SMC로 설계: 슬라이딩 면 s = Cᵀe = 0, C = [1, c_θ, 0, c_1, ..., 0, c_{n_s}]ᵀ (Eq.20)를 비정합 불확실성이 Cᵀ의 영공간(null space)에 속하도록 설계(Cᵀ(A−Â)=0, Cᵀ(B−B̂)=0). 도달 법칙 ν = −k₁s − k₂|s|^ρ sign(s) − CᵀÂX + CᵀẊ_ref (Theorem 1), 0<ρ<1로 유한시간 도달+연속성 동시 확보. Lyapunov V=½s²로 V̇ ≤ −k₁s² − ηs|^{ρ+1}, 섭동 시 s ≤ (|ε|/(k₂−η))^{1/ρ}로 유한시간 수렴. 슬라이딩 면상 축소차수 동역학은 ė_r = A_rr e_r + d_r, A_rr Hurwitz 설계 시 추종오차 ultimate bounded (Theorem 2).

**3. 비판적 검토.**
- INDI-SMC 골격 자체는 Wang et al. 2019(Ref.19,20)에서 이미 제안된 것이며, 본 논문의 신규성은 (i) 비정합 불확실성용 null-space 슬라이딩 면, (ii) 과소구동계로의 확장·증명, (iii) 슬로싱 발사체 적용에 한정된 증분적 델타다.
- 증명이 외부 기법에 크게 의존한다: Lemma 1(유한시간 수렴)은 Yu et al. 2005 인용, Proposition 1은 Ref.19 Theorem 1과 "analogous" 구조. 핵심 기계장치가 import된 형태.
- Assumption 2(샘플간 변동 유계), Assumption 3(외란 ‖d‖≤γ_d‖e_r‖+ε_d) 등 주요 가정이 유도가 아닌 단정으로 도입되고, γ_d < 1/(2‖P‖₂) 같은 조건의 실제 충족 여부는 검증되지 않음.
- 플랜트가 오차동역학의 선형 상태공간이며 슬로싱을 선형 mass-spring로 모델링—저자도 인정하듯 비선형 댐핑 진자 모델과의 대조가 없어 실제 슬로싱 비선형성에 대한 강건성은 미검증.
- 모든 baseline(LQR/INDI/FL-SMC)이 자체 구현이며 외부·발표된 비교군이 없음. 단일 발사체(PacAstro), sim-only.

**4. 시뮬/실험.** 단일 발사체(2단 PacAstro, 최대 동압 t=63.5s), θ_c=2°/5° 스텝. 명목: INDI-SMC가 피치오차 적분 Σθ_err=2.6°s로 최소(LQR 3.7, FL-SMC 3.1) (sim). Monte-Carlo 500회/배치, 슬로싱 파라미터 5%·10% 균등분포 섭동: LQR은 Σε_T 최대 54.3°s, σ=11.1로 outlier 다수; INDI-SMC는 μ=15.4, σ=0.1로 강건(Table 2). 유연모드(37.3·105 rad/s 2개)+3차 엔진 동역학 추가 시에도 안정, Σθ_err INDI-SMC 6.9 vs FL-SMC 8.3(Table 3); 10% 변동 MC에서도 INDI-SMC가 상태편차·슬로싱 에너지에서 우세(Table 4). 액추에이터 피드백 루프에 30 rad/s LPF 추가로 고주파 스위블 진동 해소. baseline: 모두 내부 구현(LQR/INDI/FL-SMC).

**5. Contribution(delta).** 2021년 기준 선행 대비: INDI-SMC 프레임워크(Wang 2019)는 기존이나, 이를 (i) 비정합 불확실성을 Cᵀ 영공간으로 보내는 슬라이딩 면 설계로 강건화하고, (ii) 단일 입력 과소구동(자세+슬로싱) 문제로 확장하여 Lyapunov 설계·증명을 제시한 점이 신규. 발사체 슬로싱 억제에 INDI-SMC를 처음 적용하고 LQR/INDI/FL-SMC와 정량 비교한 응용적 기여. 출판년 SOTA 대비 명백히 새로운 제어법 개시라기보다 기존 프레임워크의 의미 있는 확장+신규 응용.

**6. Limitation·Future work.** 저자: 강건 관측기(강체+슬로싱 상태) 설계 필요, 슬로싱을 비선형 댐핑 진자로 모델링하고 mass-spring와 대조, 완전 비선형 발사체 동역학으로 확장. 추가로 내가 보는 한계: 외부 baseline 부재로 절대 성능 위치 불명확, 단일 발사체·sim-only로 일반화 근거 약함, onboard 계산복잡도/타이밍 분석이 본문에 없음(100Hz 충분성은 타 문헌 인용에 의존), 핵심 증명이 import된 보조정리에 의존하여 자체 이론 기여 폭이 제한적.

**평가:** 72/100 (good) · 한줄평: 기존 INDI-SMC를 과소구동 슬로싱 발사체로 확장하고 LQR/INDI/FL-SMC 대비 광범위 Monte-Carlo로 강건성을 정량 입증한 견실한 core-G&C 응용 논문이나, 이론 골격이 선행 프레임워크 차용이고 외부 baseline·실비행 검증이 없어 flagship에는 못 미친다.

---

## (2019, IJRNC, Legacy recovery and robust augmentation structured design for the VEGA launcher: Navarro-Tapia et al.)

**1. 핵심 아이디어.** 실제 VEGA 발사체 대기권 상승비행(TVC) 제어기를 구조화 H-무한대(structured H-inf) 합성 프레임워크로 재설계한다. 먼저 비행 입증된 VV05 미션의 고전 설계 강체 baseline 제어기를 1% 미만 오차로 "legacy recovery"하여 산업적 신뢰를 확보하고, 이어 Dryden 난류 풍 모델과 파라메트릭 불확실성(LFT)을 합성 단계에 통합하여 견실성을 체계적으로 증강한다. 폐루프 전달함수의 저/고주파 점근 해석으로 가중함수 선정을 분석적으로 안내하는 것이 방법론적 핵심이다.

**2. 알고리즘.** 강체+노즐+굴곡 동역학을 상태공간(식 15-18)으로 모델링하고, 불확실성을 x = x0 + σ_x# δ# 형태의 축소 LFT(차원 95→41)로 표현. 구조화 H-inf 최적화: min_K ||T_{e'd'}(s)||_inf = min_K max_ω σ̄(F_l(P̂(s,Δ),K)) 를 MATLAB hinfstruct(Gahinet-Apkarian 비매끄러운 비볼록 최적화)로 풀며, 고정된 저차 강체 게인 구조 K=[Kpψ Kdψ Kz Kż]^T 를 random multi-start 5회로 국소최적 완화. 가중함수는 W_ψc, W_ψINS(s) 가 민감도/보완민감도를 제한하고(||Sψ||_inf 가 GM/PM 하한을 직접 부여), W_Qα 가 하중(받음각) 제한. 견실 단계는 명목 LTI를 LFT로 치환하고 Δ=diag(Δ_LV,Δ_τ,Δ_TVC) 에 대해 합성, μ_Δ(M11(jω))<1 ∀ω 로 견실안정성(RS) 보증.

**3. 비판적 검토.**
- 신규 이론적 기여는 없다. 구조화 H-inf, μ-해석, LFT 모델링은 모두 기존 기법의 적용이며, 본 논문의 자체 기여는 가중함수 선정을 돕는 점근 해석(Table 3,4)과 산업 벤치마크에의 체계적 적용에 한정된다.
- "legacy recovery" 자체가 새 성능을 내는 것은 아니며 기존 비행 제어기를 재현하는 것이 목적이므로, 증강 단계(wind/uncertainty)에서의 정량 개선(Qα 10-13%, drift-rate 20%)이 실질적 delta다. 그 개선폭은 견고하나 극적이지는 않다.
- 합성에서 굴곡 필터 H(s)는 고정하고 강체 게인만 최적화한다. 저자도 통합 설계(rigid+bending) 여지를 인정하나, 본 논문 결과는 부분 최적화에 그친다.
- 게인 수치가 "confidential reasons"로 비공개이고 VEGACONTROL 시뮬레이터가 독점이라 외부 재현이 사실상 불가능하다.
- yaw/pitch 비결합 및 roll rate 무시 가정은 표준이지만 그 영향이 "ad hoc하게 검토된다"고만 언급되어 정량적 뒷받침이 약하다.

**4. 시뮬/실험.** 실제 VEGA VV05 미션 데이터 기반. 대기권 전체에 걸쳐 t=5~90s에서 9개 구조화 H-inf 제어기를 합성·스케줄링하고 고충실도 비선형 시뮬레이터 VEGACONTROL로 검증(sim). Baseline은 실제 비행한 VV05 제어기(강력한 산업 외부 baseline). μ-해석은 명목+1000 random LFT 샘플로 RS 확인, 분산조건 안정여유(LF-GM≥0.5dB, DM≥40ms 등 Table 2) 전부 충족. 최종 견실 설계는 4개 MC 캠페인×500런(실측 French Guiana 풍 4종) 수행, Fig.19에 2000샘플 표시. 정량: 구조화 설계가 baseline 대비 Qα 피크 10%↓(severe-wind 설계는 Mach 3 근처 13%↓), 자세오차 에너지 양축 10%↓, drift-rate ∞-/2-norm 양축 20%↓, 평균 TVC 편향 5%↓.

**5. Contribution(delta).** 출판년(2019) 기준 선행(Simplicio 2016 μ-해석, Saussié 2013/Knoblauch 2012/Ganet-Schoeller 2015-17 구조화 H-inf 발사체, 본 저자들의 2016/2017 conference 회복) 대비 delta: (1) ψ̇_e를 INS에서 직접 추출하는 새 폐루프 상호연결로 의사미분필터 H2(s) 제거 및 제어기 크기 축소, 각 강체 게인에 독립 채널 부여; (2) 폐루프 전달함수의 점근 해석으로 가중함수 선정을 역공학이 아닌 물리 기반으로 체계화; (3) Dryden 풍 모델과 불확실 LFT를 합성에 통합하여 풍 외란 제거를 정량 개선. 새 방법이라기보다 비행 입증 산업 벤치마크에의 성숙한 체계적 적용·검증이 핵심 기여다.

**6. Limitation·Future work.** 저자: 굴곡 필터를 합성에 미포함(통합 rigid+bending 설계 여지, ref.24), 가중함수를 설계점별로 더 세밀화 가능, 작동기 rate/각가속도 제약 미반영. 내가 보는 한계: 이론적 신규성 부재로 IJRNC 같은 제어이론지 기준에서는 응용 논문 성격이 강함; 게인/시뮬레이터 비공개로 재현성 낮음; 부분(강체) 최적화에 머문 점; 가정의 정량적 검증 부족.

**평가:** 76/100 (strong) · 한줄평: 실제 VEGA 비행 제어기에 구조화 H-inf와 μ-기반 견실해석을 체계적·산업적으로 적용·검증한 견실한 발사체 GNC 응용 연구로, 이론적 신규성은 제한적이나 도메인 적합성과 검증 충실도가 높다.

---

## (2003, JGCD, Limited Authority Adaptive Flight Control for Reusable Launch Vehicles: Johnson·Calise)

**1. 핵심 아이디어.** 액추에이터 변위/속도 제한, 선형 입력 동역학, 시간지연, 양자화(bang-zero-bang) 등 입력 특성이 존재할 때 신경망(NN) 적응 제어가 잘못 적응하는 문제를 다룬다. 핵심은 Pseudocontrol Hedging(PCH): 기준모델(reference model) 동역학을 명령 pseudocontrol과 실제 달성 pseudocontrol의 차이만큼 "헤지"하여, 적응 법칙이 액추에이터 특성을 보지 못하도록 만든다. 이로써 입력이 포화된 구간에도 올바른 적응이 유지되고, 권한 회복 시 빠르게 복귀한다.

**2. 알고리즘.** 근사 동적역변환 delta_cmd = f̂^{-1}(x,ẋ,nu). 추정 액추에이터 위치 deltâ로 헤지 신호 nu_h = f̂(x,ẋ,delta_cmd) - f̂(x,ẋ,deltâ) = nu - nû 계산. 기준모델: ẍ_rm = nu_crm - nu_h (PCH 적용). 총 pseudocontrol nu = nu_crm + nu_pd - nu_ad + nu_r, 여기서 nu_pd = K_p(x_rm - x) + K_d(ẋ_rm - ẋ). 추적오차 동역학 ė = Ae + B[nu_ad - Delta], A=[[0,I],[-K_p,-K_d]]. 단일 은닉층 NN nu_ad = W^T sigma(V^T x̄)로 모델오차 Delta 근사. 가중치 적응: Ẇ = -[(sigma - sigma'V^T x̄)r^T + kappa||e||W]Gamma_w, V̇ = -Gamma_v[x̄ r^T W^T sigma' + kappa||e||V], r^T = e^T PB, e-modification 포함.

**3. 비판적 검토.**
- 정리는 가중치/기준모델 추적오차의 UUB(uniformly ultimately bounded)만 보장하며, 점근 안정/추적은 아니다. NN 보편근사·표준 적응제어 기법을 다수 차용하여 C1이 완전 자체증명은 아니다.
- Assumption 5(접촉 사상 nu_ad↦Delta가 contraction, ||∂Delta/∂nu_ad||<1)은 다소 강하고 SISO 외에는 검증이 어렵다. 저자도 이를 인정한다.
- 액추에이터 위치 추정 deltâ의 오차가 다시 모델오차로 나타나 NN이 보정한다고 하나, 추정 품질이 나쁠 때의 성능 저하에 대한 정량 분석이 없다.
- 검증이 X-33 MAVERIC 단일 플랫폼에 국한되고, Monte Carlo·외부 baseline·통계 처리가 없다. 비교는 "PCH 없으면 회복 불가"라는 정성적 대조에 그친다.

**4. 시뮬/실험.** X-33 RLV의 MAVERIC 고충실도 시뮬(sim), Mach~8, 고도~180,000 ft. 상승(ascent): 6개 공력면+4개 throttle, 명목 비행 자세오차 ~±0.5 deg 수준; 60 s에서 다중 액추에이터 hard-over(러더/엘레본/플랩 전개) 실패 시 자세오차가 ±100 deg까지 벗어나도 권한 회복 후 수렴(PCH 없으면 명목 게인 스케줄·적응 모두 복구 불가 departure). 재진입(entry): alpha/bank 명령 추종, RCS 양자화 bang-bang 제어, 60 s 완전 RCS 실패 케이스에서도 안정 유지. NN 출력이 roll 축 모델오차(~0.2~1 rad/s^2)를 추종함을 제시. 외부 baseline·MC 없음.

**5. Contribution(delta).** 2003년 기준, 입력 포화에 대한 기존 접근(명령 수정/AWBT/포화 시 적응 정지)과 달리, 액추에이터 특성을 기준모델에 흡수시켜 적응 법칙에서 분리하는 PCH를 새 정식화로 제시. 포화뿐 아니라 속도제한·선형 입력 동역학·시간지연·양자화/bang-bang을 단일 프레임으로 통합 처리한다는 점이 핵심 novelty이며, AWBT(Hanus 등)·증강오차 신호 접근과 구별된다. 게인 스케줄·차량 공력 지식 없이 동작하고, 입력 포화 중에도 올바른 적응을 가능케 해 실패 후 회복과 비행 중 적응 검증(flight certification)을 지원하는 길을 연다.

**6. Limitation·Future work.** 저자: 더 일반적 입력 동역학에 대한 강건성, 비행 인증 절차로의 확장이 필요. 본인 평가: (i) UUB 한계로 추적 성능 보장이 약함, (ii) contraction 가정의 실무 검증성, (iii) 액추에이터 위치 추정 의존성과 그 오차 영향 미정량화, (iv) MC·외부 baseline·통계 부재로 강건성 주장이 단일 시뮬에 의존. 향후 다중 차량/MC 검증과 추정 강건성 정량화가 요구된다.

**평가:** 82/100 (strong) · 한줄평: PCH라는 적응 비행제어의 표준 도구를 정식화하고 Lyapunov 보장과 X-33 실패 시나리오로 뒷받침한, 영향력 큰 core-G&C 논문.

---

## (2021, JGCD, Tracking Filter Integration in the Adaptive Augmenting Controller of a Launch Vehicle: Trotta et al.)

**1. 핵심 아이디어.** 발사체 대기권 비행 자세제어에서, NASA SLS 계열의 적응증강제어(AAC)는 곱셈형 게인 적응으로 강건성을 높이지만 1차 굽힘모드 주파수가 공칭에서 크게 변하면 노치필터가 무력화되어 성능이 저하되거나 불안정해진다. 본 노트는 RLS 기반 적응 추적필터로 1차 굽힘모드 주파수를 온라인 추정하여 노치·저역통과 필터를 실시간으로 재튜닝하고 이를 AAC 게인 적응층과 결합한 확장 아키텍처(EAAC)를 제안한다. VEGA급 발사체 모델로 굽힘 주파수가 강건성 한계를 벗어나는 상황에서 안정성과 구조하중(Qα) 저감을 동시에 달성함을 보인다.

**2. 알고리즘.** 기저제어기 BC는 피치/요축 PD + 노치(3중 캐스케이드 2차) + 저역통과 필터. AAC 게인법칙: k_Ṫ = p_hi(k_T)·a_AAC·e_r^2 − p_lo(k_T)·α_AAC·y_s − β_AAC(k_T−1) (오차제곱 비례 증가, 스펙트럴 댐퍼 감소, 누설항). 적응 노치는 z영역 2차 필터 H_AN(z⁻¹)=(1+2az⁻¹+z⁻²)/(1+a(1+γ)z⁻¹+γz⁻²)로 두고, RLS로 a를 추정: a(n)=−η(n)/(2Φ(n)), η·Φ는 재귀적 상관식 (식 11–13). 강체 동역학 성분은 고역통과 H_HPAN로 제거, a는 평활화 a_actual(n)=a(n)k+(1−k)a(n−1). 추정 주파수 ω_id=cos(−a(n))⁻¹/Δt로 노치 중심 및 H_X 극(ω_p=1.3ω_id)을 갱신. Δt=0.01s, RK4 적분.

**3. 비판적 검토.**
- 시변 적응 비선형 루프에 대해 frozen-time 선형 Nichols 마진 + 시뮬레이션으로만 안정성을 평가한다. EAAC 전체 폐루프에 대한 형식적(Lyapunov 등) 안정성·수렴 보장은 없다.
- 추정기 자체의 한계를 저자도 인정: 50–70s 구간에서 ω_id 추정이 공칭 대비 최대 40% 편향. AAC의 비선형 게인 적응이 고주파 강체모드를 여기시켜 추정 입력 스펙트럼을 오염시키는, AAC↔추정기 상호간섭이 본질적 약점.
- 핵심 기법(RLS [2], AAC 게인법칙 [6,8], 노치/추적필터 [19,20])이 모두 외부 인용으로, 본 노트의 기여는 이들의 통합에 한정된다.
- baseline이 모두 자체 변형(BC, BC-AN, BC+AAC vs EAAC) 즉 ablation이며 외부 경쟁기법과의 비교가 없다. 단일 발사체 클래스(VEGA)에 국한.

**4. 시뮬/실험.** VEGA급 선형 시변 모델, 발사~60–140s 대기권 구간, Dryden 난류(피크 25 m/s, severe). 특정 테스트(test1: 굽힘주파수 −35%·모달게인 +6 dB → BC-AN은 LoC, EAAC만 안정·제어노력 적분 52.20→38.67 deg·s로 저감; test2: A6 +35%, K1 −35%, ω_BM −45%). MC1·MC2 각 1000회(스캐터링 범위 표로 명시). L∞-norm 평균/표준편차 표: MC1에서 Qα 안전한계 초과 횟수 BC 324, BC-AN 329, BC+AAC 119, EAAC 144; EAAC가 Δθ·z·제어노력에서 균형. 모든 결과 (sim).

**5. Contribution(delta).** 선행 AAC([6,8])는 고정 노치를 가정해 굽힘주파수 대변동에 취약했고, 적응 노치 추적([19,20])은 AAC 게인적응과 분리되어 연구되었다. 본 노트의 delta는 RLS 굽힘주파수 추정 기반 노치 재튜닝을 AAC 게인적응층과 단일 아키텍처(EAAC)로 결합하여, BC 안정성이 marginal하고 굽힘주파수가 크게 떨어지는(phase-stabilization이 필요한) 영역에서 안정화와 구조하중 저감을 동시 달성한 점. 2021년 기준 두 확립된 기법의 비자명한 통합으로 점진적~중간 수준의 진전.

**6. Limitation·Future work.** 저자: 특정 구간 추정 편향(AAC 비선형성↔추정기 간섭), 신뢰성이 중요한 실제 적용에는 능동 필터링과 AAC의 상호작용에 대한 더 포괄적 분석 필요, 위상지연 특성을 유지하는 다른 노치 토폴로지로 개선 여지. 내가 보는 한계: 형식적 안정성 보장 부재, 외부 baseline·다기종 검증 부재, 온보드 실시간성에 대한 명시적 타이밍/복잡도 분석 및 임베디드 데모 부재(설계상 실시간 가능하나 미입증).

**평가:** 63/100 (fair) · 한줄평: VEGA AAC에 적응 노치 추적을 통합해 굽힘주파수 대변동 시 안정성·하중을 개선한 견실한 in-domain 엔지니어링 노트이나, 형식적 보장·외부 baseline 없이 ablation+MC 검증에 머문 점진적 통합 기여.

---

## (2021, Acta Astronautica, Launcher flight control design using robust wind disturbance observation: Simplício et al.)

**1. 핵심 아이디어.** 재사용 발사체의 능동 하중 경감(load relief)을 위해, 기존 피드백 전용 제어 구조에 풍속 외란을 추정하는 풍속 외란 관측기(rWDO, robust Wind Disturbance Observer)를 증강하는 비행 제어 설계법을 제안한다. 관측기 출력을 명목 안정성/추종 루프 밖의 정적 게인 K_LR로 주입하여, 명목 안정성·추종 특성을 훼손하지 않으면서 풍하중 거부 성능만 별도로 조정할 수 있게 한다. DLR EAGLE VTVL 실증기(3DoF)와 일반형 6DoF RLV 벤치마크에 적용하고, 상승 단계에서 핀(fin)을 사용해 TVC 측력을 보상하는(통상 비사용) 추가 기여를 제시한다.

**2. 알고리즘.** 플랜트는 LFT 불확실성 블록 Δ_RLV를 갖는 선형 모델 G_RLV(s)로, 풍은 Dryden 필터 G_wind(s)=3.54/(s+0.32)로 모델링. 제어 입력 u=K_FB·[θ_e, θ̇_e, ż] + K_LR·v̂_w. 관측기 L_w(s)는 운동방정식(피치/횡방향 드리프트 동역학) 역변환으로 해석적 초기해 L_w^0(s)(Eq.19)를 얻은 뒤 reshaping. rWDO 설계 문제: min_{L_w} γ=||M(s)||_∞ (Eq.21), 일반화 플랜트 M(s)에 W_u, W_n, W_e 가중. 강건성은 SSV(μ) 해석으로 평가(μ<1 if&only if 전 Δ_RLV에 대해 RS). H∞ 설계(γ=0.43)는 추력 불확실성 δT·cosβ_TVC로 인해 μ>1(RS 미달) → D-K iteration으로 재설계(γ^DK=0.58, μ<1 전 주파수 달성). K_FB, K_LR은 structured H∞로 동시 튜닝.

**3. 비판적 검토.**
- 핵심 이론 기계(H∞, μ/D-K, SSV, LFT)는 모두 인용·임포트(MATLAB Robust Toolbox, Apkarian structured H∞)이며, 자체 정리/증명은 없다. 관측기 해석해 도출은 깔끔하나 엔지니어링 수준의 엄밀성에 머문다.
- baseline이 전부 내부(self): FB-only vs WDO, 저자들의 선행 VEGA 연구. LiDAR 전방예측·적응제어·AoA 피드백 등 경쟁 LR 기법과의 정량 비교가 없어 상대적 우위 입증이 약하다.
- 분류 시뮬(EAGLE)의 일부 WDO는 수동 튜닝(σ_d=30 선택)으로 "sub-optimal일 수 있다"고 인정. 핀 상승 사용의 실제 비행 적합성(공력 가열·구조)은 다루지 않음.
- 굽힘(bending)·슬로싱·롤 커플링·센서/액추에이터 비선형을 단순화 명분으로 배제 — 실 발사체 적용 시 가장 까다로운 모드들이며 강건성 마진(μ peak ~0.53)이 이를 커버한다는 주장은 검증되지 않음.

**4. 시뮬/실험.** EAGLE 3DoF: 30 km/h(=v_w 30km/h) 돌풍에 2^5 코너케이스 MC, 최대 횡드리프트율 1.28→0.65 m/s로 감소, 외란력 ~13.5 N에 대해 관측오차 ~2 N(15%)로 수렴 (sim). 6DoF RLV: 2^10 LFT 코너 + 100 white-noise seed, 3종 테스트(0.5deg step, 저주파 고진폭 돌풍 peak 33 m/s, 고주파 저진폭). 고주파 돌풍에서 하중 분산 ~50% 경감, Q_α(동압×AoA) 100런 MC에서 >50% 감소(상승 [30,70]s LR 활성). 주파수영역: Nichols GM/PM, μ-bound 비교. tethered 비행시험은 성공적으로 수행했으나 본 논문 미수록. 외부 baseline 없음, 전부 (sim).

**5. Contribution(delta).** 선행(저자들의 VEGA SSV 연구 [1], 항공/풍력의 외란관측기 LR [14-19]) 대비: (i) 외란관측기 기반 LR을 발사체에 강건합성(H∞·D-K)으로 적용, (ii) K_LR을 피드백 루프 밖에 배치해 명목 안정성·추종과 LR 조정을 분리하는 구조, (iii) 상승 단계 핀 사용으로 TVC 측력 보상(저자 표현으로 발사체 문헌에서 novel). 2021년 SOTA 기준 — 개념(disturbance-observer LR) 자체는 항공/풍력에서 기성립이므로 점진적 진보이나, 발사체로의 강건합성 이식 + 핀 상승사용 + 실증기 적용은 명확한 응용적 delta.

**6. Limitation·Future work.** 저자: 관측기 수동튜닝의 sub-optimality, 추력 불확실성이 RS 저하 주원인, Q_α와 추종오차 간 균형 튜닝 필요. 추가로 보는 한계: 외부 baseline 부재로 경쟁기법 대비 정량우위 미입증, 굽힘/슬로싱/센서동역학 배제로 실비행 강건성 갭, 비행시험 데이터 본문 미수록, 온보드 실시간/임베디드 타이밍 분석 부재(저차 LTI라 타당하나 명시 분석 없음).

**평가:** 74/100 (good) · 한줄평: ESA 자금·실증기(EAGLE) 기반의 견실한 발사체 G&C 하중경감 설계로, 강건합성·μ해석을 정석대로 운용했으나 자체 baseline과 단순화 가정으로 영향력은 중상위 수준에 그친다.

---

## (2024, AST, Adaptive multivariate reusable launch vehicles reentry attitude control with pre-specified performance in the presence of unmatched disturbances: Ge et al.)

**1. 핵심 아이디어.** RLV 재진입 자세 추종에서 제어 입력으로 직접 상쇄할 수 없는 불일치(unmatched) 외란을 처리하기 위해, 적응 유한시간 관측기(AFO)로 불일치 외란을 각속도 동역학으로 묶어(lumping) 관측 문제로 전환한다. 그 위에 사전지정 성능(pre-specified performance)을 보장하는 슬라이딩모드 제어(PSMC)를 결합하되, 적응 이득을 과대추정(over-estimation)하지 않는 시변 이득으로 설계하여 채터링과 입력 크기를 억제한다. 미지의 외란 상한을 몰라도 추종오차를 미리 정한 성능 영역 안에 가두는 것이 목표다.

**2. 알고리즘.** 자세/각속도 오차 e_θ=Θ−Θ_r, e_ω=ω−ω_r에 대한 오차 동역학(식 14)에서 불일치항을 묶은 Δ를 정의(식 15). AFO는 ż1=−k1 e1/||e1||^{1/2}−k2 e1+z2, ż2=−k3 e1/||e1||−k4 e1−R I⁻¹ΩIω+M̄−θ̇_r (식 16), 이득 k_i가 시변 스칼라 χ(t)에 따라 k1=m1√χ, k2=m2χ, k3=m3χ, k4=m4χ²로 적응(식 17), χ̇=χ̄ (식 19). 동차성(homogeneity)+Lyapunov V=½ξ^T P ξ (식 38–39)로 유한시간 수렴 증명. 제어기는 슬라이딩면 s=e_θ+ê_ω에 대해 u=−y2−||s||/(ε+||ε0||e⁻ᵗ−||s||)·sign(s) (식 59), Lyapunov 장벽함수(식 65)로 ||s||<||ε0||e⁻ᵗ+ε 의 사전지정 영역 보장(식 62). RLV 적용 시 실제 제어 M'=−ê_ω−F−||s||/(ε+||ε0||e⁻ᵗ−||s||)sign(s), M=I R⁻¹ M' (식 78).

**3. 비판적 검토.**
- 핵심 기법이 차용 조립에 가깝다. 장벽함수 적응 SMC는 [43]의 스칼라 결과를 다변수+천이성능으로 확장, AFO는 [34]/[19] 단일변수 관측기의 다변수화로, 각각의 delta는 명확하나 새로운 방향 개척은 아니다.
- Assumption 1(모든 상태 측정 가능)이 강하고 항법 오차/잡음을 고려하지 않는다. 출력피드백이 아닌 전상태 피드백 전제다.
- 시변 χ(t) 때문에 관측기 수렴시간 T_se의 정밀 해석이 어렵다고 저자 스스로 인정(Lemma 2 remark). 제어를 관측기 수렴 이후 시작한다는 가정(e2→0)에 의존한다.
- ε를 "random positive constant"로 반복 표기하는 등 정의가 느슨하고, 제어입력 크기가 10⁶~10⁷N대로 매우 커 실 액추에이터 포화에 근접한다(ASMC 대비 작다는 점은 강조).
- MC 없음, 단일 외란 프로파일·단일 초기조건으로 통계적 robustness 근거가 약하다.

**4. 시뮬/실험.** RLV 3축 재진입 자세(I_xx=434270, I_yy=961220, I_zz=1131541 kg·m², I_xz=17880; ΔI=0.05I 불확실성). 지령 Θ_r=[0.5sin t,0.4cos t,0.3sin t], 초기각 0.2rad. t=10s에 시변 불일치 외란 Δ_f, Δ_M(~10⁶) 주입(식 83). 샘플링 1ms. 추종오차·s 모두 사전지정 영역(식 79) 내 유지, t=10 이후 빠르게 재수렴(Fig.3–6). 외부 baseline은 ASMC[5] 단일 비교(Fig.7–10): AFO-ASMC가 수렴 더 빠르고, 외란 주입 시 ASMC는 제어입력이 급증해 한계 초과 위험인 반면 제안기법은 안정적. 정량 통계·MC는 없음 (모두 (sim)).

**5. Contribution(delta).** 출판년(2024) 기준 선행 대비: (i) [34]의 단일변수 적응이득 관측기를 다변수 AFO로 확장하고, 불일치 외란을 관측 문제로 전환(식 15)하는 통합 구조 제시; (ii) [43]의 스칼라 장벽함수 적응 SMC를 다변수로 확장하고 지수감쇠 시변항(||ε0||e⁻ᵗ)을 추가하여 정상상태뿐 아니라 천이(transient) 성능까지 사전지정 영역으로 보장, 또한 슬라이딩면이 0근방에 진입하는 시점을 탐지하는 메커니즘 불필요화. 미지 외란 상한·과대추정 없는 적응 이득이 실무적 장점.

**6. Limitation·Future work.** 저자: 시변 이득의 수렴시간 정밀 해석 곤란, 향후 분석 필요. 평가자 추가: 전상태 측정·항법잡음 미고려, 출력피드백 미지원, MC/통계·다중 시나리오 부재, 단일 baseline(ASMC) 비교로 우월성 근거 제한적, 거대한 제어입력의 포화/추력배분 현실성 미검토, 온보드 실시간 복잡도/타이밍 정량분석 부재.

**평가:** 67/100 (good) · 한줄평: RLV 재진입 자세제어에 불일치 외란용 다변수 AFO+장벽함수 PSMC를 견실히 통합한 in-domain G&C 논문으로, 증명은 탄탄하나 차용 조립적 novelty와 단일 시나리오·단일 baseline 검증이 한계.

---

## (2022, ASR, Adaptive finite-time prescribed performance attitude tracking control for reusable launch vehicle during reentry phase: An event-triggered case: Luo et al.)

**1. 핵심 아이디어.** RLV 재진입 단계의 자세 추종 문제를, 공력 파라미터 섭동·외란·입력 포화·전송자원 제약을 동시에 고려해 다룬다. backstepping 골격 위에 확장상태관측기(ESO)로 집합 불확실성을 추정하고, appointed-time 규정성능제어(PPC)로 추종오차를 사용자 지정 시간 내 규정 포락선에 가두며, 상대임계값(relative-threshold) 이벤트트리거 메커니즘으로 제어신호 갱신 빈도를 줄인다. Zeno-free 트리거링을 이론적으로 보장하는 것이 부가 기여다.

**2. 알고리즘.** 3-DoF 회전 동역학을 2계 연쇄형 x1=x2, x2=f(x)+U로 변환. ESO(10): z1'=z2+β1(x1-z1), z2'=z3+β2(...)+U, z3'=β3(...)로 집합외란 f-bar를 추정. 표준 변환오차 ε1=e1/ρ1, ε2=s2/ρ2 (PPC). appointed-time 성능함수 ρ(14)는 t<=Tf에서 멱함수형으로 감쇠 후 ρ_∞ 유지 → 수렴시간이 초기조건 무관. 가상제어 α1=-k1ε1+yr'+(ρ1'/ρ1)e1-θ1_hat·η1·(ε1/||ε1||) (19), 보조제어 U=k2ε2+z3-α1'-s2(ρ2'/ρ2)+θ2_hat·η2·(ε2/||ε2||) (27). 적응법 θi_hat'=γi(||εi||-2κiθi_hat). 트리거 조건(26): t_{k+1}=inf{t>tk : |e_EU,i|>=λ1,i|Ui|+τ/(τ2+τ3·e^{vt})} — 시간감쇠 항이 정상상태 정확도 저하를 제거. Lyapunov V=V1+V2로 V'<=-ξ0·V+ξ1, Lagrange 평균값정리로 트리거 간격 하한 t*>0 도출(Zeno 배제).

**3. 비판적 검토.**
- 구성요소(ESO, PPC, ETC, backstepping)가 모두 출판년 기준 성숙한 기법으로, 기여의 본질은 RLV에의 통합과 상대임계값·시간감쇠 임계값 조합에 있다. 방법론적 신규성의 delta가 얇다.
- "finite-time/appointed-time"은 제어기 자체가 아니라 외부에서 도입한 성능함수 ρ(14)에 의해 달성된다. 수렴시간 보장이 PPC 포락선의 설계 산물이라는 점에서 제목의 "finite-time"은 다소 과장된 framing이다.
- ESO 오차 e_E2의 유계성을 Zheng(2007) 인용으로 단정하고("not elaborated"), 미지 상수 θ1,θ2의 존재도 가정적으로 처리. 핵심 보조정리가 import에 의존해 자기완결적 증명은 아니다.
- 입력 포화를 다룬다고 abstract/서론에서 명시하나, 본문 제어법칙·증명에서 포화 보상 메커니즘(anti-windup 등)이 명시적으로 전개되지 않는다. 시뮬에서 20deg 제한을 언급할 뿐 포화 활성 시 안정성 증명과의 연결이 약하다.
- 가상제어 미분 α1'을 DSC 저역통과 근사로 처리하나 Remark 4에서 인용 처리하고 본문 증명엔 반영이 명확치 않다.

**4. 시뮬/실험.** 3-DoF 수치 시뮬(MATLAB, 고정 샘플 0.01s, 50s). 운용점 2개: (30km, 8Mach, α=8deg)와 (40km, 15Mach, α=13deg). 외란 d=[0.4sin...]·3성분, 공력계수 ±20% 바이어스. baseline 2종(self-implemented): Wang&Sun(2019) backstepping+제안 트리거(Ex2), 고정임계 ETC(Ex1). 정량: 제안법 ||e1||_∞=6.215e-4, 트리거 628회 / Ex1 1.2e-3, 724회 / Ex2 2.69e-3, 1756회 (Table 1). 시간트리거 대비 약 95.4% 전송자원 절감. MC 100회로 robustness 검증(±20% 공력 섭동). 모두 (sim), 외부/실측 baseline·HIL 없음.

**5. Contribution(delta).** 선행 RLV ETC 연구 대비: (i) appointed-time PPC를 ETC와 결합해 불연속 제어신호로 인한 성능 저하를 규정 포락선으로 차단, (ii) 상대임계값에 단조감쇠 시간항을 추가해 기존 고정 μ2 항이 야기하는 정상상태 정확도 손실을 제거(lim→0), (iii) RLV 자세 추종 맥락에서 ESO+PPC+ETC 통합 및 Zeno-free 증명. 출판년(2022) SOTA 기준으로 각 요소는 기존 도구이며, 신규성은 조합과 트리거 임계값 설계의 점진적 개선에 한정된다.

**6. Limitation·Future work.** 저자: fault mode, formation tracking, controller-to-actuator 채널 트리거링으로 확장 예정. 내가 보는 한계: 3-DoF 수치검증에 한정(6-DoF/병진 결합 없음), 모든 baseline이 self-implemented이고 실데이터·HIL 부재, 입력 포화의 증명-시뮬 연결이 느슨함, onboard 실시간성/연산복잡도 분석 부재(자원 절감은 통신량 기준일 뿐 계산부하 미분석), 수렴 보장이 PPC 성능함수 설계에 의존.

**평가:** 64/100 (fair) · 한줄평: 성숙한 ESO·PPC·ETC를 RLV 재진입 자세제어에 능숙히 통합하고 Zeno-free까지 증명한 견실한 in-domain G&C 논문이나, 방법론적 신규성과 검증 폭이 점진적 수준에 머문다.

---

## (2021, ASR, Re-entry trajectory tracking of reusable launch vehicle using artificial delay based robust guidance law: Sarkar et al.)

**1. 핵심 아이디어.** RLV 재진입 단계의 자세(Theta=[alpha,beta,sigma]) 추적을 위해 인공지연 기반 시간지연제어(TDC/TDE)를 제안한다. 상태의존 비선형항과 외란/관성 불확실성을 하나의 함수로 묶어(lumped) 한 샘플 이전의 입출력 데이터로 추정하고, PD 피드백과 결합해 강인 제어 모멘트를 생성한다. 기준 자세궤적은 횡거리(latitude) 최대화 최적제어 문제를 Pseudo-Spectral Method(GPOPS-II)로 오프라인 생성하여 사용한다.

**2. 알고리즘.** 자세오차 동역학 J_Theta(Theta) ddTheta_tilde = J_Theta F + d - J_Theta ddTheta_r + M 에 대해, 사용자정의 행렬 Jbar(t)=psi*J_Theta(k) (0<psi<1)를 도입해 F_hat을 한 샘플 지연 추정: F_hat(t) ≈ Jbar(t-delta) ddTheta(t-delta) - M(t-delta). 최종 제어법칙은 M(t) = -Jbar{K_D dTheta_tilde + K_P Theta_tilde} (피드백) - Jbar(t-delta) ddTheta(t-delta) + M(t-delta) (TDE). 지연 delta는 프로세서 샘플링 주기로 설정. Lyapunov V=(1/2)Theta_tilde^T Gamma Theta_tilde, K^T Gamma + Gamma K = -Xi 로 UUB 경계 r = ||phi||/(lambda_min(Xi)-rho) 도출.

**3. 비판적 검토.**
- 핵심 방법인 TDC/TDE는 로봇 매니퓰레이터/Euler-Lagrange계에서 이미 정립된 기법으로(Hsia 1990, Roy&Kar 2016/2017), 본 논문의 본질적 기여는 이를 RLV 자세동역학에 이식한 것이다. 이론적 기계장치(estimation-error 유계성, UUB)는 기존 TDC 결과를 따라가며 새로운 정리라기보다 도메인 적용에 가깝다.
- 검증이 단일 궤적/단일 외란 시나리오에 국한된다. Monte-Carlo 분산 분석이나 초기조건 산포 연구가 없어 강인성 주장이 통계적으로 뒷받침되지 못한다. 불확실성도 상수 1%/10% + 결정론적 주기외란(식45)으로 한정.
- 비교군이 QC-HO-SMC(Tian 2013b) 단일 baseline이며, 우월성 근거가 주로 채터링 정성 관찰과 "고차 미분/적분 회피로 연산부담 감소"라는 정성 주장이다. 실제 연산시간/실시간성 수치 측정이 없다.
- TDE는 J_0, R(t)가 완전히 알려져야 하며, beta=pi/2에서 det(R) 특이로 제어 불능(논문도 명시: -pi/4<beta<pi/4 제약). 또한 delta 증가 시 UUB 경계 확대-감소 trade-off를 K로 상쇄하는데, 고gain이 입력/과도응답을 악화시키는 구조적 한계가 본질적이다.
- 기준궤적은 오프라인 PSM이라 온라인 재계획/궤적 외란에 대한 적응성은 다루지 않는다(자세 추적에만 강인성 집중).

**4. 시뮬/실험.** X-33 영감 RLV, t_f=1995 s 재진입 궤적(고도 79,248→24,384 m, v 7802→762 m/s). delta=0.01 s, K_P=5I, K_D=2I, psi=0.5. 불확실성 dC_L=0.01C_L, dC_D=0.01C_D, dJ=0.1J_0 + 주기외란 d~10^6 N-m. (sim) 결과: 모든 상태가 기준궤적 추종, 경로제약(Q_r<=2.27e6, rho_d<=14.364 kPa, eta<=2.5) 만족; 자세오차 alpha~1e-4, beta/sigma~1e-3 수준 유계 수렴. QC-HO-SMC 대비 초기 과도 작고 채터링 없음(Fig.4 g-i, Fig.6,7). 추가로 delta=aT_s(a=1,2) 및 T_s=0.01~0.07 변화에 따른 오차경계 민감도 분석. MC/통계 없음.

**5. Contribution(delta).** 출판년(2021) 기준 RLV 재진입 자세제어에서 SMC/TSMC/관측기 계열이 주류였던 상황에서, 외란상한 사전지식이 불필요하고 연산이 가벼운 TDC/TDE를 RLV 자세동역학에 적용하고 UUB를 증명한 점이 delta다. SMC의 보수적 switching gain·채터링·외란경계 추정 문제를 회피한다는 점은 의미가 있으나, TDC 자체는 선행기술이라 novelty는 방법 발명이 아닌 도메인 이식+안정성 보장 수준의 점진적 기여다.

**6. Limitation·Future work.** 저자: 액추에이터 고장 강인성 미고려, VTVL RLV 등 타 구조로의 확장이 향후 과제. 내가 보는 추가 한계: (i) MC/통계 부재로 강인성 검증이 약함, (ii) 단일 baseline·실시간성 수치 미제시, (iii) J_0·R(t) 완전 가지 가정과 bank angle 특이점 제약, (iv) 오프라인 기준궤적 의존(재계획 부재), (v) delta-gain trade-off의 본질적 보수성.

**평가:** 67/100 (good) · 한줄평: 기존 TDC/TDE를 RLV 재진입 자세제어에 견실하게 이식하고 UUB까지 증명한 도메인 정합 G&C 논문이나, 방법 novelty가 점진적이고 검증이 단일 시나리오·무MC로 제한적이다.

---

## (2025, AST, Adaptive attitude tracking control for reusable launch vehicle with actuator and sensor faults: Li et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 재진입 단계에서 액추에이터(그리드 핀) 고장과 센서 고장이 동시에 존재하는 상황을 다루는 적응 고장허용제어(FTC)를 제안한다. 기존 RLV FTC가 액추에이터 고장에만 집중한 것과 달리, 다중 센서 중복성(redundancy)을 도입하고 각 상태변수마다 미분가능한 센서 융합(fusion) 함수를 구성하여 센서 고장의 영향을 자동으로 제거한다. 융합 함수를 RLV 동역학에 직접 통합하고 백스테핑 기반 적응 제어기를 설계하여, 정지(stuck) 및 유한 횟수의 부분효율손실(PLOE)을 견디면서 추종 성능을 보장한다.

**2. 알고리즘.** 센서 출력 y_{s,j}=y+η_j에 대해 거리 d_{j,k}와 성능지표 D_j(y,t)로 정상/고장 센서를 판별하고, 가중함수 w_j=ζ_j/Σζ_k (Σw_j=1, ẇ_j∈L_∞)를 설계해 융합신호 y_F=Σ w_j y_{s,j}를 만든다. 고장 센서의 가중치는 자동으로 0으로 수렴(Lemma 1, Corollary 1), 융합오차는 lim‖y_F−y‖≤(4M_y+1)/(2M_y+1)·d_m로 유계(Thm 1). 제어 모델: ẋ1=R(x1)x2+d1, ẋ2=BD_ρδ_c+Bf_f+d2. 백스테핑으로 e1=x1−Ω_c, e2=x2−x2c 정의, 가상제어 x2c=R^{-1}(−K1 e1+Ω̇_c−d̂1). 적응 제어법칙 δ_c=K̂u, u=(J^{-1}B1)^{-1}(−K2 e2−R e1−λ̂+...), 적응갱신 d̂̇1, λ̂̇, K̂̇를 σ-수정 형태로 설계. Lyapunov V=V1+½e2^T e2+(1/2μ2)λ̃^Tλ̃+(1/2μ3)tr(K̃^T D_ρ K̃)에서 V̇≤−κV+Δ로 UUB 보장(Thm 2).

**3. 비판적 검토.**
- 핵심 신규성인 "미분가능한 센서 융합으로 관측기/추정 없이 센서고장 보상"은 견고하나, Assumption 3(중복 센서 수가 홀수이고 AG 센서가 NAG보다 항상 많음)에 본질적으로 의존한다 — 이는 다수결(voting)의 일반화이며 동시 다중센서 고장에는 한계가 있다.
- Assumption 4가 d1,d2,ḋ1,ḋ2의 유계성을 직접 가정한다. d1=ΔRω−Rη2+Δf+η̄1을 명시적으로 풀어 유계성을 도출하지 않고 가정으로 처리해 이론적 완결성이 다소 약화된다.
- 추종오차 잔차집합이 d_m(센서 정확도)과 Δ(불확실성/적응항)에 비례하므로 "임의로 작게"는 설계파라미터 κ를 크게 잡는 것에 기댄다 — 큰 κ는 제어입력 포화/측정잡음 증폭 우려가 있으나 이에 대한 분석은 없다.
- 액추에이터 정지가 "최대 1개"로 제한(Assumption 1)되어 D̄D_ρ의 행 풀랭크/제어가능성을 보장. 그리드 핀 4개 중 2개 이상 정지 시 적용 불가.

**4. 시뮬/실험.** VTVL RLV 재진입 단일 시나리오 (sim). 초기조건 h=260000 ft, V=24061 ft/s, γ=−1.064°, Ω(0)=[12,−10,−30]°. 액추에이터: δ3가 t=0.3s에 0.1°로 정지, δ1이 t=10s부터 50% 효율손실. 센서고장: α,β,r 각각 시변 bias/drift 주입(예: α_{s,2}에 0.01sin+0.03sin−0.04 rad). 강건성: 공력계수 20%, 관성 20%, 질량 80kg bias + 외란벡터 Δd 추가. 결과: 제안기법은 α,β,σ 모두 가이던스 명령을 추종, 고장 센서 가중치가 자동 감소(Fig.3,5). 외부 baseline [31](적응 신경 제어)는 t≥330s에서 불안정화되어 추종 실패. Monte Carlo는 없음.

**5. Contribution(delta).** 출판년(2025) 기준 SOTA 대비: RLV FTC에서 액추에이터 고장과 센서 고장을 "동시에" 다룬 최초의 연구를 표방. 기존 RLV FTC([14,16,17,20–23])는 액추에이터 고장만, 센서고장 대응은 고장관측기/적응추정([24–32])에 국한되어 분포행렬 풀랭크 요구나 정확한 추종 실패 한계가 있었다. 본 논문은 (i) 센서마다 미분가능한 융합함수 설계(다수결의 연속·미분가능 일반화), (ii) 융합오차의 명시적 유계성 증명, (iii) 융합신호를 동역학에 통합한 백스테핑 적응 FTC로 두 고장을 통합 보상. 관측기 없이 backstepping 적합성을 확보한 점이 실질 delta.

**6. Limitation·Future work.** 저자: 추종오차가 잔차집합으로 수렴(점근 0 아님), d_m에 의존. 내가 보는 한계: (a) 단일 시나리오·MC 부재로 통계적 견고성 미검증, (b) 온보드 실시간/연산복잡도 분석 전무(그리드 핀 다중센서 융합의 계산부담 미평가), (c) Assumption 3,4의 강한 가정, (d) baseline 1종에 그침. Future work으로 동시 다중 액추에이터 고장, 측정잡음 하 강건성, HIL/실시간 검증이 필요.

**평가:** 75/100 (strong) · 한줄평: RLV 재진입에서 액추에이터+센서 동시고장을 미분가능 융합+백스테핑 적응으로 통합 보상한, 이론적으로 탄탄한 in-domain G&C 기여이나 단일 시나리오·MC/실시간 분석 부재가 strong 하단에 묶는다.

---

## (2022, Automatica, Attitude tracking control for reentry vehicles using centralised robust model predictive control: Chai et al.)

**1. 핵심 아이디어.** 재진입 비행체의 자세 추종을 위해, 대부분의 기존 설계가 채택하는 느린 각도/빠른 각속도 이중 시간척도 캐스케이드 구조를 버리고 모든 조작변수를 단일 최적화 루프에서 결정하는 중앙집중형 강건 MPC(CRMPC)를 제안한다. 비선형 피드백 법칙과 제약 강화(constraint tightening)를 통해 허용 가능한 모든 외란에 대한 강건 제약 만족을 보장하고, 종단 제어기와 종단 영역을 도입하여 재귀 실현가능성과 폐루프 입력-상태 안정성(ISS)을 보장한다. 이로써 내부 루프(각속도)용 추가 제어기 개발과 파라미터 튜닝을 제거한다.

**2. 알고리즘.** 6상태 오차계 E=[E_Θ,E_ω]^T (식 5)에 대해 명목계(식 7) 기반 open-loop 비용 J_φ(식 9, 종단비용 g_φ=||E(t_k+T)||_G^2, 과정비용 L_φ=||E||_P^2+||U_e||_Q^2)를 풀어 (x̃*,ũ*)를 얻는다. 종단 제어기 ũ^f = -Ṙ(Θ̃)ω̃ + R(Θ̃)I^-1Ω(ω̃)Iω̃ + K̃E + Θ̇_r (식 14)와 종단 불변집합 Ω_φ^tube={E:||E||_R≤ε} (식 12)을 A^TG+GA+K̃^TQK̃+P≤0 (식 15) 만족하도록 오프라인 설계. 실제 제어는 비선형 피드백 u(τ)=KO_e + R(Θ)I^-1Ω(ω)Iω - R(Θ)ω + Ṙ(Θ*)ω̃* - R(Θ̃*)I^-1Ω(ω̃*)Iω̃* + ũ* (식 20), O_e=x-x̃. Lemma 1-3가 R, R^-1, I, Ω의 노름 상하한과 튜브/강화 제약 영역을 제공.

**3. 비판적 검토.**
- 비정합 외란을 배제(Δ_f=0, 제어채널 외란 Δ_m만 고려)하여 강건 보장의 범위가 제한적이다. 저자도 비정합 외란 확장을 future work로 남긴다.
- 가중행렬 P,Q,G와 종단 이득 K,K̃의 선택이 "설계자 경험/실험 시행"에 의존하며 체계적 합성 절차가 없다. 종단 영역 크기와 예측 horizon T의 trade-off(작은 T는 연산 절감, 큰 T는 인력영역 확대)도 정성적 언급에 그친다.
- 이론적 골격(tube-MPC, 제약 강화, 종단 불변집합, ISS Lyapunov)은 표준 강건 MPC 문헌에서 가져온 것이며, 본 논문의 기여는 재진입 자세계 특유의 비선형 운동학(R, Ω의 특이점 β=±π/2 회피 포함)에 대한 노름 상하한 도출과 적용에 있다. 즉 machinery 자체는 import.
- β=±π/2에서 R, Ṙ가 특이해지는 케이스를 단순히 배제(Remark 1)할 뿐, 큰 측면각 기동에 대한 대안은 제시되지 않는다.
- Monte Carlo 캠페인이 없고 단일 명목 시나리오의 HIL 결과만 제시되어, 5% 관성 불확실성·백색잡음에 대한 통계적 강건성 분포가 부족하다.

**4. 시뮬/실험.** HIL 플랫폼(NI PXI 실시간 시뮬레이터 + ADVANTECH 제어유닛 + IMU + 3축 회전 플랫폼)에서 실제 하드웨어 검증. 관성 I_xx=588791, I_zz=24242, I_yy=1303212, I_zz=1534163 kg·m^2, 초기 Θ=[7.5°,10°,-30°], 5% 관성 불확실성, |Δ|≤2°/s 백색잡음. 외부 baseline 2종 비교: DLMPC(Bayat 2019, 이중층 MPC+TSMC)와 LQR. 정량 결과: EMS(E_θ) 1.8193→1.6055 (-11.75%), CMS(m) 0.5399→0.3552 (-34.21%); 평균 실행시간 0.1168 s vs DLMPC 0.1527 s (실행시간 분포 Fig.7). 튜브 케이스 2종(|Θ_e|≤0.2, ≤0.1) 편차가 경계 내 유지 확인(Fig.6). LQR은 제약 위반 발생.

**5. Contribution(delta).** 출판년(2022) 기준, 재진입 자세 추종에서 지배적이던 캐스케이드 이중층 구조(Bayat 2019, Mao 2020, Tian 2015)를 대체하는 중앙집중형 강건 tube-MPC를 제시한 것이 핵심 delta다. 단일 최적화 루프로 내부 각속도 루프 제어기 설계·튜닝을 제거하면서, 제약 강화 기반 강건 제약 만족 + 종단 제어기 통한 재귀 실현가능성·ISS를 동시에 보장하고, 동일 HIL에서 외부 baseline 대비 추종/제어노력/연산시간 우위를 정량적으로 입증. 방법론의 골격은 기존 강건 MPC에서 가져왔으나 재진입 운동학에 대한 적용·노름 한계 도출·HIL 검증이 명확한 진전.

**6. Limitation·Future work.** (저자) 비정합 외란 미고려, horizon T 선택의 연산-성능 trade-off, β 특이점 배제. (추가) 가중치·종단이득의 비체계적 선택, MC 부재로 통계적 강건성 미검증, 단일 시나리오, 종단영역 합성(LMI 등)의 보수성 미분석, 코드 미공개. 향후: 비정합 외란 확장, 큰 측면각/특이점 회피, 적응형 가중치, MC 기반 강건성 통계.

**평가:** 78/100 (strong) · 한줄평: 표준 tube-MPC 기법을 재진입 자세계에 견고하게 이식해 캐스케이드 구조를 단일 루프로 통합하고 HIL+외부 baseline으로 입증한, 도메인 정합성 높은 견실한 G&C 논문.

---

## (2012, J. Franklin, Launch vehicle attitude control using sliding mode control and observation techniques: Stott·Shtessel)

**1. 핵심 아이디어.** 발사체(SLV-X) 자세 제어를 위해 고전 슬라이딩 모드 제어(SMC)와 슬라이딩 모드 외란 관측기(SMDO), 그리고 Levant의 준연속(quasi-continuous) 고차 슬라이딩 모드(HOSM) 제어를 적용한다. PID+게인 스케줄링이 지배적인 발사체 제어 분야에서, 외란/플랜트 불확실성에 본질적으로 강건하고 게인 스케줄링이 불필요한 비선형 제어를 제안한다. 다중 루프(특이섭동 기반 2-루프) SMC-SMDO와 단일/이중 루프 HOSM 설계를 제시하고, 단일 루프는 채터링 없이 완전 동적 붕괴(full dynamical collapse, 쿼터니언 추종오차와 그 도함수를 유한시간에 0으로)를 달성한다.

**2. 알고리즘.** 운동방정식: 쿼터니언 운동학 q̇0=−(1/2)q̄ᵀω, q̄̇=−(1/2)Kq ω, 동역학 ω̇=J0⁻¹ΩJ0ω+J0⁻¹T+F(·). 다중 루프 SMC-SMDO: 외루프 가상제어 ωc=−2Kq⁻¹[q̄c+(K1+K2)eq+K2K1∫eq], 내루프 T=T1+T2, T1은 알려진 동역학 보상, T2=−J0F̂(·). SMDO는 보조 슬라이딩변수 s̃i=ωi−zi, 주입항 vi=−ρi sign(s̃i)(ρi>|Φ(·)i|)를 LPF로 필터링해 Φ̂(·) 추정, F̂(·)=J0Φ̂(·)+ΩJ0ω로 외란 재구성. 단일 루프 HOSM: σs=q̄c−q̄, T=2J0Kq⁻¹T̄−ΩJ0ω, T̄̇=v4, v4j는 3차 준연속 HOSM 법칙(식 34). σ,σ̇,σ̈ 계산엔 식(13)의 정확 고차 슬라이딩 모드 미분기 사용.

**3. 비판적 검토.**
- 이론적 기여가 대부분 인용 기반이다. 준연속 HOSM의 유한시간 안정성·동차성(Levant), SMDO의 수렴성·SMC 정리 등 핵심 기계는 [11,14] 등에서 가져오고, 본문은 슬라이딩변수 동역학 유도와 적용에 한정된다. 발사체 모델에 대한 자체 안정성 증명은 없다.
- 검증이 빈약하다. 단일 시나리오(SLV-X, 주로 피치 채널), Monte Carlo 없음, 외부 baseline 없음(SMC-SMDO vs HOSM은 모두 자기 방법). 정량 결과는 피치 추종오차 최대 0.02° 수준 제시뿐이고 강건성에 대한 통계적 처리가 없다.
- 시간척도 분리(K1~K4를 ζ=1.1, ωn으로 설정해 충분한 분리 확보)는 단언적이며 정량적 검증이 없다. 또한 단일 루프 HOSM이 각속도(rate)를 매우 공격적으로 만들어(인간 탑승 불가), 적용범위가 무인 ELV로 제한됨을 저자도 인정한다.
- 액추에이터 동역학을 무시(상대차수 증가·한계주기 가능성을 Remark 4로 언급만)하여 실제 토크 명령(±5×10⁵ lbs·ft 급의 고주파 진동, Fig.8)의 구현 가능성이 불명확하다.

**4. 시뮬/실험.** Matlab/Simulink 6.5 + Aerospace Blockset 기반 고충실도 6DOF SLV-X 모델(COESA 대기, WGS84 중력, KSC 풍모델+Dryden 난류). 126 s 비행(스테이징 이벤트 포함), 풍각속도 외란 최대 ~10°/s(이륙 직후), ~63 s에 안정화. (sim) 단일 루프 HOSM 피치 추종오차 평균 0(최대 0.02°), 1차·2차 도함수도 유한시간 0 수렴(Fig.5,6). Fig.9에서 SMC-SMDO와 HOSM 단일루프의 피치율 비교(외란 제거 조건). 외부 baseline·MC·통계 없음.

**5. Contribution(delta).** 출판년(2012) 기준, Levant의 준연속 HOSM과 SMDO를 발사체 자세 제어에 통합 적용하고, 단일/이중 루프 HOSM로 채터링 없는 연속 제어 하에서 완전 동적 붕괴를 보인 점이 delta. 다만 저자들의 선행 다중-시간척도 SMC 연구([10,11,16,17])에 대한 증분이 크지 않고, 방법론 자체(HOSM, SMDO)는 기존 확립된 것의 도메인 이식에 가깝다. 발사체에 단일 루프 HOSM의 full collapse를 적용한 점은 신선하나 새 방법론적 직관을 열지는 않는다.

**6. Limitation·Future work.** (저자) 단일 루프 HOSM은 공격적 각속도로 인해 무인 ELV에만 적합, 유인/재사용 발사체는 SMC-SMDO 이중루프가 필요; 액추에이터를 기생동역학으로 보면 안정성 재평가 필요. (추가) MC·외부 baseline·정량 강건성 검증 부재, 단일 채널 위주 결과, 실시간/온보드 연산부하 분석 없음, 미분기 측정노이즈 민감도는 인용으로만 처리. 토크 명령의 물리적 포화·구동기 대역 고려가 빠져 실비행 적용성 논증이 약하다.

**평가:** 59/100 (fair) · 한줄평: 발사체 자세 제어(core-GNC)에 HOSM/SMDO를 견실히 통합 적용했으나, 이론은 인용 의존적이고 단일 시나리오·MC/외부 baseline 부재로 검증 깊이가 부족한 응용 논문.

---

## (2005, CEP, Aerospace launch vehicle control: a gain scheduling approach: Clement et al.)

**1. 핵심 아이디어.** 대기권 비행 단계의 발사체(런처) 요(yaw)축 자동조종(autopilot)을 LTV(선형 시변) 모델로 보고, 궤적 상의 여러 운용점에서 각각 다목적 H_inf 제어기를 설계한 뒤 이를 안정성을 보장하며 보간(interpolation)하는 게인 스케줄링 방법론을 제시한다. 다목적 설계는 Youla 파라미터화 + LMI 최적화(각 목적마다 독립 Lyapunov 함수)로, 보간은 알려진 궤적을 이용한 이산시간 안정성 조건으로 구성된다. CNES/EADS 비선형 시뮬레이터에서 바람 외란과 파라미터 불확실성에 대해 검증한다.

**2. 알고리즘.** (i) Youla 파라미터화: 안정화 제어기 집합을 K=J*Q로 표현(식 4), 관측기 기반 구조로 차수 축소(식 5,6 Riccati T). (ii) 다목적 H_inf: 각 전달 w_i→z_i의 H_inf<gamma_i를 LMI(정리 1, 식 8)로, 목적별 독립 R_i,S_i,T_i 사용하여 보수성 감소. (iii) 게인 스케줄링 보간(정리 2): 두 상태피드백 게인 K1,K2에 대해 X1,X2>0, gamma>0가 식 (10a),(10b)의 닫힌루프 LMI와 핵심 조건 (10c) X1-X2<(c-b)gammaI를 만족하면, K(k)=( (c-k)/(c-b) K1 X1 + (k-b)/(c-b) K2 X2 )( (c-k)/(c-b) X1 + (k-b)/(c-b) X2 )^{-1} (식 11)가 LTV 폐루프를 지수 안정화. 관측기 게인은 쌍대(식 13,14), 분리정리로 관측기 기반 제어기 보간. X1=X2면 선형보간(식 12).

**3. 비판적 검토.**
- 핵심 정리(정리1, 정리2)와 보간 안정성 증명이 모두 저자들의 선행 연구(Clement & Duc, 2001; Clement, 2001 박사논문)로 인용·귀속되어 있어, 본 논문 자체는 자기완결적 증명을 담지 않는다. 적용·통합 논문 성격이 강하다.
- 기밀(confidentiality) 이유로 모든 수치값·축 단위가 비공개(Remark 1). 모든 그림이 정규화/무단위라 정량 평가와 재현이 불가능하다.
- 외부 baseline과의 정량 비교가 전혀 없다. 단일 자체 시뮬레이터 결과만 제시되고 Monte Carlo 통계도 없다(최악 파라미터 분산 케이스만 중첩).
- 비정형(nonstructured) 제어기 보간 시 안정성 보장은 Youla 파라미터가 시불변이거나 선형보간일 때로 한정(4.5절)—일반 경우 보장 손실.
- 시간영역 사양(편향각 beta, 소비 C)은 "saturations에서 멀다"고만 서술—여유는 있으나 정량 마진 미제시.

**4. 시뮬/실험.** CNES/EADS 산업용 비선형 시뮬레이터(sim). 궤적상 10개 운용점(등간격, 식 15), 각 점 제어기 차수 15, Youla 파라미터 차수 2. 5개 굽힘모드(bending mode) 포함 폐루프, 강체+굽힘모드 파라미터 최악조합 분산 적용. 주파수영역: Black-Nichols 차트(Fig.11)에서 게인마진·굽힘모드 감쇠 사양 거의 충족. 시간영역: 최악 바람 프로파일+센서잡음 하 받음각 i(Fig.13) 사양 거의 충족, beta·dβ/dt(Fig.14,15) 포화 여유, 소비 C(Fig.17) 제약 충족. 정량 수치/외부 baseline 없음.

**5. Contribution(delta).** 출판년(2005) 기준 delta: (a) Stilwell & Rugh(1999,2000)의 연속시간 보간을 이산시간으로 확장하고, 알려진 궤적 지식을 활용해 norm 제약이 필요 없는 원천적 조건 (10c)를 제시—연속시간에서 X1-X2 norm이 스케줄링 변수 변화율을 제한하던 것을 제거. (b) Youla 기반 독립-Lyapunov 다목적 H_inf 합성과 관측기 기반 게인 스케줄링을 하나의 발사체 autopilot 워크플로로 통합. 당시 게인 스케줄링 문헌이 소홀히 하던 '보간 단계의 사전 안정성 보장'을 정면으로 다룬 점이 차별점.

**6. Limitation·Future work.** 저자: 비정형 제어기 일반 보간의 안정성 미보장, 슬로싱 모드 미포함, 요축만 다룸. 내가 보는 한계: 핵심 증명 외부 의존으로 본문 자기완결성 부족; 기밀로 인한 정량/재현 불가; 외부 baseline·MC 통계 부재로 우수성의 통계적 입증 미흡; 실시간/온보드 연산복잡도(차수 15 × 보간) 분석 없음; 불확실성을 형식적 mu/IQC 보장이 아닌 최악케이스 중첩으로만 처리.

**평가:** 69/100 (good) · 한줄평: 알려진 궤적을 활용한 이산시간 안정성 보존 보간이라는 견고한 in-domain 발사체 G&C 기여이나, 핵심 증명의 외부 의존·기밀로 인한 정량/재현 부족·baseline 부재가 상위 밴드를 막는다.

---

## (2024, CEP, Fixed-time attitude control of reusable launch vehicles utilizing reliability-based control allocation: Meng et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 상승단 자세제어를 대상으로, 빠르고 정밀한 추종과 동시에 작동기 신뢰성(재사용성)을 높이는 fixed-time 제어 프레임워크를 제안한다. fixed-time RBFNN 외란관측기로 외란을 추정하고, 비특이 터미널 슬라이딩모드(NTSMC)에 anti-windup 보상기를 결합해 엔진 스윙각 포화를 다루며, Birnbaum 중요도(BIM) 기반 신뢰성 분석을 control allocation의 가중치로 직접 삽입한다. 신뢰성 가중 할당 문제를 SOCP로 변환해 MATLAB로 푼다.

**2. 알고리즘.** 모델: Theta_dot = R*omega, J*omega_dot = -omega^x J omega + tau + d, 오차동역학 x2_dot = B1 tau + D. fixed-time RBFNN 관측기: D_hat = rho_hat S(x2) + lambda1 sig^r1(e2) + lambda2 sig^r2(e2) + theta_hat (적응식 (27)-(28)). 슬라이딩면 s = x2 + alpha1 sig^beta1(x1) + alpha2*Lambda (비특이 (47)-(48)). 제어법칙 tau0 = -k B1^-1(sig^beta1(s)+sig^beta2(eps)) - B1^-1(alpha1 beta1|x1|^{beta1-1}x2 + alpha2 Lambda_dot + D_hat), anti-windup chi_dot = -k sig^beta1(chi) - k sig^beta2(chi) + B1*Delta_tau, eps = s - chi. 할당: u = argmin {u^T Omega u + c||tau0 - Bu - v + vb||^2}, Omega = Omega_hat diag(1-I_i) (BIM 기반), SOCP (77)로 정식화. Theorem 1-2가 관측오차/상태의 fixed-time 수렴을 Lyapunov로 보장.

**3. 비판적 검토.**
- novelty의 실질은 BIM 신뢰성을 allocation 가중치로 넣은 부분이며, fixed-time NTSMC + RBFNN 관측기 + anti-windup은 모두 기성 요소의 조합이다. 각 빌딩블록은 인용 lemma(Polyakov, Zuo&Tie, Jiang 등)에 크게 의존한다.
- 검증이 단일 시나리오(상승 한 구간)에 그친다. MC 분산 시험, 파라미터 불확실성 robustness 정량화가 없어 fixed-time/신뢰성 주장의 통계적 뒷받침이 약하다.
- baseline이 모두 자체 구성(기존 SMC, RBFNN 제거판, 신뢰성 무시판)으로 내부 ablation 성격이다. 외부 SOTA 할당/제어 기법과의 비교가 없다.
- SOCP를 MATLAB로 푼다고만 하고 실시간/온보드 연산시간·복잡도 분석이 없다. "real-time" 동기를 강조하나 계산예산 증거가 부재하다.
- 신뢰성 모델(failure rate, load function (14)-(16))의 파라미터(lambda^0, 상하한)가 가정적이며 실제 작동기 데이터로 보정되지 않았다.

**4. 시뮬/실험.** 모두 (sim). RLV 자세제어, J=대각우세 3x3 관성, 초기 [3.1,4.5,41]deg, 목표 Theta_d=[3,5,40.3-0.001t^2]deg, 외란 d~10^6~10^7 Nm 정현파. 관측기: 측정노이즈(std 5e-5)에서도 추정 유지; Table 1 평균추정오차 제안법 X/Y/Z=0.0021/0.0040/0.0039 (고정시간 관측기 0.015/0.0239/0.0189, RBFNN 제거판 0.039/0.066/0.051) 대비 우수. anti-windup: Table 2 포화토크 감소(예 X축 6.77e5->5.79e5). 신뢰성: Table 3 F(t) 25s~200s에서 신뢰성 무시 대비 43.6%~63.9% 불신뢰도 저감. baseline은 모두 self/내부 ablation.

**5. Contribution(delta).** 선행(Johansen&Fossen, Shen 2017, Wang 2023의 robust/pseudo-inverse 할당)이 신뢰성을 직접 다루지 않은 데 비해, BIM 중요도를 가중행렬 Omega로 환원해 신뢰성을 control allocation 루프 안으로 끌어들이고 SOCP로 동시최적화한 점이 핵심 delta. 출판년(2024) 기준 fixed-time RLV 자세제어 자체는 포화된 영역이나, "신뢰성-제어 동시 설계"는 fault-tolerant control(사후 대응)과 차별화되는 사전적(예방적) 접근으로 중간 수준의 새로움.

**6. Limitation·Future work.** 저자: CRE와 SBE의 상대적 중요도 구분 부재, 보다 표적화된 신뢰성 할당 필요. 내가 보는 한계: 단일 시나리오·MC 부재로 통계적 일반화 불충분, 외부 baseline 부재, 온보드 SOCP 연산비용 미분석, 신뢰성 파라미터의 실측 보정 부재, fixed-time 수렴이 실제로는 원점 근방(practical) 수렴이라는 점의 명시적 한계.

**평가:** 70/100 (good) · 한줄평: 신뢰성을 control allocation에 SOCP로 결합한 발상은 RLV 재사용성 측면에서 의미 있는 in-domain G&C 기여이나, 단일 시나리오·자체 baseline·온보드 연산분석 부재로 검증 깊이가 강팀 저널 상위권에는 미치지 못한다.

---

## (2016, Acta Astronautica, Robust adaptive backstepping control for reentry reusable launch vehicles: Wang et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV) 재진입 자세 동역학의 강한 비선형성·연성·시변성과 대기밀도·공력계수·외란의 불확실성을 다루기 위해, 외부 루프(자세)·내부 루프(각속도)로 구성된 2-루프 강인 적응 백스테핑 제어(RABC)를 제안한다. 핵심은 외란·불확실성의 경계가 미지(unknown)여도, 그 경계를 적응법으로 온라인 추정하고 가변구조항(variable-structure term)으로 보상하여 폐루프 전역 점근안정성을 보장한다는 점이다. 슬라이딩모드 백스테핑(SMBC)과 비교해 추종정밀도·수렴속도를 개선한다고 주장한다.

**2. 알고리즘.** 자세 동역학 Ω̇ = f(Ω)+g(Ω)ω+Δf, 각속도 동역학 JΩ̇... → Jω̇+ω×Jω = u+d (J=J₀+ΔJ, d는 등가외란). Step1(외루프): Ω̃=Ω−Ωd, V₁=½Ω̃ᵀΩ̃+(1/2c₀)ρ̃², 가상제어 ω_d = g(Ω)⁻¹[Ω̇d−f(Ω)−K₁Ω̃−ρ̂·sgn(Ω̃)], 적응법 ρ̂̇=c₀‖Ω̃‖. Step2(내루프): S=ω̃=ω−ωd, ‖d‖≤b₁+b₂‖ω‖+b₃‖ω‖² 및 ‖ω̇d‖≤b₄+b₅‖ω‖+δ‖Ω̂‖ 가정으로 ‖H(·)‖≤ξᵀQ (Q=[1 ‖ω‖ ‖ω‖² ‖Ω̂‖]ᵀ) 매개화, V₂=½ΩᵀΩ+½SᵀJS+(1/2c₀)ρ̃²+(1/2c₂)ξ̃ᵀξ̃, 제어토크 u=−(ξ̂ᵀQ)·sgn(S)−c₁S+ω×Jω, 적응법 ξ̂̇=c₂‖S‖Q. Barbalat 보조정리로 Ω̃,ω̃→0. 채터링 완화용 sat(x) 도입.

**3. 비판적 검토.**
- "explosion of complexity" 회피를 ω̇d 직접계산 대신 ‖ω̇d‖를 경계로 가정(식 26)해 처리하는데, 이는 DSC/command-filtered backstepping 같은 정식 기법 대신 임시방편 가정에 가깝다. ω̇d가 ω, Ω̂에 선형 경계된다는 가정의 타당성은 증명되지 않고 가정으로만 도입된다.
- 안정성 증명은 표준 Lyapunov+Barbalat 조합으로 자기완결적이나, 사용된 모든 기계는 교과서(Khalil [21])에서 가져온 것이며 새 이론적 도구는 없다. 적응법이 σ-modification/projection 없이 순수 적분형이라 ρ̂가 단조증가(Fig.8에서 실제로 우상향)하는 파라미터 드리프트 위험이 구조적으로 존재한다.
- 외란·불확실성 경계가 미지여도 된다고 강조하나, 실제로는 경계의 함수적 구조(다항식 차수 1,‖ω‖,‖ω‖²)는 선험적으로 가정해야 한다. 즉 "경계 미지"는 계수에 한정되고 구조는 알아야 한다.
- 비교 baseline이 저자 자신이 설정한 SMBC 단일 기법뿐이며, 외부·SOTA 기법(MPC, dynamic inversion 등 introduction에서 언급)과의 정량비교는 없다.

**4. 시뮬/실험.** 특정 RLV 1기에 대한 수치 시뮬(sim) 단일 시나리오. 관성 J₀=[554486 0 −23002; 0 1136949 0; −23002 0 1376852] kg·m², 외란 d₀=1000[sin(0.1t) cos(0.1t) sin(0.1t)]ᵀ N·m, 관성 ±10% 섭동, Δf는 f(Ω) 공칭의 ±5% 섭동. 초기자세 [40°,−45°,−3°], 약 260s 재진입. 결과: RABC 추종오차 — 받음각 <7×10⁻³ deg, 뱅크각 <2×10⁻² deg, 슬립각 <5×10⁻³ deg로 SMBC 대비 정밀도·수렴속도 개선. Monte-Carlo 없음, baseline은 self-baseline(SMBC) 1종.

**5. Contribution(delta).** 출판년(2016) 기준, RLV 재진입 자세제어에 백스테핑+가변구조 보상을 결합하고 외란/불확실성 경계의 미지 계수를 적응 추정하는 구성을 제시. 선행 SMC/SMBC([5,7,17])는 경계를 사전에 알아야 하는 한계가 있었고, 본 논문은 그 경계를 온라인 추정하여 사전지식 의존을 줄인 점이 delta. 다만 적응 백스테핑·경계추정 자체는 항공 외 제어문헌에서 이미 확립된 조합이라 방법론적 신규성은 점진적(RLV 도메인 적용+이중루프 통합 수준).

**6. Limitation·Future work.** 저자: 채터링과 안정성 간 trade-off, sat() 도입 시 점근안정성 손실. 내가 보는 한계: (i) 단일 시나리오·MC 부재로 강건성 주장이 통계적으로 미검증, (ii) 적응법 드리프트 방지 장치(projection/dead-zone) 부재, (iii) ω̇d 경계가정의 정당화 결여, (iv) 외부 SOTA baseline 부재, (v) 실시간/연산복잡도·임베디드 구현 분석 전무로 onboard 적용성 미입증, (vi) 코드/데이터 비공개. 추력 액추에이터 배분·HIL 검증이 future work으로 적절.

**평가:** 64/100 (fair) · 한줄평: 이론적으로 단정하고 RLV 도메인 적용은 적절하나, 신규성이 점진적이고 단일 시나리오·self-baseline·연산분석 부재로 G&C 저널 표준에서 견실하지만 평범한 수준의 core-GNC 논문.

---

## (2023, Acta Astronautica, Adaptive flight control system for flight safety improvement in reentry and other high-velocity vehicles: Efremov et al.)

**1. 핵심 아이디어.** 재진입체(Space Shuttle, Buran)와 초음속 수송기(SST)의 수동 조종 비행안전을 개선하기 위해, 비선형 동적역변환(NDI) 기반 baseline 제어기에 온라인 시스템 식별 기반 적응증강(adaptive augmentation)을 결합한다. 식별은 replay buffer(이동 윈도우)에 저장된 최근 비행데이터에 최소제곱을 적용해 국소 선형모델 (A, B)를 실시간 추정한다. 추가로 actuator rate-limit에 기인하는 조종사 유발 진동(PIO)을 억제하기 위해 SAAB rate limiter를 개량한 비선형 prefilter(lead-lag + feedback-with-bypass)를 도입한다. 모든 효과는 MAI 지상 시뮬레이터의 human-in-the-loop 실험으로 검증한다.

**2. 알고리즘.** NDI: x_dot=f(x)+gamma(x)u, y=d(x), y_dot=F(x)+Gamma(x)u 이므로 u=Gamma^-1(x)[v-F(x)], v=(y_des-y)*omega. 적응증강: 시스템을 trim 근방 선형화 x_dot=Ax+Bu-Ax0-Bu0 로 두고, l개 데이터점을 Eg=h (E in R^{l x (m+n+1)}, g=[A,B,T0]) 로 구성, full-rank 확인 후 g=(E^T E)^-1 E^T h 로 최소제곱 추정해 Fig.1 제어구조의 A,B를 갱신. 종방향 종축 회전: q_dot=m_u u+m_w w+m_q q+m_theta theta+m_eta eta 를 역변환하여 elevator 입력 eta=(1/m_eta)[v-(m_u u+m_w w+m_q q+m_theta theta)], v=q_dot_d. PIO 억제: 개량 rate limiter가 |delta_dot^*|/|delta_dot| 비와 게인 K_s=0.9~1.1 로 actuator 입력을 축소하여 rate-limit 내 유지하면서 위상지연 증가를 억제.

**3. 비판적 검토.**
- 이론적 신규성이 약하다. NDI(Ref.44), least-squares 식별, SAAB rate limiter(Ref.43) 모두 기존 기법의 조합/이식이며, 적응증강의 핵심도 표준 indirect adaptive + RLS류 최소제곱이다. 저자가 "usual recursive least squares보다 빠르다"고 주장하나 비교 baseline이나 수렴/안정성 증명이 제시되지 않는다.
- 폐루프 안정성·강건성에 대한 형식적 보장이 전무하다. 식별기 오차가 NDI 역변환에 들어갈 때의 안정성, persistency of excitation 조건, full-rank 실패 시 fallback의 정당성 등이 정성적으로만 다뤄진다. CLAUDE 기준의 "정형 문제정의+증명"에 미달.
- 식별 정확도 근거가 빈약하다. Fig.3에서 offline 모델이 ~4초 후 발산하고 online이 잘 추종한다고 하나, 정량 오차지표·여러 작동점에 걸친 통계가 없다. piecewise-linear 가정(Ref.45)의 적용 타당성 정량 검증도 부족.
- Rate limiter 설계(K_s=0.9~1.1, T1=0.05, K_FB=8 등)의 파라미터 선정 근거가 휴리스틱하며 PIO 억제의 이론적 보장이 아니라 실험적 관찰에 의존한다.

**4. 시뮬/실험.** MAI 지상 시뮬레이터(60Hz 시각계, 사이드스틱)에서 조종사 1명+엔지니어 2명, 케이스당 최소 5회 시행 평균. (HIL/piloted-sim) 대상: Space Shuttle 동역학 및 SST. 정량결과 — 가속 task에서 h_dot 오차분산 baseline 1.95 vs adaptive 0.7211 m^2/s^2 (약 2.7배 감소, Fig.9); inceptor deflection 오차분산 sudden alpha-failure 0.2926→0.2342, elevator-failure 0.3603→0.2615 cm^2 (최대 ~40% 감소, Table 2); sudden alpha-failure에서 2.2배 오차 감소. PIO 확률: 적응제어로 사고확률 10^-2→5x10^-4~10^-5 추정(식 20-21, PR 기반). Rate limiter: SST/Shuttle, 15·30 deg/s rate-limit에서 오차분산 (Table 3) — 예: SST 30deg/s without 0.709 → SAAB 0.357 → modified 0.277 cm^2. Baseline은 self-baseline(PI 제어기/고정계수 NDI) 위주이며 외부 SOTA 적응제어와의 직접 비교는 없음.

**5. Contribution(delta).** 출판년(2023) 기준, NDI+온라인 LS 식별 적응증강과 개량 PIO rate limiter를 재진입체/SST의 수동조종 비행안전 맥락에서 통합하고 human-in-the-loop 실험으로 PR(pilot rating)·사고확률까지 연결한 점이 delta다. 다만 개별 구성요소(NDI, indirect adaptive, SAAB limiter)는 모두 선행기술이며, 진정한 신규성은 "조종사-기체 폐루프에서 적응제어가 PIO와 고장을 동시 억제함을 piloted-sim으로 실증"한 응용적 통합에 있다. 알고리즘 자체의 이론적 진보는 incremental.

**6. Limitation·Future work.** 저자: 적응제어가 actuator의 높은 rate를 요구해 PIO 원인이 될 수 있어 rate limiter 필요; Shuttle은 큰 시간지연으로 SAAB limiter가 일부 task에서 1.36-1.37배 성능저하. 내가 보는 한계: (i) 폐루프 안정성/수렴의 형식적 보장 부재, (ii) 실제 비행이나 고충실도 6-DOF가 아닌 지상 piloted-sim 한정, 조종사 표본 3명으로 통계력 약함, (iii) 외부 SOTA(INDI fault-tolerant, MRAC 등)와의 정량 비교 부재, (iv) 코드/데이터·전체 파라미터 미공개로 재현성 낮음, (v) "RLS보다 빠르다"는 핵심 주장이 미검증.

**평가:** 57/100 (fair) · 한줄평: core-GNC 도메인의 견실한 응용 통합·piloted-sim 검증은 강점이나, 신규성은 기존기법 조합 수준이고 안정성 보장·외부 baseline·재현성이 약해 분야 평균 수준의 in-domain 논문.

---

## (2021, AST, Ascent control of heavy-lift launch vehicle with guaranteed predefined performance: Zhang et al.)

**1. 핵심 아이디어.** 초중량 발사체(HLV)의 상승단 자세추종 제어에서 오버슈트·정상상태오차·상승시간·천이시간 같은 시간영역 성능을 사전에 보장하는 것을 목표로 한다. 2차 기준모델(model reference)로 기대 오버슈트/감쇠비를 설계하고, 수렴시각을 사전 파라미터로 지정 가능한 새로운 predefined-time 규정성능함수(PPF)와 tangent 기반 오차변환을 도입한다. 여기에 predefined-time 슬라이딩면(PTSMS)과 적응 이득을 갖는 predefined-time 슬라이딩모드 제어기(APTSMC)를 결합하여 외란·모델불확실성 하에서도 사전 규정 성능을 달성한다.

**2. 알고리즘.** 핵심 구성: (i) PPF rho_i(t) = [rho_inf + (-ln(1-(1-e^{-(rho_i0-rho_inf)^tau})((T_f-t)/T_f)^{1/(1-p1)}))]^{1/tau}, 0<=t<=T_f, 이후 rho_inf로 연속 수렴(T_f, p1, tau가 사전 조정 파라미터). (ii) 변환 z_i=(1/Delta_i)(e_{1i}/rho_i - (delta_Li-delta_Ui)/2), eta_i = tan((pi/2) z_i)로 제약문제를 무제약화. (iii) 슬라이딩면 S = eta_dot + ((||eta||^mu+Upsilon)^{2-p})/(mu·Upsilon(1-p)T_c)·||eta||^{mu·p-mu}·eta. (iv) 제어법칙 U = -(RHJ^{-1}B)^{-1}{ (2^{mu-mu·p-1}/(mu·Upsilon(1-p)T_c))·sig^{mu·p-2mu+1}(S)(2^{-mu}||S||^{2mu}+Upsilon)^{2-p} + A + U_p + Sigma }, U_p는 적응 외란보상항, 적응법칙 c_hat_i_dot = -k_i l_i c_hat_i + k_i(·)||S||. 기준모델 기반 predefined-time 안정성(Theorem 2)으로 T_c 상한 보장.

**3. 비판적 검토.**
- Predefined-time 안정성의 핵심 기계(Lyapunov-like 특성화, class-K 함수, 부등식 V_dot <= -(1/(mu·Upsilon(1-p)T_c))V^{mu·p-mu+1}(V^mu+Upsilon)^{2-p})는 Sanchez-Torres/Esteban [24,31,32,34]에서 그대로 수입한 것이며, 본 논문 기여는 이를 HLV 자세제어에 적용·조립한 부분이다. 이론적 신규성은 통합에 있다.
- 베이스라인 PID·MRSMC·FxTAMRSMC가 모두 저자 측 재구현(self/internal)이며 외부 공표 방법과의 직접 비교나 몬테카를로 통계가 없다. 정량 결과(roll/yaw/pitch 최대오차 0.025/0.0610/0.131 deg)는 단일 실행값이다.
- 적응법칙은 leakage 항(-k_i l_i c_hat_i)으로 인해 표준적 UUB만 보장하며 c_hat이 참값 c로 수렴함을 보이지 않는다(Theorem 4도 S와 V_3의 predefined-time 수렴까지만, 외란 추정 정확도는 아님).
- Remark 6에서 저자 스스로 연산복잡도가 클 수 있음을 인정하고 "탑재컴퓨터 성능 향상으로 해결" 식의 정성적 언급에 그친다. 실시간/탑재 타이밍 분석이 없다.
- 강체 모델만 다루며(Remark 5), 논문 도입부에서 강조한 탄성진동·연료슬로싱은 제어기 설계에 직접 반영되지 않아 동기와 범위 사이에 간극이 있다.

**4. 시뮬/실험.** 8.5m 코어+4×3.5m 부스터, 이륙질량 2800t, 길이 95.383m HLV (sim). 시나리오 2종: (a) 단일 특징점(70s) 고정계수 비교, (b) Wenchang 발사 전체 상승단(~150s, 12s 수직선회, 정상풍+50~100s 전단풍, 10% 모델바이어스). 베이스라인 3종: PID, MRSMC, FxTAMRSMC(자체 재구현). 단일점 정량: 상승시간 1.315s, 오버슈트 4.6%, 정상상태오차 0, 천이시간 4.695s, 최대자세오차 4.003e-4 deg, 각속도오차 1.0e-7 deg/s. 상승단 정량: roll/yaw/pitch 최대오차 0.025/0.0610/0.131 deg, 최대 등가 스윙각 2.56 deg, 채터링·포화 없음. 몬테카를로 없음.

**5. Contribution(delta).** 출판년(2021) 기준 델타: (1) 수렴시각 T_f를 사전 지정하는 predefined-time PPF를 새로 제시하고, [22](appointed-time)·[30] 대비 동일 파라미터에서 더 빠른 수렴과 더 간결한 구조를 보임. (2) 수렴시간 T_c를 사전 파라미터로 갖는 predefined-time 슬라이딩면을 설계. (3) 이를 HLV 상승단 자세제어에 적용한 첫 규정성능제어(PPC) 사례를 주장. 다만 PTC·PPC·MRAC 각 요소는 모두 선행 존재하므로 본질은 확립된 구성요소들의 도메인-특화 통합이다.

**6. Limitation·Future work.** 저자: 강체모델 한정, 적응 notch 필터·고장검출·능동하중저감 미반영(Remark 5), 연산복잡도(Remark 6). 추가로 평자 관점: 외부 공표 베이스라인 부재와 몬테카를로 미수행으로 강건성 주장의 통계적 근거가 약함; 탑재 실시간성에 대한 타이밍/복잡도 정량 분석 부재; predefined-time 안정성 핵심 정리들의 자체 기여 비중이 제한적; 도입부의 탄성/슬로싱 동기와 강체 설계 사이의 간극.

**평가:** 68/100 (good) · 한줄평: predefined-time PPF·SMC를 HLV 상승단에 견실하게 통합한 in-domain G&C 저널 논문이나, 핵심 안정성 기계의 수입 비중과 자체 베이스라인·무 MC로 신규성/검증 강도가 제한적.

---

## (2018, J. Franklin, Multivariable uniform finite-time output feedback reentry attitude control for RLV with mismatched disturbance: Tian et al.)

**1. 핵심 아이디어.** RLV 재진입 자세제어에서 정합(matched)·비정합(mismatched) 외란이 모두 존재하고 각속도 상태를 측정할 수 없는 상황을 다룬다. bi-limit 동차성(homogeneity)을 이용해 수렴시간이 초기조건과 무관하게 균일하게 유계인(uniform finite-time) 다변수 연속 제어기를 설계하고, 임의차수 균일수렴 슬라이딩모드 미분기 기반 관측기로 미지상태와 합성외란을 동시에 추정하여 출력피드백 제어를 구성한다. Lyapunov와 동차성 기법으로 폐루프의 균일 유한시간 안정성을 증명한다.

**2. 알고리즘.** 이중적분기 x1'=x2, x2'=u 에 대해 u(x) = -k1(||x1||^ρ1 sign(x1) + x1 + ||x1||^ρ1' sign(x1)) - k2(||x2||^ρ2 sign(x2) + x2 + ||x2||^ρ2' sign(x2)), ρ1=ρ/(2-ρ), ρ2=ρ, ρ1'=(4-3ρ)/(2-ρ), ρ2'=(4-3ρ)/(3-2ρ), ρ∈(0,1). 자세모델 Θ'=Rω+Δ1, Iω'=-ΩIω+M+Δ2 를 ω̄=Rω, M̄=RI⁻¹M 로 변환해 오차동역학 z1'=z2, z2'=M̄+F(z2)+Δ 로 만들고, 제어 M̄ = Θ_ref'' - F(ẑ2) - ẑ3 - (위 이중적분기 제어형). 관측기는 e1=z1-ẑ1 기반의 3단 동차/SMC 미분기(식 21)로 ẑ1,ẑ2,ẑ3(=Δ 추정)을 동시 산출. 실제모멘트 M=IR⁻¹M̄ (det R = 1/cosβ, β≈0이라 역행렬 존재).

**3. 비판적 검토.**
- 핵심 안정성 도구인 bi-limit Lemma 1(Andrieu/Praly/Astolfi)과 임의차수 미분기 수렴(Angulo/Moreno/Fridman, Levant)은 인용·수입이며 본문 증명은 이를 조합·적용하는 수준이다. 저자도 Remark 3에서 결과가 컴팩트 집합 내 국소(local)임을 정직히 인정한다.
- 검증이 단일 차량(X-33급)·단일 궤적·MATLAB 시뮬에 국한되고, 비교 baseline은 자체 구현 ADRC 하나뿐이다. 외부 표준 SMC/HOSM과의 직접 비교나 다중 시나리오가 없다.
- 관측기/제어기 파라미터(λ1,2,3, Hurwitz 행렬, κ_i, α, T_u)가 시행착오·레시피 기반으로, 게인 선택의 체계적 지침이나 강건성 마진 분석이 부족하다.
- 온보드 실시간성·연산복잡도에 대한 정량 분석이 없다(1ms 고정스텝 시뮬만 제시).

**4. 시뮬/실험.** MATLAB/Simulink, 1ms 고정스텝, X-33급 관성(Ixx=43,427 등), α0=12.62°, β0=-11.46°, σ0=-57.30°. 외란 Δ1은 정현파 합(식 34), Δ2=2×10⁶Δ1. 500초 가이던스 명령 추종에서 추종오차 약 10⁻³ deg, 상태추정 오차 약 10⁻⁶ deg (sim). ADRC 대비 추종정확도 향상·제어노력 감소(초기 ESO 고게인으로 인한 ADRC 큰 진폭 회피). 200-run 몬테카를로(관성 ±15% 불확실성)로 강건성 확인 (sim). 실제/HIL 검증은 없음.

**5. Contribution(delta).** 출판년(2018) 기준 SOTA 대비 delta는 (i) 초기조건 독립의 균일(uniform) 유한시간 수렴을 bi-limit 동차성으로 다변수 RLV 자세제어에 도입한 점(기존 [19]~[31]의 유한시간법은 큰 초기조건일수록 수렴시간이 크게 증가), (ii) 정합·비정합 외란을 모두 갖는 RLV를 출력피드백으로 안정화한 점, (iii) 미지상태와 합성외란을 동시 추정하는 다변수 관측기와의 결합 및 엄밀한 균일 유한시간 안정성 증명. 저자 주장대로 이 조합은 당시 공개문헌에 보고되지 않았다.

**6. Limitation·Future work.** (저자) 결과는 국소적이며 β≈0 가정에 R⁻¹ 존재가 의존. (평가자) 단일 시나리오·자체 baseline·게인튜닝 비체계성·온보드 복잡도 미분석이 한계. 다양한 재진입 궤적·다중 외부 baseline·실시간 임베디드 검증, 게인 합성의 체계화가 후속 과제.

**평가:** 74/100 (good) · 한줄평: 동차성 기반 균일 유한시간 출력피드백을 정합·비정합 외란 RLV 자세제어에 잘 결합한 견실한 in-domain G&C 논문이나, 수입된 안정성 도구·단일 시나리오·자체 baseline으로 영향력은 분야 내 점진적 개선 수준.

---

## (2018, AST, Fixed-time extended state observer based non-singular fast terminal sliding mode control for a VTVL reusable launch vehicle: Zhang et al.)

**1. 핵심 아이디어.** VTVL 재사용 발사체(RLV)의 내권(endoatmosphere) 단계 자세 추종 제어를 다룬다. 그리드핀을 구동기로 사용하는 6자유도 모델을 세우고, 고정시간(fixed-time) 수렴 확장상태관측기(FxTESO)로 자세 추종오차·그 미분·총외란을 동시 추정한 뒤, 추정값 기반의 비특이 고속 종단 슬라이딩면(NFTSM)과 고정시간 제어법칙을 설계한다. 외란 상한을 사전에 알 필요가 없고 제어이득을 외란보다 크게 둘 필요가 없다는 점이 핵심 주장이다.

**2. 알고리즘.** 추종오차 상태 x1=Ω-Ωc, x2=x1_dot에 대해 이중적분 오차계 x1_dot=x2, x2_dot=H+BU (H=F+ΔD-Ωc_ddot). FxTESO(식 19)는 sig^α/sig^β 이중항과 스위치함수 Λ, Υ·sign(x1-Z1) 항으로 Z1,Z2,Z3(=H 추정)를 고정시간 추정. 슬라이딩면 S=sig^v1(x1)+(λ2v2/(2v2-1))·sig^(2-1/v2)(Z2+λ1·sig^v1(x1)) (식 34), 추정값 Z2를 사용해 x2 직접 미분을 회피. 제어법칙 U=-B^{-1}(k1v1·diag(|x1|^{v1-1})(Γ/k1+x2)+χ·sig^γ1(S)+λ·sig^γ2(S)+Z3) (식 41). 안정성은 Lyapunov V_S=½S^T S, V̇_S ≤ -K·V_S^{(γ1+1)/2}-M·V_S^{(γ2+1)/2} 형태로 고정시간 수렴 증명(정리 1-3).

**3. 비판적 검토.**
- 증명은 Lemma 1-5(refs [18,20,21,28,36,38])의 고정시간 안정성 기계를 그대로 가져와 조립한 형태로, 핵심 부등식은 자기완결적 도출이라기보다 인용 보조정리에 의존한다.
- 슬라이딩면이 ref [36]의 part-motivated라고 명시되어 있어 순수 신규성은 제한적이며, 본질은 "추정 x2(=Z2) 대입"이라는 변형이다.
- 검증이 단일 시나리오 + 자체구현 단일 baseline(SFTSMC, ref [36])뿐이고 Monte-Carlo·통계가 없어 강건성 주장의 정량적 근거가 약하다.
- FxTESO 이득 e^{η+b},e^{2η+b},e^{3η+b}와 스위치시간 T_u, η/b/ε 등 설계 파라미터가 다수이며 그 선정 가이드가 경험적이다(Remark 1에서 기존 관측기 대비 단순함을 주장하나 여전히 튜닝 부담 존재).
- 실시간성/연산복잡도/탑재 가능성에 대한 정량 분석이 없다(샘플링 5ms 언급만).

**4. 시뮬/실험.** 6자유도 수치시뮬(sim) 단일 시나리오: h=25km, V=1180 m/s, 시뮬 72 s, 샘플링 5 ms, 제어입력 ±30° 제한. 불확실성 20% bias(공력계수·관성모멘트·대기밀도) + 질량 100kg bias 주입. 결과: 3채널 자세추종오차 < 0.04°, 약 2 s 내 수렴, SFTSMC(ref [36]) 대비 특히 롤 채널에서 우수(SFTSMC 오차 < 0.1°). 채터링 회피 확인, FxTESO 추정 성능(Z1-Z3) 양호, 속도/고도 프로파일이 연착륙 요구 충족. baseline은 자체 구현 1종, real/HIL 없음.

**5. Contribution(delta).** (i) 기존 고정시간 외란관측기 대비 더 단순한 구조·적은 파라미터를 갖는 FxTESO 제시, (ii) 자세추종오차 미분 계산을 회피하기 위해 추정값 Z2를 슬라이딩면에 대입하는 비특이 고속 종단 슬라이딩면 설계, (iii) 외란 상한 사전지식·이득 부등식 불요의 고정시간 NFTSM 제어기. 출판년(2018) 기준 고정시간 SMC+ESO는 활발한 주제였고, VTVL RLV 그리드핀 자세제어에 첫 적용을 표방한 점이 delta이나 방법론 자체는 기존 요소의 결합·변형에 가깝다.

**6. Limitation·Future work.** 저자: 외란/불확실성을 사전에 알 수 없어 SFTSMC의 k 선정이 어렵다는 동기는 해소했으나 FxTESO 자체 이득 설계는 여전히 남는다. 내가 보는 한계: 단일 시나리오·MC 부재로 통계적 강건성 미입증, 외부/실측 baseline 부재, 탑재 실시간성 미검증, 그리드핀 구동기 동특성·포화·지연 모델 단순화, ξ3≡0 불가(샘플링·노이즈)에 따른 잔여 수렴영역 존재. 향후 비행/HIL 검증과 다수 분산 시나리오 평가가 필요.

**평가:** 70/100 (good) · 한줄평: VTVL RLV 자세제어에 고정시간 ESO+NFTSM을 견고하게 조립한 in-domain G&C 논문으로 증명·시뮬 모두 견실하나, 신규성은 인용 보조정리 기반 결합·변형 수준이고 단일 시나리오·자체 baseline이라 검증 폭이 제한적이다.

---

## (2019, Acta Astronautica, Adaptive fault-tolerant control for a VTVL reusable launch vehicle: Zhang et al.)

**1. 핵심 아이디어.** VTVL 재사용 발사체의 엔도애트모스피어(내대기) 단계 자세 추적에 대해, 불확실 파라미터·미모델링·외란·그리드핀 액추에이터 고장을 동시에 다루는 능동형 FTC를 제안한다. 자세 추적 오차 동역학을 이중적분 2차 시스템으로 변환한 뒤, (i) 추적오차 미분과 집중외란을 추정하는 고정시간 확장상태관측기(FxTESO), (ii) 두 개의 전통적 빠른 종단 슬라이딩면을 스위칭 결합한 비특이 빠른 종단 슬라이딩면(NFTSMS), (iii) 적응 이득 K1/Ψ(S)를 갖는 적응 FTC를 결합한다. 제어행렬 B만 알면 되고 외란 상한 지식이 불필요하다는 점을 강조한다.

**2. 알고리즘.** 오차 상태: e1=Ω−Ωc, e2=Ω̇−Ω̇c, ė1=e2, ė2=H+BU (H=F1+ΔD−Ω̈c 가 집중외란). FxTESO(식18): Ż1=Z2+k1Θ·sig^α1(x1−Z1)+k1(1−Θ)sig^β1(·), Ż2=Z3+k2Θ·sig^α2(·)+k2(1−Θ)sig^β2(·)+BU, Ż3=k3Θ·sig^α3(·)+k3(1−Θ)sig^β3(·)+Υsign(Z1−x1), Θ는 시각 Tu 기준 스위칭. 슬라이딩면(식20): S=e1+sig^γ2(K(e1)Z2), 이득 k_i(e1i)는 |e1i|≥ε0/<ε0 두 구간 정의로 비특이성 확보. 제어칙(식30): U=−B^{-1}( (K1/Ψ(S))[sig^μ(S)+sig^η(S)+S] + Z3 + F ), Ψ(S)=k+(1−k)e^{−τ1|S|^c} 적응 게인. Thm1~3에서 e1,e2,S가 고정시간 Tc≤T1+T2+T3 내 소영역 수렴함을 Lyapunov(V=½SᵀS 등)로 증명.

**3. 비판적 검토.**
- 핵심 빌딩블록(Polyakov 고정시간 안정성, 종단 SMC, ESO)이 모두 기존 결과이며, FxTESO의 수렴 증명(Thm1)은 직접 증명하지 않고 [41]/[26]로 인용 처리. 기여는 이들의 조합·재배열 성격이 강하다.
- "novel" 슬라이딩면은 기존 두 FTSM 면(S1,S2)을 |x|≷ε0로 스위칭 결합한 것으로, 비특이성·빠른 수렴 주장은 타당하나 개념적 신규성은 제한적.
- 집중외란 H가 연속 미분가능하고 ‖H‖,‖Ḣ‖이 알려진 상한 이내라는 Assumption 1은 액추에이터 고장(특히 lock-in-place/hard-over의 불연속 점프)과 정합성이 약할 수 있다 — 고장 발생 순간의 비연속성에 대한 논의 부재.
- 제어행렬 B(=RJ^{-1}B1)가 가역·기지라고 가정하나, B1은 동압 q0·제어모멘트계수에 의존하므로 비행 중 변동/추정 오차에 대한 민감도 분석이 없다.
- 검증이 단일 초기조건·단일 시나리오이며 Monte Carlo·통계 처리가 없어 강건성 주장(20% 바이어스)이 대표성 한 점으로만 뒷받침됨.

**4. 시뮬/실험.** 수치 시뮬(sim)만. h=25000m, V=1180m/s, γ=−4°, 초기 α=−5.317°,β=−0.573°,σ=−1.146°. 불확실성: 공력계수/관성/공기밀도 20% 바이어스 + 질량 100kg 바이어스. 액추에이터 고장: t>10s에 LOE Λ_z=0.6 및 시변 바이어스 ρ1z=0.04sin(0.2(t−10)). 제어입력 ±30°. 베이스라인 3종(Polyakov[38], Yang[39], TFxTSMC) 자체 구현 비교. Table 2(롤 채널): 추적오차 proposed 0.02973° vs Polyakov 0.04942°, Yang 0.05011°, TFxTSMC −0.4233°; 수렴시간 1.65s vs 2.18/3.33/>10s. 모든 채널 추적오차 <0.03°, 약 1.65s 내 수렴, 채터링 없음 주장. MC·통계 없음.

**5. Contribution(delta).** 출판년(2019) 기준 SOTA(Polyakov 고정시간 제어, 적응 KF/SMO 기반 RLV FTC) 대비: (1) 자세 추적오차 미분(e2=Ω̇−Ω̇c)을 직접 계산하지 않고 FxTESO로 추정하여 Ω̈c 미지 항을 회피, 모델 파라미터 의존성·연산량 저감. (2) 두 FTSM 면을 결합한 비특이 NFTSMS로 기존 FTSM 대비 빠른 수렴·특이점 회피 동시 확보. (3) Ψ(S) 적응이득으로 도달시간 단축. 전체적으로는 고정시간 ESO+SMC+적응이득의 통합 적용에 가까운 점진적 진전이며, VTVL 그리드핀 고장에 특화한 점이 도메인 기여.

**6. Limitation·Future work.** 저자: 내대기 단계만 다루며 다른 비행 단계로의 확장 미논의. 추가로 내가 보는 한계 — (a) MC/통계 부재로 강건성 정량 근거 약함, (b) 실시간성·연산복잡도(관측기+스위칭 면) 정량 분석 없음(HIL/임베디드 미검증), (c) 고장 불연속과 Assumption 1의 정합성, (d) 베이스라인이 모두 자체 재구현이라 외부 벤치마크 부재, (e) B 추정오차/입력포화 상호작용 미분석. Future: 다단 비행 통합 FTC, MC 기반 강건성·실시간 구현 검증.

**평가:** 66/100 (good) · 한줄평: 고정시간 ESO·비특이 종단 SMC·적응이득을 그리드핀 고장 VTVL 자세제어에 견실히 통합했으나, 신규성은 점진적이고 단일 시나리오·자체 베이스라인·MC 부재로 검증 깊이가 제한된 정통 core-G&C 저널 논문.

---

## (2020, AST, Fixed-time observer based fault tolerant attitude control for reusable launch vehicle with actuator faults: Liang et al.)

**1. 핵심 아이디어.** VTVL 형태 재사용 발사체(RLV)의 재진입 단계 자세추종 문제에서 모델 불확실성, 외란, 그리고 작동기(그리드핀) 고장(이득·바이어스)을 동시에 다루는 능동 내고장(FTC) 기법을 제안한다. 핵심은 초기조건과 무관한 고정시간(fixed-time) 내에 복합외란 ΔD를 추정하는 고정시간 관측기를 설계하고, 그 추정치를 비특이 종단 슬라이딩모드(NTSM) 제어법칙에 주입하여 폐루프 유한시간 안정성을 달성하는 것이다. 수치 시뮬레이션으로 효과를 검증한다.

**2. 알고리즘.** 자세 동역학을 2차 추종오차계 x1=Ω−Ωd, x2 형태로 변형(식 11): ẋ1=x2, ẋ2=A+Bτ+ΔD. 고정시간 관측기(식 15): ż1=−l1·e/||e||^{1/2} − l2·e||e||^{μ−1} + z2 + A + Bτ, ż2=−l3·e/||e|| (e=z1−x2), 게인조건 l1>√(2l3), l2>0, l3>4D̄로 ΔD≈z2를 고정시간 t_f 내 추정(정리 1). 슬라이딩면(식 35): S=x1 + (1/κ)x2^{m/n}. FTC 법칙(식 36): τ = −B⁻¹[A + κ(m/n)x2^{2−m/n} + z2 + ξ·sgn⟨S⟩^υ + η·sgn(S)]. Lyapunov V=½S²로 유한시간 도달성 증명(정리 2).

**3. 비판적 검토.**
- 신규성이 약하다. 2020년 시점에서 고정시간 외란관측기와 고정/유한시간 SMC는 극초음속/RLV 자세제어에 이미 광범위하게 정립되어 있었고(참고문헌 [11][12][13][31][32][33] 등 2017–2019), 본 논문은 이들 기성 도구를 그리드핀 작동기·복합외란 조합에 적용한 점진적 결합에 가깝다.
- 검증이 단일 시나리오·단일 외부 baseline([19])에 그친다. Monte Carlo나 파라미터 분산 연구가 없어 강건성에 대한 통계적 근거가 부족하다. "20% 바이어스" 등 단일 명목 케이스만 제시된다.
- 증명이 자체 완결적이지 않다. 정리 1의 3단계 증명과 정리 2가 외부 보조정리(Lemma 1 in [21], Theorem 4.5/Lemma in [38][39])에 핵심적으로 의존한다. 장르 표준이긴 하나 핵심 수렴 기계는 인용 수입이다.
- 모델 변환에서 식 (9)의 2차 미분 과정 중 B(=GI⁻¹b) 행렬이 비행상태 의존인데 그 미분 항(Ġ 등)의 처리가 다소 매끄럽게 가정되어 ΔD에 흡수된다. Assumption 1(||ΔḊ||≤D̄)로 정당화하나 D̄의 실현 가능성에 대한 정량 논의가 없다.
- 정량 결과가 thin하다. 추종오차 0.01 vs 0.018, 수렴 16s vs 26s 정도의 비교만 제시되고, 고정시간 관측기의 핵심 강점(초기조건 독립 수렴시간)이 시뮬레이션에서 다양한 초기조건으로 직접 검증되지 않았다.

**4. 시뮬/실험.** 단일 재진입 시나리오 (h=25 km, V=1200 m/s, γ=−41°, m=6165 kg). 목표 [α,β,σ]=[15°,0°,−35°]. 시변 외란 Δd 및 시변 작동기 고장(t≥15s에서 이득 e1=0.7, e2=0.8, e3=0.8 단계 고장 + 바이어스 ρ1) 인가. Baseline은 [19]의 FTC 기법(외부, non-self) 1종. 결과(모두 sim): 측슬립각 추종 16s/오차 0.01 (제안) vs 26s/0.018 ([19]); 제어입력 ±10° 이내; 에너지 E=½∫||u||dt 히스토그램에서 제안 기법이 [19]보다 적은 에너지 소비. MC·통계처리 없음.

**5. Contribution(delta).** 선행 대비 delta는 (i) 복합외란(불확실성+외란+이득/바이어스 고장)을 단일 고정시간 관측기로 통합 추정하고 (ii) 이를 VTVL RLV 그리드핀 작동기 모델에 특화된 NTSM FTC와 결합한 점이다. 다만 2020년 SOTA 기준으로 고정시간 관측기·NTSM·유한시간 FTC 각각은 이미 표준이었으므로, 신규성은 새 방향 개척이 아니라 기존 요소의 도메인 특화 결합(점진적 개선)으로 평가된다.

**6. Limitation·Future work.** 저자: 고정시간 제어의 에너지 소비가 커서 작동기 포화·2차 손상 우려로 FTC부에서는 유한시간 전략을 채택(에너지-수렴속도 trade-off). 내가 보는 한계: 단일 시나리오·MC 부재로 강건성 정량 근거 부족; 외부 baseline 1종뿐; 고정시간 관측기의 초기조건 독립성이라는 핵심 주장이 다양한 초기조건 sweep으로 검증되지 않음; 센서/항법 잡음, 작동기 동역학·지연 미반영; 게인조건의 보수성(l3>4D̄)과 채터링 실측 부재.

**평가:** 61/100 (fair) · 한줄평: RLV 재진입 자세 내고장 제어에 제대로 들어맞는 도메인 적합 논문이나, 2020년 기준 기성 고정시간 관측기·NTSM의 점진적 결합이고 단일 시나리오·baseline 1종으로 검증이 얕다.

---

## (2022, AST, Quaternion observer-based sliding mode attitude fault-tolerant control for the Reusable Launch Vehicle during reentry stage: Liang et al.)

**1. 핵심 아이디어.** 재진입 단계 RLV(재사용 발사체)의 자세 결함허용제어(FTC) 문제를, Euler각 대신 단위 쿼터니언 기반 자세 동역학으로 정식화하여 광범위 자세 기동에서의 특이점을 회피한다. 액추에이터 부분 효율저하·바이어스 고장과 공력 불확실성·외란을 하나의 합성외란(compound disturbance) ΔT_d로 묶어 확장상태로 보고, 단위 쿼터니언의 노름 제약을 보존하는 신형 쿼터니언 관측기로 상태와 ΔT_d를 추정한다. 추정치를 이용해 다변수 적분 터미널 슬라이딩모드 FTC 법칙을 설계하여 고장 상황에서도 유한시간 자세추종을 보장한다.

**2. 알고리즘.** 모델: q̇=C(q)ω, J ω̇ = -ω×(Jω)+T_a+T_ad+Bδ+ΔT_d (식 13), ΔT_d≜T_d+B((κ-I)δ+ρ) (고장·외란 통합). 쿼터니언 관측기(식 14): q̂̇=C(q̃)Gᵀ(q̂)[ω̃ + a₁C_v⁻¹q̃̂_v/τ + 2bq̂_v/(q̂₀(1-q̂_vᵀq̂_v))], ω̃̇=J⁻¹[…+ΔT̂_d + a₂C_v⁻¹q̃̂_v/τ²], ΔT̂̇_d = a₃C_v⁻¹q̃̂_v/τ³. 슬라이딩면(식 63): S=ω_e+∫(μ₁‖q_e‖^{k₁} q_eC(q_e)/‖q_e‖ + μ₂‖ω_e‖^{k₁} ω_e/‖ω_e‖)dτ, k₁=2k₂/(2-k₂). 제어법칙(식 65): δ=-B⁻¹J[F(q_e,ω_e)+J⁻¹ΔT̂_d+μ₁‖q_e‖^{k₁}q_eC(q_e)/‖q_e‖+μ₂‖ω_e‖^{k₂}ω_e/‖ω_e‖+l₁S+l₂‖S‖^{k₃}S/‖S‖]. Lyapunov(Lemma1, Thm1: V₁,V₂로 관측오차 잔차집합 수렴; Thm2: V₃,V₄ + 동차성(homogeneity) 논거로 추종오차 유한시간 수렴) 증명 제시.

**3. 비판적 검토.**
- 관측기 게인이 1/τ, 1/τ², 1/τ³ 구조(τ=0.01 → ~10⁶)로, τ→0일 때만 수렴이 강해지는 전형적 고이득(high-gain) 관측기다. 측정잡음 민감도·peaking 현상에 대한 분석이 전혀 없어 실측 환경 적용성에 의문이 남는다.
- 수렴은 "충분히 작은 잔차집합으로의 practical 수렴"이며 정확 수렴이 아니다. 또한 ΔT_d, ΔṪ_d의 유계성(Assumption 2)을 가정하는데, 액추에이터 바이어스가 sin으로 가해지는 시뮬과 일관되나 일반 고장에 대한 정당화는 약하다.
- 제어 분배행렬 B의 가역성을 전제(δ=-B⁻¹J[…])하나, 그리드핀 4채널→3채널 등가변환(식4)의 잉여성 활용·고장 시 분배 재구성은 다루지 않는다. 본문이 강조한 "다변수 설계로 액추에이터 잉여성을 활용"이라는 주장과 실제 알고리즘 사이에 간극이 있다.
- 새로움의 핵심으로 내세운 항목들(터미널 SMC FTC, 외란/확장상태 관측기, 쿼터니언 자세)이 모두 출판년 기준 기성 도구이며, 진짜 차별점은 "노름제약 보존 쿼터니언 관측기"라는 좁은 부분이다.
- 단일 결정론적 시나리오·단일 외부 비교군([24])뿐이며 Monte Carlo, 관성/공력계수 분산, 잡음 주입 연구가 없다. 통계적 강건성 주장이 뒷받침되지 않는다.

**4. 시뮬/실험.** VTOL형 RLV 1개 시나리오(sim). 파라미터 전부 명시: s_r=3.976 m², l_r=20 m, J_x=5285, J_y=J_z=148700 kg·m², 명령 ω_c=[0.32cos(0.01t),0.12sin(0.02t),-0.32sin(0.01t)] rad/s, 외란 T_d=[0.1sin(0.1t),0.2cos(0.2t),0.15cos(0.2t)] N·m, 시변 효율저하 κ(20s에 0.6~0.8) 및 15s 바이어스 ρ_x=0.05sin. 관측기/제어 게인 a₁=a₂=3, a₃=1, τ=0.01, μ₁=0.75, μ₂=1.5, l₁=l₂=3.65. 정량결과(제안 vs [24]): q₃ 오차<0.01를 12s 내 달성(vs 27s, 0.017); ω 오차<0.05(vs 0.09); 관측오차 3s 내 수렴; 제어입력 ±5 deg 이내. 외부 baseline은 [24](선행 fixed-time 관측기 기반 FTC)로 적절하나 1종뿐.

**5. Contribution(delta).** 선행 대비 delta: (1) Euler각 기반 [4,14,33-35]과 달리 쿼터니언 기반 자세 동역학으로 광범위 기동 특이점 회피, (2) 단위 쿼터니언의 노름 제약을 이론적으로 보존하는 신형 쿼터니언 관측기로 상태+합성외란 동시 추정, (3) [38]의 m개 SISO 재모델링과 달리 다변수 적분 터미널 SMC로 디커플링 과정 제거. 출판년(2022) SOTA 기준으로 보면 closest prior art([24])에 대한 명확하나 점진적인 개선으로, 추종속도·정밀도에서 정량 우위를 보이나 방법론적 패러다임 전환은 아니다.

**6. Limitation·Future work.** 저자: 액추에이터 포화 하의 FTC를 향후 과제로 명시. 추가로 본 평가자 관점: (a) 고이득 관측기의 잡음 민감도·peaking 미분석, (b) practical 수렴(정확 수렴 아님), (c) 단일 시나리오·Monte Carlo 부재로 통계적 강건성 미검증, (d) B 가역 전제와 그리드핀 잉여성 미활용 간 간극, (e) 실시간 연산복잡도·온보드 적용성 정량분석 부재, (f) 센서 고장·측정잡음 미고려.

**평가:** 68/100 (good) · 한줄평: 쿼터니언 노름제약 보존 관측기 + 다변수 터미널 SMC라는 견실한 in-domain RLV 재진입 FTC 논문이나, 구성요소가 기성 도구의 점진적 결합이고 검증이 단일 시나리오·단일 baseline에 그쳐 영향력은 중간 수준.

---

## (2021, AST, Adaptive prescribed performance attitude control for RLV with mismatched disturbance: Tian et al.)

**1. 핵심 아이디어.** RLV 재진입 자세제어를 대상으로, 유계이지만 상한이 미지인 matched/mismatched 외란 하에서 과도·정상상태 성능을 동시에 보장하는 적응 다변수 prescribed performance control(PPC)을 제안한다. 핵심 특징은 규정 성능 영역(prescribed region)이 외란 상한에 독립적이며, 추종오차가 predefined-time(사전 지정 시간 T) 내에 0의 작은 근방으로 수렴한다는 점이다. backstepping + 시변 적응이득 + Lyapunov 안정성 증명으로 구성된다.

**2. 알고리즘.** 일반 적분기계 x1'=x2+Δ1, x2'=u+Δ2에 대해 가상입력 x2d=-Kb(x1,t)·sign(x1), 시변이득 Kb(e1,t)=‖e1‖/(ε+f(e10,t)-‖e1‖). 제어법칙 u = -[x2(ε+f)-x1 f' -Kb(e,t)(ε+f)sign(e)]/(ε+f-‖x1‖)^2 - (‖e‖/(ε+f(e0,t)-‖e‖))·sign(e), e=x2-x2d. 자세적용: e1=Θ-Θref, e2=Rω-Θref'에 대해 u' = -F - e2(ε+f10)-e1 f10' /(...)^2 - Kb(z,t)(ε+f)sign(z)/(...) - (‖z‖/(ε+f(z0)-‖z‖))sign(z), z=e2-e2d. 성능함수 f(e1,t)=‖e10‖((T-t)/T)^2 (t≤T), 0 (t>T)으로 predefined-time 수렴 부여.

**3. 비판적 검토.**
- 외란 상한 미지·mismatched 처리와 predefined-time 보장은 의미 있으나, 제어구조는 PPC + backstepping + sign() 이라는 기성 기법의 결합에 가깝다.
- 제어신호가 sign() 불연속을 포함하며, Fig.6-7에서 토크가 ~10^5 N·m 수준으로 매우 큼. "continuous control"이라 서술하나 채터링/포화 영향에 대한 정량 분석이 없다.
- 시뮬레이션이 단일 시나리오(고정 외란, T=5/10s 두 케이스)뿐이며 Monte Carlo·외부 baseline 비교가 전무하여 강건성 주장에 대한 실증 뒷받침이 약하다.
- 액추에이터 포화·측정잡음·이산화 등 실제 구현 요소가 전혀 고려되지 않았다.

**4. 시뮬/실험.** RLV 관성행렬(Ixx=434270, Iyy=961220, Izz=1131541, Ixz=17880 kg·m^2) 기반 수치 시뮬(sim). matched ΔM=[0.3sin2t;0.3cos2t;0.03sin2t]×10^5, mismatched Δω=[0.03sin2t;0.03cos2t;0.03sin2t]. 명령 Θref=[0.05sin t;0.04cos t;0.03sin t] rad. T=5s와 T=10s 두 케이스에서 추종오차가 규정 상·하한(±0.005 정상상태) 내에 confine되고 T 시점에 수렴 확인. 외부 baseline·MC 없음.

**5. Contribution(delta).** 출판년(2021) 기준 선행 PPC/RLV 자세제어 대비 delta: (1) 외란 상한이 미지인 상황에서도 규정 성능 영역이 외란 상한에 독립적으로 설계됨, (2) matched·mismatched 외란을 동시 처리, (3) predefined-time(임의 지정 T) 수렴을 보장. 다만 PPC·backstepping 자체는 기존 확립 기법으로, 신규성은 방법론 자체보다 mismatched·상한미지·predefined-time 결합에 있다.

**6. Limitation·Future work.** 저자: 초기조건 미지 시 대안 성능함수(M1,M2 기반) 제시로 일부 완화. 내가 보는 한계: 단일 시나리오·MC 부재로 실증이 빈약; 토크 크기·포화·잡음 등 실제 구현 제약 미고려; sign() 불연속의 실용성 분석 부재; 외부 SOTA 대비 정량 비교 없음. 향후 HIL/액추에이터 동역학 포함 검증, 다중 외란·MC 통계, 채터링 완화가 필요.

**평가:** 69/100 (good) · 한줄평: mismatched·상한미지 외란에 대한 predefined-time PPC를 엄밀히 증명한 견실한 core-G&C 자세제어 논문이나, 단일 시나리오·외부 baseline/MC 부재로 실증 깊이가 부족하다.

---

## (2022, ISA Transactions, Performance improvement-oriented reentry attitude control for reusable launch vehicles with overload constraint: Gu et al.)

**1. 핵심 아이디어.** 재사용발사체(RLV) 재진입 자세제어에서 과부하(normal overload) 제약을 성능함수(funnel) 경계에 직접 내장하여 전 비행구간에서 제약 위반이 없도록 보장하는 funnel control(FC) 기법을 제안한다. 또한 전통 FC 대비 과도응답(transient) 성능을 개선하기 위해, 이벤트 트리거 발상에 기반한 스위칭/구간별(piecewise) 비매끄러운 성능경계를 도입해 수렴속도를 적응적으로 조절한다. 가상입력으로 2차 시스템을 상대차수 1 시스템으로 변환하여 backstepping 없이 제어기를 설계하고, 공력타(rudder)와 RCS 간 모멘트 분배 전략까지 포함한다.

**2. 알고리즘.** 자세모델 Ω̇=f1+g1ω+d1, ω̇=f2+g2Uc+d2를 가상입력 x2=F1(x1)으로 변환해 ẋ1=x2+d̄1, ẋ2=F2+b(x1)Uc+d̄2 (상대차수 1)로 만든다. 추종오차 e1=x1−x1d, e2=u(x2−ẋ1d), 가상출력 s=e1+k2e2를 정의하고 제어법칙 Uc=(1/u²)b⁻¹(x1)k2⁻¹(−e2+uυ), υi=−(1/(ψi−|si|))si로 둔다. 성능함수는 ψi=e^(−li t)(ψi,max−ψi,∞)+ψi,∞이며, ψi,max는 과부하/초기오차 상한(Eq.24-26)으로 설정해 α≤αn, β≤βn 보장. 핵심 신규성은 구간별 경계 Eq.(27)와 가변 수렴율 li(t)=ε·li (조건 ε|ei|<ψi일 때) 스위칭. 모멘트 분배 kδ=√((q̄−q̄1)/(q̄2−q̄1)), kRCS=1−kδ.

**3. 비판적 검토.**
- 신규성의 본질은 funnel control(특히 [13,18])에 비매끄러운 스위칭 성능경계를 추가한 점으로, 과부하 제약의 funnel 내장 역시 진폭제약으로의 환산(Eq.8-10)이라는 비교적 직접적 처리다. 방법론적 골격(ISS 기반 안정성, K-function, Lipschitz funnel 논증)은 [23,24,27]에서 import한 표준 구조에 의존한다.
- 안정성 증명은 자체 전개가 충실하나, 스위칭 시점 tj가 사전 미지라는 점에 대한 형식적 처리(스위칭 횟수 유한성, Zeno 배제)가 Remark 수준에 머문다.
- 상수 스케일링 u의 선택이 초기조건 funnel 진입과 제어토크 크기를 동시에 좌우하는데(Remark 2,5), 이에 대한 체계적 설계지침이 부족하다.
- baseline이 traditional funnel [13] 단일 비교이며 self-implemented이다. SMC/ADRC 등 동시대 RLV 자세제어 기법과의 직접 비교가 없어 우월성 주장이 funnel 계열 내부로 제한된다.

**4. 시뮬/실험.** 단일 RLV(m=7500kg, 6.5Ma, 70km), 20초 재진입. (i) 공칭 비교(무외란): 제안 vs traditional funnel [13] — 추종오차가 funnel 내 수렴, 오버슈트·수렴율 개선. IAE 최대 63.08%, ITAE 52.46%, ISC(에너지) 49.60% 감소(sim). (ii) 강건성 Case 3: 측정잡음(공분산 1e-4)+풍교란(펄스+다주파 정현파, d2=10d1)에서 funnel 유지. (iii) 300-run 몬테카를로: 공력계수 ±30% 섭동 — 전 시행에서 오차가 funnel 내 유지(Fig.16,17). 모두 sim. RCS는 PWM on/off로 모델링, q̄=200Pa 임계로 rudder/RCS 전환.

**5. Contribution(delta).** 출판년(2022) 기준 SOTA인 RLV/극초음속 funnel control([13,18])과 비교한 delta는 (1) 과부하 제약을 성능함수 경계에 직접 내장해 전 구간 위반 없음 보장, (2) 이벤트 트리거 발상의 비매끄러운 스위칭 성능경계로 가변 수렴율 → 과도성능 개선, (3) 가상입력에 의한 상대차수-1 변환으로 backstepping 구조 단순화, (4) q̄ 기반 rudder–RCS 모멘트 분배 통합. 개별 요소는 알려진 기법의 결합이나 과부하-funnel 통합과 스위칭 경계는 명확한 점증적 진보.

**6. Limitation·Future work.** 저자: 통합유도제어(IGC)로의 확장을 future work로 명시. 내가 보는 한계: 실기/HIL 검증 부재(전부 sim), 스위칭 횟수·Zeno에 대한 형식보장 미흡, 단일 vehicle·단일 시나리오군, baseline이 funnel 계열 1종으로 제한, u·li·ε 등 파라미터 튜닝의 체계성 부족, 온보드 실시간성·연산복잡도 분석 없음.

**평가:** 67/100 (good) · 한줄평: RLV 재진입 자세제어 도메인에 정확히 부합하는 견실한 funnel-control 변형으로 과부하 제약 통합과 스위칭 경계가 실용적 가치는 있으나, 신규성이 점증적이고 검증이 단일 baseline·sim에 머물러 영향력은 niche 수준이다.

---

## (2023, Acta Astronautica, Finite time fractional order ISMC with time delay estimation for re-entry phase of RLV with enhanced chattering suppression: Sarkar et al.)

**1. 핵심 아이디어.** RLV 재진입 단계의 자세 제어를 위해 분수계 적분 슬라이딩 모드 제어(FOISMC)와 시간지연추정(TDE)을 결합한 FOISMC-TDC 기법을 제안한다. TDE가 직전 시각의 입출력 정보로 모델 불확실성·외란을 추정하여 보상하므로, 스위칭 이득에 주로 의존하지 않고도 외란 보상이 가능해져 입력의 고주파 채터링을 크게 줄인다. 적분 슬라이딩 면을 사용해 도달 위상(reaching phase)을 제거하여 초기 시점부터 강건성을 확보하고, 분수계 항으로 유한시간 실질 수렴(practical finite-time)을 달성한다.

**2. 알고리즘.** 적분 슬라이딩 면: s = Θ̃̇ + ∫₀ᵗ (Λ₁⌊Θ̃⌋^α₂ + Λ₂⌊Θ̃̇⌋^{α₂/α₁}) dτ (식 8), α₂∈(0,1), 2α₁−1=α₂. 역동역학 기반 제어 모멘트: M(t)=J̄(t)u(t) − F̂₁(t) (식 11). 보조 피드백 제어법칙: u(t) = −Λ₁⌊Θ̃⌋^α₂ − Λ₂⌊Θ̃̇⌋^{α₂/α₁} − k₁⌊s⌋^α₃ − k₂s (식 12), 0<α₃<1. TDE 추정: F̂₁(t) ≈ F̂₁(t−ρ) = J̄(t−ρ)Θ̃̈(t−ρ) − M(t−ρ) (식 15). 최종 제어 모멘트는 피드백 FOISMC 항과 F₁ 추정 항(인공 시간지연 제어)으로 구성된다(식 16). Lyapunov V₁=½sᵀs 와 동차성(homogeneity) 이론으로 닫힌루프의 실질 유한시간 안정성을 증명.

**3. 비판적 검토.**
- 핵심 안정성 기계장치(Def 1-2, Lemma 1-3, Theorem 1)가 대부분 [23,32-35]에서 인용·수입된 것이며, Theorem 1의 증명은 "[23]에 있음"으로 처리된다. 자체 기여는 이들을 조립해 Theorem 2-3을 구성한 부분에 한정되어 이론적 독창성이 약하다.
- 신규성이 기성 구성요소의 결합에 머문다. ISMC+TDE는 이미 [26], RLV 재진입의 분수계 SMC 및 TDE 기반 강건제어는 동일 저자군의 [22-24]에 존재했다. 본 논문은 적분 슬라이딩 면에 분수계 항을 입힌 변형 + TDE 결합으로, delta가 점진적이다.
- 사용자 정의 행렬 J̄(t)=0.5I·J₀·R⁻¹(t)의 선택 근거가 "이후 논의"로 느슨하게 처리되고, 식 17(‖J̄_Θ⁻¹ J̄ − I‖<1)이라는 안정 조건의 실제 보장 메커니즘이 충분히 다뤄지지 않는다.
- 검증이 단일 X-33 공칭 궤적 1건에 그친다. 몬테카를로나 파라미터 스윕이 없어, 불확실성·외란 강건성 주장의 통계적 뒷받침이 부족하다.
- 분수계 ⌊·⌋^α 연산의 온보드 연산 비용·실시간 타이밍에 대한 정량 분석이 없다(관련 연구 [29]에서 분수계 미분기가 차수를 높여 복잡도를 키운다고 본문이 직접 지적하면서도 본 기법의 비용은 측정하지 않음).

**4. 시뮬/실험.** X-33 RLV 모델, 재진입 약 2000 s, GPOPS-II로 생성한 참조궤적(식 38, [38] 기반). 외부 baseline 2종과 비교: QC-HO-SMC [10], PD-TDC [23] (모두 외부 기법) (sim). 정량지표: 잔류 한계 FOISMC-TDC 5.85×10⁻⁶ vs QC-HO-SMC 1.0×10⁻³ vs PD-TDC 1.0×10⁻⁴; 정착시간 3.76 s 수준; 오차지표(Table 3) ISE 0.0765, IAE 0.8241, ITSE 0.8527, IATE 560.94로 QC-HO-SMC(2.08/3.30/3.42/820)보다 우수, PD-TDC(0.056/0.95/0.22/625)와는 혼재. 입력 변동량 TV로 채터링 정량화: 제안기법이 QC-HO-SMC보다 채터링 현저히 감소(단 signum을 안 쓰는 PD-TDC가 TV 최소). 열률·동압·하중계수 제약 만족 확인(Fig 2). MC·통계 부재.

**5. Contribution(delta).** 출판년(2023) 기준, RLV 재진입 자세제어에서 (i) 적분 슬라이딩 면으로 도달 위상을 제거해 초기시점부터 강건성 확보, (ii) TDE로 외란 상계의 사전가정을 완화하고 스위칭 이득을 낮춰 채터링 억제, (iii) 분수계 항으로 정확도·수렴속도 개선을 동시에 노린 결합이 delta다. 다만 각 구성요소(ISMC, TDE, 분수계 SMC)가 모두 선행에 존재하므로, 본질적으로는 기존 기법들의 통합·적용형 기여이며 새로운 G&C 방법론을 여는 수준은 아니다.

**6. Limitation·Future work.** 저자: 이득(k₁,k₂,Λ,샘플링 ρ)의 자동 조정(self-tuning/적응법칙) 부재를 한계로 인정, 적응 메커니즘과 슬라이딩 면 설계 추가 탐구를 향후 과제로 제시. 잔류 한계 Δ가 추정오차 ξ와 이득에 의존함을 명시. 내가 보는 한계: 단일 시나리오·MC 부재로 강건성 주장의 통계적 근거 미흡, 분수계 연산의 온보드 실시간성 미검증, 안정성 이론의 자체 기여가 인용 기계장치 조립에 가까움, 동일 저자군 선행작 대비 점진적 신규성.

**평가:** 61/100 (fair) · 한줄평: X-33 재진입 자세제어에 적분 SMC+TDE+분수계를 결합해 채터링을 정량적으로 줄인 견실한 in-domain G&C 논문이나, 신규성이 기성 구성요소 통합에 머물고 단일 공칭 시나리오·이론의 인용 의존이 상승을 제약한다.

---

## (2014, Acta Astronautica, Thrust-vector control of a three-axis stabilized upper-stage rocket with fuel slosh dynamics: Hervas·Reyhanoglu)

**1. 핵심 아이디어.** 부분적으로 채워진 단일 추진제 탱크를 가진 3축 안정화 상단 로켓(상단 AVUM/Vega)의 추력벡터제어(TVC) 문제를 다룬다. 슬로싱 추진제를 다중 질량-스프링 시스템으로 모델링하여 주요 슬로시 모드를 표현하고, 강체(기저체)와 결합한 완전 비선형 3차원 운동방정식을 SE(3)-불변 라그랑지안으로 유도한다. 제어 목적은 짐벌 편향각(δ1,δ2)과 3개 토크(Mx,My,Mz)를 입력으로 병진속도벡터와 자세를 제어하면서 내부동역학인 슬로시 모드를 감쇠시키는 것이며, Lyapunov 기반 비선형 피드백 제어법칙을 설계한다.

**2. 알고리즘.** Lagrangian L=T−U, Rayleigh 소산함수 R로부터 Eq.(1)-(3)을 적용해 결합 운동방정식 Eq.(5)-(13)을 얻는다. 입력변환으로 원래 입력(δ1,δ2,Mx,My,Mz)을 새 입력 (u1,...,u5)으로 사상하여 축소계 Eq.(15)-(21)을 만든다. 축차류 변수에 대한 후보 Lyapunov 함수 V=Σ(ri/2)(...)로 양정부호를 구성하고, V̇=−2r9ΣζiΩiξ̇i²−2r10ΣζiΩiη̇i²−l1[...]²−l2[...]²−l3θ̇x²−l4θ̇y²−l5θ̇z²≤0가 되도록 피드백법칙 u1=−l1[r1vy−r9Σ(ξ̇i−hiωz−ηiωx)], u2 유사형, [u3;u4;u5]=−F⁻¹[g1+l3θ̇x; g2+l4θ̇y; g3+l5θ̇z]을 설계한다(F는 가역 가정). LaSalle 원리로 원점의 점근안정성을 증명한다.

**3. 비판적 검토.**
- 슬로시 모드를 질량-스프링으로 근사하는 것은 표준적 기계해석(mechanical analogy)이며 본 모델의 신규성은 3D 결합 동역학 유도와 Lyapunov 합성의 결합에 한정된다. Bond number>1000인 고가속 영역에서만 모델이 유효(자체 인정)하며, 저가속/계면장력 지배 영역으로의 일반화는 없다.
- 제어법칙은 nominal 모델에 대해서만 설계되었고 파라미터 불확실성, 슬로시 주파수/감쇠 추정오차, 외란에 대한 robustness 분석이나 보장이 전혀 없다. ki 일정 가정 등 단순화 가정이 다수.
- F의 가역성을 모든 t에 대해 가정하나, 짐벌 편향이 큰 영역(Fig.7에서 δ2가 30deg 초과)에서 가역성 또는 특이점 회피에 대한 분석이 없다. 입력변환의 well-posedness가 시뮬레이션 영역에서만 암묵적으로 확인됨.
- 단일 시나리오(단일 초기조건), MC 없음, 외부 baseline 없음. 무제어 케이스(Fig.9-10)와의 정성비교만 제시.

**4. 시뮬/실험.** AVUM 상단(m=975kg, F=2450N, tb=600s) 단일 시나리오 (sim). 초기조건 v0=[3000,75,25]m/s, θ0=[5,2,−5]deg, s0=[0.15,0.05,−0.1,0.15]. 2개 슬로시 모드(m1=89kg, m2=2.7kg). 결과: 횡속도·자세각·슬로시 좌표(ξ,η)가 모두 0으로 점근수렴, vx는 1.720m/s²로 가속하여 증가. 무제어 케이스에서는 슬로시가 발산(불안정). MC 통계나 외부 baseline 비교는 없음.

**5. Contribution(delta).** 저자 자신의 선행 연구([6]-[10], 평면 기동 및 다중 질량-스프링/진자 모델)를 3차원 완전 비선형으로 확장한 것이 핵심 delta다. 출판년(2014) 기준, 선행 슬로시 제어는 주로 선형화 기반([11]-[17])으로 횡진동 여기 문제를 가졌는데, 본 논문은 선형화 없이 결합 비선형 동역학에 직접 Lyapunov 합성을 적용해 underactuated 슬로시 감쇠를 점근안정성과 함께 보장한 점이 진전. 다만 방법론 자체(Lyapunov+LaSalle on mass-spring slosh)는 저자 그룹 선행작의 차원 확장 성격이 강해 incremental에 가깝다.

**6. Limitation·Future work.** 저자: 슬로시 파라미터가 축가속에 크게 영향받지 않는다는 가정(완화가 유망한 방향), 다중 추진제 탱크 문제로의 확장. 내가 보는 한계: robustness/불확실성 보장 부재, 단일 시나리오·M#C·외부 baseline 부재로 실증 강도 약함, F⁻¹ 가역성/특이점 분석 미비, 실시간/온보드 구현성(연산복잡도, 토크 포화) 미분석, 큰 짐벌각에서의 모델 타당성 미검증.

**평가:** 61/100 (fair) · 한줄평: 도메인 정합성 높은(core-GNC, 상단 로켓 TVC+슬로시) 견실한 비선형 제어 저널 논문이나, 자체 선행작의 3D 확장 성격·robustness 및 실증(MC/baseline) 약함으로 in-domain 솔리드 하한대에 위치.

---

## (2024, CJA, Compensated acceleration feedback based active disturbance rejection control for launch vehicles: Zhang et al.)

**1. 핵심 아이디어.** 고고도 강풍/고동압 구간에서 발사체의 자세추종(attitude tracking)과 하중경감(load relief)을 동시에 달성하기 위한 보상 가속도 피드백 기반 ADRC(CAF-ADRC) 프레임워크를 제안한다. 피치(PA) 부시스템의 총외란을 확장상태관측기(ESO)로 추정·보상하여 자세추종을 확보하고, 가속도계 측정에 포함된 엔진추력에 의한 법선가속도를 추력곡선 추정과 ESO 외란추정으로 보상함으로써 하중경감 효과를 강화한다. 나아가 폐루프 전체의 정량적 성능(추정오차·추종오차 한계)과 모듈별(ESO/PD/CAF) 튜닝법칙을 엄밀하게 유도한다.

**2. 알고리즘.** PA 부시스템을 상태공간 (3)으로 모델링. 제어법칙은 CAF-ADRC: delta_phi = -(k_phi,p/b3)Delta_phi - (k_phi,d/b3)Delta_phidot - (kz/b3)n_hat_y - (1/b3)z_{f_phi} (식 13). 여기서 보상가속도 n_hat_y = y3 - k3_hat*delta_phi - la*b3*delta_phi - la*z_{f_phi} = k2*Delta_alpha + k3tilde*delta_phi + la*(f_phi - z_{f_phi}) (식 12). 2차 ESO: [zdot; zdot_f] = Abar[z; z_f] + Bbar*delta_phi + Lbar(y2 - z_phi), Lbar=[2*omega_o, omega_o^2]^T (식 11). 비교용 CAF-PD(식 14)는 z_{f_phi}=0인 특수경우. 안정성: |kz| < epsilon (Lemma 1, Routh-Hurwitz). Theorem 1: |f_phi - z_{f_phi}| <= gamma1(e^{-gamma2*omega_o(t-t0)} + 1/omega_o), 추종오차도 omega_o, omega_c 증대로 감소.

**3. 비판적 검토.**
- Assumption 1(외란 및 그 도함수 유계)은 표준적이나 강한 가정이며, ESO 대역폭 omega_o를 키워 추정오차를 줄이는 구조는 측정잡음·구동기 대역에 취약하나 잡음 영향 분석은 부재하다.
- AOA 측정이 없어 f_alpha는 추정·보상 불가(Remark 13 자인). 따라서 AOA 부시스템 하중경감은 kz 게인 조정에 의한 간접효과에 국한되며 핵심 약점이다.
- baseline 중 CAF-PD/순수 ADRC는 사실상 자기 ablation이며, 외부 baseline은 structured H∞ 하나뿐이고 동일 튜닝 노력 하의 공정비교 여부가 불명확하다.
- 정량 성능정리(Theorem 1, Prop 1-4)는 ESO/소이득정리/Routh-Hurwitz 등 표준기법의 조합으로, 발사체 ADRC에 대한 엄밀한 폐루프 분석이 '미유도'였다는 점이 기여이나 분석기법 자체의 새로움은 제한적이다.

**4. 시뮬/실험.** 14m급 발사체(전비/공비 질량 10751/3557kg, 추력 240kN, US1976 대기, CFD 공력), 6-20km 고동압 구간. (sim) 50개 풍속장 하 자세/하중 비교 + 몬테카를로(3개 풍속장 x 각 100회 = 300회, 공력/질량/관성/질심 ±5~10% 분산, Table 5). 정량결과: 고동압 구간 하중경감비 약 19~28%(Table 6 최대 28.01%, 최소 10.88%), AOA 약 0.6° 감소. baseline: 순수 ADRC, CAF-PD(ablation), structured H∞(Navarro-Tapia 등, ref 28 외부). CAF-ADRC가 CAF-PD 대비 PA편차/공력하중 모두 작고, t>32s 이후 H∞ 대비 하중경감 우수. (real)/(HIL) 없음.

**5. Contribution(delta).** 출판년(2024) 기준 SOTA 대비 delta: (1) 가속도계 측정의 엔진추력 성분을 추력곡선 추정 + ESO 총외란 추정으로 '보상'하는 CAF-ADRC 구조(기존 가속도 피드백 ADRC는 보상 없이 법선가속도를 그대로 도입), (2) ADRC+가속도 피드백 폐루프 전체의 정량적 성능·튜닝 분석을 엄밀히 유도(기존 ADRC 발사체 연구가 다루지 않던 부분). 발사체 ADRC 자체는 선행(refs 30,35-41) 존재하므로 개념적 신규성보다는 보상기법 + 엄밀분석의 결합이 핵심 delta.

**6. Limitation·Future work.** 저자: AOA 미측정으로 f_alpha 보상 불가, 하중경감은 고동압 구간에 한정 도입. 내가 보는 한계: 측정잡음/구동기 동역학/시간지연 미고려, omega_o 증대의 실제 한계 미분석, 실비행/HIL 검증 부재, 외부 baseline 빈약, 횡축 결합(yaw/roll) 무시한 PA 평면 단일채널 분석. Future: 잡음·구동기 제약 하 강건 ESO 대역폭 설계, 다채널 결합, HIL/비행 검증.

**평가:** 76/100 (strong) · 한줄평: 발사체 고동압 하중경감을 위한 보상 가속도 피드백 ADRC를 엄밀한 폐루프 정량분석·튜닝법칙과 함께 제시한 도메인-정합도 높은 견실한 G&C 연구이나, 외부 baseline과 실비행/HIL 검증의 부재가 아쉽다.

---

## (2024, Neurocomputing, A reinforcement learning approach for adaptive tracking control of a reusable rocket model in a landing scenario: Costa et al.)

**1. 핵심 아이디어.** Falcon 9급 재사용 발사체(RLV)의 하강/착륙 단계 제어를 대상으로, 심층 신경망을 쓰지 않는 저복잡도 강화학습(Q-Learning) 기반 적응 최적제어를 적용한다. 선형-이차(LQ) 구조를 활용해 quality function Q를 상태·입력의 이차 기저함수의 선형회귀로 표현하고, 그 가중치를 온라인 최소제곱(DF-RLS)으로 추정한다. 자세(yaw, 짐벌)와 위치(고도·종거리)를 분리한 cascade SISO 구조에서 LQG/PID를 RL로 교체했을 때의 성능을 비교한다.

**2. 알고리즘.** 무한구간 이차비용 V(x)=Σ γ^{i-k}[(x_i-r_x)'M(x_i-r_x)+R u_i^2], Bellman식 V(x_k)=r(x_k,u_k)+γV(x_{k+1}). quality function Q(x_k,u_k)=r+γQ(x_{k+1},u(x_{k+1})), Q(x,u)=W^T φ(x,u) (φ=z⊗z, z=[x';u]'). 선형회귀 W^T(φ(z_k)-γφ(z_{k+1}))=r(x_k,u_k)를 DF-RLS로 추정 후 피드백 u_k=F(r_x-x_k)+η_k(dither). Policy Iteration(평가→F 갱신) 반복. Youla-Kucera 구성에서는 LQG 내부루프 + 칼만 잔차 ε에 대한 Q-RL 보정 s=-K_{QL}x_{QL}을 cascade로 결합. 자세: δ_{yinc}=-K_Y x̂_Y(k); 고도/종거리: T_inc=-K_{QL}x_{QL}, Y_inc=-(K_{QL}x_{QL}+K_I I(y_I)).

**3. 비판적 검토.**
- 이론 골격(선형계 LQR용 Q-learning, Youla 파라미터화)이 전부 인용([13],[38])이고 자체 증명·안정성/수렴 보장이 없다. RL 학습이 45s/30s 짧은 구간에만 작동하고 이후 고정 게인으로 운용되어 '적응'의 실질이 제한적이다.
- 검증이 평면(planar) 운동, 단일 초기조건(Table 1), 분리된 SISO 저차 선형모델(2/3/4차)로 단순화돼 있고 기준궤적은 오프라인 선계산된다. 6-DOF 결합 동역학에서의 일반성이 입증되지 않는다.
- 비교 baseline(LQG/PID)이 모두 저자 자체 설계이며 외부 기준이 없다. Monte Carlo 분산 분석이 없어 정량 우위(특히 종거리 오차 개선)의 통계적 유의성을 판단할 수 없다.
- '저복잡도/온보드 적합' 주장에 비해 연산시간·복잡도 수치나 임베디드 실증이 없다. DF-RLS의 정보행렬 전파 비용((n_x+n_u)^2)은 언급되나 정량 분석은 부재.

**4. 시뮬/실험.** 평면 재진입 시뮬레이션(sim only), 단일 초기상태(고도≈50 km급, v=[-700,800,0] m/s, m=45000 kg). 풍계(5-20 km 또는 5-10 km 수평 돌풍) + 액추에이터 상수 바이어스(고장 모사) 시나리오. 자세제어 yaw 추종오차 max|Y_err|≈1.04 deg(외란 존재). 위치오차: LQ/PID에서 max|x_I|=0.18 m, max|y_I|=5.6 m, 최종 x_{I_f}=0.0003 m, y_{I_f}=0.57 m. QL/PID 최종 y_{I_f}=0.58 m, LQ/QL(종거리 PID→QL 교체) 최종 y_{I_f}=0.088 m로 개선(Table 2). HIL/real 없음, MC 없음, 외부 baseline 없음.

**5. Contribution(delta).** 출판년(2024) 기준, 다수의 RLV-RL 연구가 심층 RL(6-DOF, 다층 NN)을 쓰는 데 비해, 본 연구는 선형-이차 구조를 활용한 저복잡도 Q-learning 적응제어를 RLV 착륙 cascade 구조에 적용하고 Youla-Kucera 안정화 집합 내로 탐색을 제한하는 구성을 제시한 점이 delta다. 다만 핵심 기법은 기존 LQ-RL/Youla의 조합·재적용이며, 종거리 루프 PID→QL 교체로 정착오차를 개선했음을 보인 수준의 점진적 기여다.

**6. Limitation·Future work.** 저자: 학습 가용시간이 짧고 외란이 주로 dither에 의존, 비용가중치 선택으로 내·외 루프 시간척도 분리를 수동 보장해야 함. 추가로(평가자): 안정성/수렴 보장 부재, 평면·단일조건·자체 baseline·MC 부재로 통계적 검증이 약하고, 6-DOF 결합·연산 실시간성·외부 SOTA(예: 볼록화 G&C, 심층 RL) 대비 직접 비교가 빠져 있다. 오픈소스 시뮬레이터 공개가 향후 과제로 언급됨.

**평가:** 56/100 (fair) · 한줄평: 도메인 적합한 RLV 착륙 제어 문제에 저복잡도 LQ-RL을 합리적으로 적용했으나, 자체 증명·안정성 보장·외부 baseline·MC가 없고 평면 단일조건 검증에 그쳐 in-domain G&C 저널 기준 평균선 약간 아래에 위치한다.

---

## (2023, EAAI, Optimal trajectory-tracking guidance for reusable launch vehicle based on adaptive dynamic programming: Xueyun et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV) 재진입 단계의 최적 궤적추종 유도를 신경망 기반 적응동적계획법(ADP)으로 구성한다. 먼저 Gauss Pseudo-spectral Method(GPM)로 흐름각(AOA·BA)의 진폭뿐 아니라 각속도·각가속도까지 포화제약에 넣어 기준궤적과 정상상태 제어를 오프라인 생성하고, 단일 critic ADP 컨트롤러로 정규화 추종오차에 대한 최적 피드백 제어를 학습해 정상상태 제어와 합쳐 추종한다. 더불어 매 시간스텝마다 가중치를 1회만 갱신하는 "innovative weight iteration"으로 학습연산을 줄인다.

**2. 알고리즘.** 6상태(반경·위도·경도·속도·경로각·방위각) 정규화 점질량 모델 ẋ#=f(x#,u)+d, 제어 u=[α,σ]. 효용 r=Q(e)+R(u_e), Q=eᵀQ₀e, R은 포화 반영을 위해 R(u_e)=Σ∫λᵢ tanh⁻¹(vᵢ/λᵢ)R₀dvᵢ. HJB H(e,u_e*,V*)=0 에서 최적 피드백 u_e*=−½λ tanh((1/λ)R₀⁻¹∇f_u ∇V*). Critic V̂(e)=Ŵ_cᵀφ_c(e), 회귀행렬 A,B (Ȧ=−lA+ppᵀ, Ḃ=−lB+pr), 보조벡터 M=AŴ_c*+B에 대해 가중치법칙 Ŵ̇_c*=−K_W M. Lyapunov V_Ly=½W̃_cᵀK_W⁻¹W̃_c 로 ε_HJB=0이면 지수수렴, ≠0이면 컴팩트셋 수렴 증명.

**3. 비판적 검토.**
- ADP 핵심 기계장치(linear-in-parameter critic, HJB→tanh 포화 제어, A/B 회귀 보조벡터)는 Vamvoudakis & Lewis, Liu et al. 등에서 표준 차용이며, 유도루프 전체 안정성은 Bu & Qi(2022)로 외주되어 본 논문의 고유 증명은 weight-iteration 수렴 부분에 한정된다.
- 검증이 단일 재진입 시나리오 1회 결정론적 실행에 그치고 Monte Carlo가 없다. 불확실성은 초기오차+공력계수 −5%+합력 8% 한 세트만 적용되어 강건성 주장이 통계적으로 뒷받침되지 않는다.
- baseline이 모두 자체 구현(gradient-descent critic, 자사 LQR)으로 외부 SOTA 비교가 부재하다. "95% 연산 감소, 종단오차 75% 감소"는 자기참조 상대수치다.
- 단일 시나리오라 PE(persistent excitation) 조건 충족 여부, 일반화 가능성에 대한 논의가 약하다.

**4. 시뮬/실험.** 3-DOF 재진입 1700s, 사거리 >5500km, 초기 고도 60km→종단 24km, 종단속도 760m/s·경로각 −2°. 초기오차: 위치 각축 1200m, 속도 10m/s, 경로각·방위각 0.12°, 공력계수 −5%, 합력 8% 외란. 제안법 종단 수평위치오차 2.06km(자체 정상상태 제어만 시 35km, gradient-descent ADP 3.75km, LQR 9.5km). 연산: 제안법 5.66s/1750 iter, gradient-descent 112.89s/1,748,801 iter(95% 감소), LQR 176.05s. 모두 (sim), HIL·실비행 없음.

**5. Contribution(delta).** 출판년(2023) 기준, GPM 궤적생성에 흐름각 각속도·각가속도 포화제약까지 모멘트 입력으로 추가한 점과, 단일 critic ADP를 RLV 3D 궤적추종에 적용하며 actor 없이 매 스텝 1회 가중치 갱신하는 weight-iteration을 결합한 점이 delta. 다만 단일 critic ADP, GPM, tanh 포화제어는 모두 기존 확립 기법이라 통합·튜닝 수준의 점진적 기여에 가깝다.

**6. Limitation·Future work.** 저자: 오프라인 학습 불요·온라인 적응을 강점으로 제시. 내가 보는 한계: (i) MC·외부 baseline 부재로 강건성/우월성 입증이 약함, (ii) 안정성 증명이 weight-iteration에 국한되고 폐루프 유도 안정성은 외주, (iii) 데이터·코드 비공개("no permission to share"), (iv) 단일 임무 프로파일로 일반화·실시간 임베디드 검증 미흡. Future: 다양한 임무·외란에 대한 MC, 임베디드 타이밍 실측, 외부 강건유도 기법과의 비교가 필요.

**평가:** 61/100 (fair) · 한줄평: GPM+단일critic ADP의 견실한 통합형 in-domain G&C 논문이나, 단일 시나리오·자기참조 baseline·MC 부재로 영향력은 제한적이다.

---

## (2025, Acta Astronautica, Propulsive landing of launchers' first stages with Deep Reinforcement Learning: Iafrate et al.)

**1. 핵심 아이디어.** 재사용 발사체 1단의 추진 착륙(터미널 착륙) 문제를 모델프리 심층강화학습(PPO)으로 통합 유도·제어한다. 김벌 추력기를 직접 제어하는 단일 정책으로 3-DOF/6-DOF 환경에서 연료최적 착륙을 학습하며, Falcon 9 비행 데이터 기반의 현실적 초기조건과 미모델링 동역학·외란에 대한 강건성까지 평가한다. 기여의 핵심은 보상함수 설계(target-velocity, 2단계 ablated, target-acceleration)와 검증된 오픈소스 시뮬 환경이다.

**2. 알고리즘.** 강체 6-DOF 동역학(병진: ṙ=v, v̇=F/m+g, ṁ=-||T||/(Isp·g0); 회전: 쿼터니언 q̇=½Ωq, ω̇=J⁻¹(M-ω×Jω)). 항력만 고려하는 spherical aero 모델(ISA 대기). 제어=[δy, δz, ||T||], PPO clipped surrogate L=E[min(rt·Â, clip(rt,1-ε,1+ε)·Â)]. 보상은 가중 합: r=α||v-v_targ||+β||T||+γ(자세한계)+κ(터미널 보너스)+η. target velocity는 LOS 기반 지수감쇠 waypoint 휴리스틱. 6-DOF에서는 에너지최적 폐형해 a_targ=-6r/t_go²-4v/t_go-g (t_go는 quartic 해)를 추종하는 target-acceleration 보상으로 전환, 추력 포화로 클리핑.

**3. 비판적 검토.**
- 형식적 보장 부재. 안정성·최적성·수렴에 대한 이론적 증명이 없고 블랙박스 신경망 정책이라 explainability가 낮음(저자도 인정). 유인 착륙·지구 재진입 인증 맥락에서 한계.
- SCVX(successive convexification) 대비 RL은 연료를 3-DOF에서 약 25%, 6-DOF에서 약 60% 더 소모. 즉 convex 최적해에 비해 명확히 비효율적이며, 이는 자세제어 필요성과 가속도추종 형태에 기인.
- 보상함수가 다수의 수작업 튜닝 계수(α,β,γ,δ,κ,η,τ 등)에 의존하고 단계적 ablation·전환을 요구. 이식성·일반화가 보상공학에 강하게 종속되어 재현·확장 시 부담.
- 비교 baseline이 오프라인 개곡선 최적해(SCVX/point-mass)로, 폐루프 추종제어기 대비 공정 비교라기보다 도달가능 하한 제시 성격. terminal 위치오차(6-DOF μ=10~25 m)는 핀포인트 기준으로 다소 큼.

**4. 시뮬/실험.** 3-DOF/6-DOF, 단순화·현실적(Falcon 9 비행 데이터) 초기조건. Deimos FES 시뮬레이터로 환경 검증(오차 수 오더 작음). MC 100 runs. 6-DOF 현실 조건 터미널 통계: 위치 μ=10.0 m(σ=4.3), 속도 9.42 m/s(σ=2.3), 자세 4.7°, 사용연료 4219 kg, 종단속도각 18.9°(sim). 연료비교(table 14): RL 4250 kg vs 3-DOF 최적 3545 kg vs SCVX 6-DOF 7525 kg. 강건성(Section 6): CoM±3%, 유연모드, 관성±1%, 2차 TVC 액추에이터, 추력지연 2 s, 추력 정렬오차±0.5°, 풍돌풍 15 m/s, HWM14 풍층 — outlier는 100런 중 항목별 0~2건. baseline: SCVX MPC + point-mass 에너지최적해.

**5. Contribution(delta).** 출판년(2025) SOTA 대비: 외계행성·저추력·이미지기반에 치우친 기존 RL 착륙 연구와 달리, 김벌 추력 발사체 1단의 6-DOF 대기 착륙을 모델프리 RL로 직접 제어하고 현실적 초기조건+미모델링 동역학·외란 강건성을 체계적으로 분석한 점. 2단계 ablated 보상으로 성공착륙→연료최적을 분리 학습하는 접근과, 검증된 확장가능 오픈소스 환경 제공이 delta. Gaudet/Furfaro ZEM-ZEV 계열 대비 명확한 진전이나, RL-착륙이라는 확립된 흐름 내 확장.

**6. Limitation·Future work.** 저자: convex 대비 연료 비효율(~25/60% 초과), explainability 낮음, 도메인 지식 기반 보상공학 부담, 6-DOF 수렴에 ~24 h 학습. 추가로 내가 보는 한계: 이론적 안정성·재귀적 실현가능성 보장 부재, terminal 위치오차가 핀포인트 기준 다소 큼, 실시간/임베디드 온보드 타이밍 분석 부재(추론 비용은 낮을 것으로 추정되나 미검증), 실제 비행/HIL 검증 없음. Future: 전체 재진입 궤적으로 확장, 보상 자동화, 온보드 실시간성 정량화.

**평가:** 67/100 (good) · 한줄평: 현실적 발사체 1단 착륙을 6-DOF RL로 통합 제어하고 강건성을 충실히 검증한 견고한 core-G&C 응용 논문이나, convex 최적 대비 연료 비효율과 형식적 보장 부재가 상한을 제한.

---

## (2024, AST, Performance-prescribed optimal neural control for hypersonic vehicles considering disturbances: An adaptive dynamic programming approach: An et al.)

**1. 핵심 아이디어.** 극초음속 비행체(winged-cone, 3-DOF 종방향)의 고도-속도 추종 제어를 위해 세 가지 기법을 결합한 최적 신경 제어 프레임워크를 제안한다. (i) 규정 성능(PPC) 함수로 추종 오차를 제약-인지(constraint-aware) 무제약 좌표로 변환하고, (ii) 저정밀/고정밀 모델을 융합한 개선된 고차 슬라이딩 모드 보상 관측기로 확률적 외란을 추정하며, (iii) actor-critic ADP(정책 반복)로 지수형 비용의 HJB를 근사 풀어 준최적 제어 정책을 얻는다. 폐루프는 SGUUB로 증명된다.

**2. 알고리즘.** 변환오차: epsilon_i = ln(zeta_i/(1-zeta_i)), zeta_i = (e_i - rho_iL)/(rho_iU - rho_iL); 어파인 오차동역학 dot epsilon = F + G*phi + D - F_ref. 지수형 비용: J_i = ∫ exp(-eta(s-t)) r_i ds, r_i = epsilon^T Q epsilon + ubar^T R ubar + 2k_u epsilon^T R ubar. 최적정책: ubar_i* = -k_u epsilon_i - (1/2) R_i^{-1} Gbar_i^T ∇J_i*. critic/actor NN 가중치 갱신은 경사하강(식 33, 35)에 피드백항(What_ic, What_ia)을 추가하여 초기 admissible control 불필요. 관측기는 Levant 미분기를 저정밀 관측기로, 식(15)의 strict-feedback 보상 시스템을 고정밀항으로 융합해 유한시간 수렴.

**3. 비판적 검토.**
- PPC+ADP 결합은 본 논문 자신이 인용한 [25,32-37]에서 이미 활발히 다뤄진 방향이라 핵심 구조의 신규성은 제한적이다. 진짜 delta는 저/고정밀 융합 관측기와 상관항을 포함한 지수형 비용이다.
- 안정성 증명이 Levant 미분기[43], Lemma 3[42], Assumption 1-4(경계 가정)에 크게 의존하며, 저자도 결론에서 SGUUB에 그쳐 더 엄밀한 UUB에 도달하지 못했음을 인정한다.
- 비교 baseline(Terminal SMC, Single critic, Actor-critic)이 모두 내부 자체 구현이며 외부 공개 방법이나 실측 baseline이 없고 Monte-Carlo 통계도 없다.
- "without high calculation consumption"을 주장하나 실행시간/복잡도/임베디드 검증이 전혀 없어 온보드 실시간성은 미입증.
- 지수형 비용, 기본 게인=1 등 설계 선택의 정당화가 약하고 단일 비행체·단일 시나리오에 국한된다.

**4. 시뮬/실험.** 전부 시뮬(sim), winged-cone 극초음속 단일 모델, dt=0.01s. 3개 케이스: (1) 무외란(고도 34→34.5km, 속도 4650→4800m/s 계단), (2) 외란(d_V, d_h 정현), (3) 강건성(30% 공력 불확실성, 10% 대기, 5% 백색잡음). 정량: Table 1-2의 IAE/ITAE에서 제안법(Actor-Critic with PPC)이 V 채널 IAE 2.21E+3, 수렴시간 0.18s로 Terminal SMC(2.12E+4, 5s)·Single critic 대비 우세, 각상태 q채널 ITAE 22.74 deg·s로 최소. Table 3-4에서 고정밀-저정밀 융합 관측기가 단일 데이터 대비 낮은 IAE/ITAE. Baseline은 자체구현 3종, MC 없음.

**5. Contribution(delta).** 출판년(2024) 기준, (i) 제약-인지 최적 신경 제어 프레임(PPC 변환을 ADP 비용에 직접 결합), (ii) 독립 actor/critic NN 갱신으로 초기 admissible control 불필요(Ref [28] 대비), (iii) 저정밀 데이터+고정밀 모델 융합 고차 슬라이딩모드 관측기로 고차 도함수 추정 정확도 개선([43,44] 대비). 다만 PPC+ADP 결합 자체는 이미 확립된 방향이라 novelty는 점진적-중간 수준.

**6. Limitation·Future work.** 저자: 전통적 PPC 함수만 사용(유한시간/유연 경계 전략 미적용), SGUUB에 그쳐 UUB 미달. 추가로 내가 보는 한계: 외부/실측 baseline 부재, MC 통계 없음, 온보드 실시간성 정량 미검증, 단일 비행체·종방향에 국한, 게인 다수의 수동 튜닝 의존.

**평가:** 67/100 (good) · 한줄평: 극초음속 추종이라는 핵심 G&C 도메인에 정면 위치하나, PPC+ADP+SMO 관측기의 견실한 조합으로 신규성은 점진적이고 baseline·실시간성 검증이 약한 양호한 in-domain 저널 논문.

---

## (2022, J. Franklin, The VEGA launcher atmospheric control problem: A case for linear parameter-varying synthesis: Navarro-Tapia et al.)

**1. 핵심 아이디어.** VEGA 발사체의 대기권 비행 단계 추력벡터제어(TVC) 문제를 게인스케줄링(GS) 대신 Linear Parameter-Varying(LPV) 합성 프레임으로 정식화한다. 산업 표준 관행(강체 제어기 설계 후 벤딩필터를 순차/반복적으로 추가)을 LPV로 옮겨, 강체 제어기와 벤딩필터를 단일 절차에서 동시에 얻을 수 있음을 보인다. 설계는 비-rate-bounded → rate-bounded(선형/2차 기저함수)로 점진적으로 복잡도를 높이며, 결과 LPV 제어기를 VEGA 산업용 고충실도 비선형 시뮬레이터에서 baseline 및 두 개의 고급 제어기(구조적 H∞, 적응증강)와 비교한다.

**2. 알고리즘.** LPV 일반화 플랜트 P(ρ)에 대해 유도 L2 노름 min_K ||T_ed(ρ)||_{L2→L2} 를 ρ∈P, v≤ρ̇≤v̄ 제약 하에 최소화(식 2,9). 비-rate-bounded는 단일 2차 Lyapunov 함수(SQLF)로 LMI 풀이; rate-bounded는 파라미터 의존 Lyapunov 함수(PDLF)를 기저함수 X_ρ=X0+X1ρ(+X2ρ²), Y_ρ로 근사. 스케줄링 파라미터는 강체 설계는 ρ=t, 강체+유연 설계는 ρ=NGV(비중력속도, v_NGA≤ρ̇≤v̄_NGA, [17,40] m/s²). 그리드 기반(LPVTools lpvsyn) 6점 격자. 벤딩필터는 출력 가중함수 W_{β_c}(ρ) = (이중 노치필터 × 2차 LPV 저역통과 F(ρ))^{-1}(식 11)로 제어기에 내장. 최종 quadRB-LPV: 22상태, 유도 L2 노름 2.13, 최대 극점 ~150 rad/s. 구현 시 H_d(s)=s/(0.02s+1) 의사미분필터 추가 → 23상태.

**3. 비판적 검토.**
- 이론적 기여는 없다. SQLF/PDLF/IQC/grid-LPV는 모두 인용된 기성 기법이며 본 논문은 응용·튜닝·벤치마킹 논문이다(C1 제약).
- LPV의 핵심 매력인 "전 비행 포락선 보장"이 실제로는 부분적으로만 달성된다. 6 그리드점 quadRB가 유일하게 feasible했고, 극단점(t=20,90s)을 포함하면 linRB는 실패하여 격자를 좁혀야 했다 — 즉 GS의 고질적 문제(설계점 선택)가 LPV에서도 재현됨을 저자 스스로 인정.
- 유도 L2 보장은 명목 LPV 모델 기준이며, 130% 불확실성·비선형 시뮬레이터 하의 강건성은 형식적 보장이 아니라 MC로만 입증된다.
- 모델 계수가 기밀이고 시뮬레이터가 독점이라 재현이 사실상 불가(C6 낮음). 가중함수만 공개.
- 비교 대상(H∞, 적응)이 동일 저자군의 선행 연구라 완전한 외부 baseline은 아니나, VEGA baseline은 산업 실제 제어기로 외부성이 확보됨.

**4. 시뮬/실험.** 검증 규모가 강점. VEGA VV05(Sentinel-2A, 2015 실비행) 데이터, AVIO 산업 고충실도 6DoF 비선형 시뮬레이터(VEGACONTROL: 탄성/슬로싱/TWD/액추에이터 비선형 포함). Section 5: 4개 MC 캠페인 ×500런 = 2000런, 실측 4개 풍속프로파일, baseline 대비 LPV가 Qα 구조하중 포락선 위반 제거(baseline은 Mach~1.25에서 위반), 액추에이터 2-노름 소비 lane A 10%·lane B 5% 감소, drift/drift-rate 대폭 개선(전 지표 평균 baseline 대비 향상, 예외는 lane B ∞-노름 2% 악화)(sim). Section 6: 확장 불확실성(±130%, 균일분포) MC ×500, 구조적 H∞는 12건 LoV 발생, 적응·LPV는 LoV 없음; LPV가 Qα 피크 ~25%·에너지 15% 감소, 적응 대비 전반 우수(예외 lane B ∞-노름 ~5% 악화)(sim).

**5. Contribution(delta).** 출판년(2022, 단 작업은 2020 제출) SOTA 기준 delta: (i) 발사체 TVC에서 강체+유연(벤딩) 동역학을 단일 LPV 합성 절차로 동시 설계 — 산업 관행의 순차·반복 V&V를 단축; (ii) 비-rate→rate-bounded(linear/quadratic basis)로의 점진적·완전한 LPV 설계 워크플로 제시(타 논문은 최종 선택만 제시하는 경향); (iii) 발사체 LPV 응용이 희소했던 상황([13,14])에서 산업급 시뮬레이터·실비행 baseline·두 고급 기법과의 정량 벤치마킹을 통한 기술이전 입증. LPV 이론 자체는 1988년 이후 확립된 것이므로 방법 신규성이 아닌 "발사체 G&C로의 체계적 적용·검증"이 delta.

**6. Limitation·Future work.** 저자 인정: 그리드점·기저함수 선택의 비체계성(quadRB 초기 6점은 불안정·고주파 극점), 비행 포락선 극단부 처리 한계, 온보드 구현 시 이산화/보간으로 인한 성능 저하 가능성(메모리·실시간 비용). 내가 보는 한계: 형식적 강건 보장(IQC 기반 robust LPV)이 LPVTools 미지원으로 미적용 — 강건성 입증이 MC 의존; 실제 온보드/HIL 실시간 데모 부재(상태수·극점주파수 분석만); 비교 baseline의 자기 인용성; 계수 기밀로 외부 재현 불가.

**평가:** 71/100 (good) · 한줄평: 이론 신규성은 낮으나 발사체 TVC에 LPV를 체계적으로 이식하고 산업급 시뮬레이터·실비행 baseline·고급 기법과 폭넓게 검증한 견실한 in-domain core-G&C 응용 논문.

---

## (2020, Acta Astronautica, Guidance command generation and nonlinear dynamic inversion control for reusable launch vehicles: Acquatella et al.)

**1. 핵심 아이디어.** 재사용 발사체(RLV)의 예비설계 단계에서 초기 안정성·제어성 평가를 위한 범용 오프라인 G&C 아키텍처를 제안한다. (i) trajOpt/MOPS 기반 다단계 궤적최적화로 기준궤적 생성, (ii) Modelica/FMU 비선형 모델로 유도명령(비행경로각·공력각) 생성, (iii) 내부루프 자세추종을 2-루프 비선형 동적역변환(NDI)+선형 PI로 수행하는 통합 워크플로우다. DLR의 AURORA HTHL 개념(수평이착륙 TSTO)의 재진입 하강단(Phase P5)에 적용해 6-DOF 공칭 시뮬레이션으로 제어성을 보였다.

**2. 알고리즘.** 궤적최적화는 multiple-shooting 직접전사 후 가중 min-max 다기준 문제 min_T max_k{c_k(T)/d_k} s.t. 등식/부등식 제약을 SQP로 푼다. NDI 핵심: affine 시스템 x_dot=f(x)+g(x)u에 대해 u_cmd=g^{-1}(x)(v - f(x))로 비선형 상쇄 → 단일적분기형 y_dot=v, 가상입력 v=y_des_dot + K_P e로 e_dot+K_P e=0 (지수안정). 자세제어는 시간척도분리 가정하 2-루프: (1) body rate 루프 — 가변질량 자세 EOM(Eke)에서 M_B_cmd = I v_w + ω×Iω + M_i, M_B=M_a+(M_c)_δ δ로 δ_cmd 역산, PI로 ω_des = ω_cmd + K_P(ω_cmd-ω)+K_I∫(ω_cmd-ω); (2) 공력각 외부루프 — ω_cmd = G_2^{-1} v_att, G_2는 α,β 함수 행렬, e_att+K_att e_att=0.

**3. 비판적 검토.**
- NDI 이론은 전적으로 교과서/인용(refs 22,51-53)이며 가변질량 자세 EOM도 Eke(ref 54)에서 가져온다 — 본 논문 고유의 증명·정리 기여는 사실상 없다.
- 가장 큰 약점은 강건성 부재다. NDI는 모델 정확성에 본질적으로 민감(저자도 "exact knowledge가 큰 단점"이라 명시)함에도, 공칭(nominal) 시뮬레이션만 수행하고 파라미터·공력 불확실성, 바람·외란을 모두 future work로 미룬다. "제어성"을 주장하지만 불확실성 하 검증이 없어 주장과 증거의 간극이 있다.
- 외부 baseline 비교가 전무하고 단일 형상(AURORA) 단일 단계(P5)만 다루며 Monte-Carlo도 없다.
- 유도는 명시적으로 오프라인이며, 온보드 실시간 적용성에 대한 연산시간·복잡도 분석이 없다.
- 도구(trajOpt, MOPS, Modelica framework)가 DLR 사내 독점이라 재현이 어렵고 게인·파라미터 수치도 제공되지 않는다.
- 시간척도분리·정확한 모델 가역성 가정이 강하나, under-actuated/제어할당이 필요한 일반 경우는 scope 밖으로 배제된다.

**4. 시뮬/실험.** AURORA TSTO 개념의 하강 Phase P5 6-DOF 공칭 시뮬레이션 (sim). P5-a 재진입(Mach 18→5, α 50°→9°): RCS 단독, RCS+공력면 조합 두 케이스. 추적 결과 V,γ,χ,μ,α,β 모두 기준값을 잘 추종, body rate p,q,r 명령 추종 양호. 정량 결과: 공력면 병용 시 RCS 각충격량(angular impulse)을 절반 이상 절감(Fig.10). 제약: n_x<4.5g, q_dyn<60kPa(재진입), heat flux<900kW/m², 공력면 편향 ±30°(flap)/±20°(elevator)/±10°(aileron). P5-c 최종접근(Mach~1200s대): 공력면 단독 제어로 추종 성공. baseline 비교 없음, MC 없음, real/HIL 없음.

**5. Contribution(delta).** 출판년(2020) 기준 델타는 개별 요소의 신규성이 아니라 통합에 있다 — 궤적최적화(trajOpt/MOPS) + Modelica/FMU 비선형 모델 기반 유도명령 + NDI 자세제어를 하나의 일관된 예비설계 G&C 워크플로우로 결합. 저자도 refs[35-38]보다 ref[26]의 통합 접근에 기반하며 "RLV 문헌에서 이런 synergy는 널리 고려되지 않았다"고 명시. NDI 자체, multiple-shooting 전사, 가변질량 EOM은 모두 출판 이전 기성 기법이므로 방법론적 신규성은 낮고, FMU 기반 모델교환을 G&C 설계 파이프라인에 엮은 엔지니어링 통합이 실질 기여다.

**6. Limitation·Future work.** (저자) 불확실성·외란(바람,난류) 미고려 — 향후 강건성 분석 예정; 제어할당/under-actuated 경우 scope 밖; ferry flight 미고려. (내가 보는 한계) NDI의 모델민감성에 대한 정량평가·MC 부재로 "제어성" 주장이 공칭에 국한; 외부 baseline 부재로 통합 아키텍처의 상대적 이점 미입증; 온보드 실시간성 분석 없음; 독점도구·미공개 파라미터로 재현성 낮음; 단일 형상·단일 단계 검증으로 일반화 근거 약함.

**평가:** 55/100 (fair) · 한줄평: 기성 기법(NDI+궤적최적화+FMU)을 RLV 예비설계용으로 깔끔히 통합한 in-domain core-G&C 논문이나, 강건성·baseline·재현성 부재로 공칭 제어성 데모 수준에 머문다.

---

## (2018, AST, Attitude controller design for reusable launch vehicles during reentry phase via compound adaptive fuzzy H-infinity control: Mao et al.)

**1. 핵심 아이디어.** 재진입(reentry) 단계 RLV의 자세(roll/pitch/yaw) 추종 제어 문제를 파라미터 불확실성과 외란 하에서 다룬다. 6-DoF 회전 운동학을 모델 변환으로 2차 제어지향(control-oriented) 자세 모델로 만들고, 퍼지논리시스템(FLS)으로 불확실항을 근사하되 H-infinity 항이 야기하는 "근사능력 약화"를 피하기 위해 복합 적응 퍼지 H-infinity 제어(CAFHC) 전략을 제안한다. 제어기는 퍼지 근사영역 내부용 자세 추종 제어기와 외부용 경계 적응(boundary adaptive) 안정화 제어기 두 부분으로 구성되며 가중치 tau로 부드럽게 전환한다.

**2. 알고리즘.** 제어지향 모델: x_ddot = f(x)+Δf(x)+(g(x)+Δg(x))u+d, y=x (3채널 2차계). 내부 추종 제어기 u_T = [g+Δĝ]^{-1}[-f-Δf̂ + y_d_ddot + KE] + u_h^*, 여기서 H-infinity 항 u_h^* = [g+Δĝ]^{-1}u_h, u_h는 Riccati 방정식 P_iA_i+A_i^TP_i+Q_i+P_iB_i(1/rho_i^2 - 2/r_i)B_i^TP_i=0의 해 P_i로 구성. FLS로 Δf_s=Δf̂(x|θ_f)+ε_f, Δg=Δĝ(x|θ_g)+ε_g 근사. 식별모델(identification model, low-pass filter)과 적응법칙 θ_if_dot, θ_ijg_dot (학습률 gamma)로 파라미터 갱신. 외부 안정화 제어기 u_H=[g+v_g]^{-1}[-f-v_f+y_d_ddot+KE], 경계 파라미터 추정 m̂_if, m̂_ijg를 sgn(E_i^TP_iB_i) 기반으로 적응. 전체 제어법칙 u=tau*u_H+(1-tau)*u_T. Lyapunov 함수 V_Ti, V_Hi로 Theorem 1-3에서 폐루프 semi-global 점근 안정 및 추종오차 E_i의 소근방 D 수렴(UUB) 증명.

**3. 비판적 검토.**
- 안정성 결과는 원점 점근수렴이 아니라 소근방으로의 UUB(semi-global asymptotic + uniformly ultimately bounded)이며, 근방 크기는 lambda_min(Q_i), 근사오차 등에 의존한다. 강한 보장은 아니다.
- 핵심 기계장치(퍼지 근사 Lemma [38], 식별모델 [44], 경계 가정 Assumption 2 [46], Riccati H-infinity)가 대부분 인용·수입되며, 본 논문 고유 기여는 이 조각들의 복합(compound) 구성과 tau 전환에 있다. 증명 자체는 표준 Lyapunov 절차의 충실한 적용.
- tau 가중 전환식(62)의 R_x, R_0 설계가 다소 휴리스틱하며, 경계 영역에서의 두 제어기 블렌딩이 안정성에 미치는 영향(채터링, 전환 경계)에 대한 정량 분석이 약하다.
- sgn(·) 기반 경계 안정화는 실용상 채터링 우려가 있으나 액추에이터 응답에서 정량적 채터링 분석이 제시되지 않았다(Remark 2가 정성적으로만 언급).
- 추종 명령으로 sine 함수를 쓴 점은 저자도 인정하듯 실제 유도명령과 거리가 있어 현실성 검증이 제한적.

**4. 시뮬/실험.** MATLAB 6-DoF 수치 시뮬(real 데이터/HIL 없음, 모두 sim). 두 비행 case: case1 고도 50km·15Mach(alpha=theta=25deg), case2 고도 30km·8Mach(alpha=theta=15deg). 불확실성 Δf=20%f, Δg=20%g, 정현파 외란 d 주입. baseline은 self/유관 기법인 AFHC(adaptive fuzzy H-infinity) 단일 비교만. 정량 결과: case2에서 roll/pitch/yaw 최대 추종오차 < 0.65deg, CAFHC가 AFHC보다 작은 추종오차. 액추에이터 편향은 -20~20deg 범위 내. Monte Carlo 없음, 외부 SOTA(SMC, backstepping 등) 정량 비교 없음.

**5. Contribution(delta).** 출판년(2018) 기준으로, 적응 퍼지·H-infinity·강건 제어를 RLV 재진입 자세제어에 결합한 선행([24][25] SMC, [28] H-infinity, [29][30] 적응)이 이미 존재하는 상황에서, 본 논문의 delta는 (i) FLS+H-infinity 결합 시 발생하는 "근사능력 약화" 문제를 명시적으로 지적하고 (ii) 퍼지 근사영역 내부/외부를 분리해 식별모델 기반 추종기 + 경계 적응 안정화기를 tau로 결합한 복합(CAFHC) 구조를 제안한 점. 즉 새 방향을 여는 발명이라기보다 알려진 요소들의 영리한 조합·정합. 증분적 진보(incremental).

**6. Limitation·Future work.** 저자: sine 추종 명령의 비현실성 인정, 채터링·전환 가중치는 정성 논의에 그침. 내가 보는 한계: (a) 외부 baseline 부재 및 Monte Carlo 미실시로 강건성 주장이 통계적으로 뒷받침되지 않음; (b) 원점 점근 아닌 UUB 보장으로 정밀도 한계; (c) 온보드 실시간성·연산복잡도(다수 퍼지 규칙×3채널 적응)에 대한 timing 분석 전무; (d) 실제 유도명령·풍하중·액추에이터 동특성·포화 미반영. Future: 외부 강건 baseline과 MC 통계, 명령 현실화, 실시간 구현 분석이 필요.

**평가:** 67/100 (good) · 한줄평: 완결된 Lyapunov 증명과 명확한 RLV 재진입 자세제어 적용을 갖춘 견실한 core-G&C 논문이나, 기여가 알려진 요소들의 복합 구성에 머물고 검증이 self-baseline·無 MC로 약해 good 하단에 위치.

---

## (2025, AST, Optimal fault-tolerant control of reusable launch vehicle with actuator faults using zero-sum differential game theory: Ren et al.)

**1. 핵심 아이디어.** RLV 재진입 단계 자세제어계에서 작동기 고장(효율저하 + 바이어스)과 외란을 동시에 다루는 최적 내고장 제어를 제안한다. 제어기와 고장/외란을 적대적 두 플레이어로 보는 영합(zero-sum) 미분게임으로 문제를 정식화하고, Nash-Pontryagin minimax로 안장점을 구해 최적 제어칙과 바이어스의 최악경계를 도출한다. HJI 방정식의 해(이상 비용함수)는 ADP 기반 적응 critic 신경망으로 온라인 추정한다.

**2. 알고리즘.** 작동기 고장 모형 u = σ v + ū (σ=diag{σ_i}∈(0,1] 효율, ū 바이어스). 오차동역학 Φ_e에 대해 비용 J = ∫(Φ_eᵀQΦ_e + vᵀΛ_1 v − κ ŪᵀΛ_2 Ū)dt. minimax 안장점에서 v_i* = −(1/2λ_i)σ_i g_iᵀ∇J*, Ū* = (1/2κ)Λ_2⁻¹gᵀ∇J*. HJI: 0 = Φ_eᵀQΦ_e + (∇J*)ᵀ(f−Φ_d) − (1/4)(∇J*)ᵀgσΛ_1⁻¹σgᵀ∇J* + (1/4θ)(∇J*)ᵀgΛ_2⁻¹gᵀ∇J*. critic NN Ĵ=ŵ_cᵀσ_c, 경사하강 + 안정화항(G·, φ_i/k_i)을 추가한 수정 적응칙 (46)-(47)로 가중치 갱신. Lyapunov V_2로 상태오차·NN 가중치오차의 UUB 증명(Theorem 1).

**3. 비판적 검토.**
- 핵심 ADP/critic-NN 기계(가중치 갱신칙, 안정화항, UUB Lyapunov 구조)는 참고문헌 [42]-[46]에서 직접 차용한 것으로, 본 논문 고유의 이론 기여는 게임-FTC 문제로의 적용·재구성에 가깝다.
- 검증이 단일 차량·단일 공칭궤적에 한정되고 Monte Carlo가 없다. 20/40/60s에 결정론적 고장을 주입한 한 시나리오만 제시되어 통계적 일반화가 약하다.
- Table 1에서 ASMC/backstepping/MPC/고정시간 등 경쟁기법을 나열했으나, 정작 정량 비교는 "FTC 적용 vs 미적용(Fig. 6)" 자기-baseline뿐이다. 외부 SOTA와의 비교가 없어 우월성 주장이 뒷받침되지 않는다.
- 저자 스스로 "수학모형 정확도 의존·연산부담으로 실시간성 제약 가능"을 인정하며 연산시간/복잡도 분석이 전혀 없다. HIL·비행시험은 future work로만 언급.
- 관성 불확실성은 Remark 1에서 회피하고 자신의 선행연구로 미룬다 — 재진입 연료소모/스러스터 효과를 고려하면 실전성 측면의 공백.

**4. 시뮬/실험.** RLV 1대(m=6165kg, L_r=20m), H=25000m, V=1200m/s, γ=−41deg. 효율저하+바이어스 고장을 20/40/60s에 단계적 주입, 외란 d_2 포함. 결과: 자세추종오차 10⁻² deg 이내 UUB 유지(Fig.1,2), critic 가중치 수렴(Fig.4), N(0,0.01) 노이즈 하 강건성(Fig.5). IAE/ITAE/ISE 성능지표를 구간별 제시(Table 2). FTC 미적용 시 20s 이후 발산(Fig.6, 최대 ~40deg). 전부 (sim), 외부 baseline·MC·실측 없음.

**5. Contribution(delta).** 출판년(2025) 기준, RLV 재진입 자세 FTC에 영합 미분게임 + ADP를 결합하고 효율저하·바이어스·외란을 동시에, 고장패턴행렬의 경계 가정 없이 처리한 점이 delta다. 다만 게임이론 기반 FTC([32][33])와 ADP critic-NN([42]-[46])이 모두 기성 도구이고 저자 선행연구([40][41])의 연장선이라, 새 방향을 여는 형성적 기여라기보다 확립된 도구의 도메인 적용에 해당한다(점진적).

**6. Limitation·Future work.** 저자: 수학모형 정확도 의존, 연산부담으로 실시간성 제약, 관성 불확실성 미고려 → HIL·비행시험·관성적응 예고. 추가로 평자가 보는 한계: (i) 외부 SOTA baseline 부재로 정량적 우월성 미입증, (ii) Monte Carlo·다중 시나리오 부재로 통계적 강건성 미검증, (iii) 안장점이 고장 가변성으로 최적 아닐 수 있음을 본문도 인정 — 최악경계 보수성에 대한 정량평가 없음, (iv) 코드/데이터 미공개.

**평가:** 62/100 (fair) · 한줄평: 게임이론과 ADP를 RLV 재진입 FTC에 결합한 견실한 core-G&C 논문이나, 외부 baseline·MC·실시간성 분석 부재로 TAES/TASE 상위 기준에는 못 미치는 점진적 기여.

---

## (2022, ISA Transactions, Fixed-time disturbance observer-based nearly optimal control for reusable launch vehicle with input constraints: Zhang et al.)

**1. 핵심 아이디어.** RLV 재진입 자세제어를 시간 스케일에 따라 외부루프(자세각 Theta=[alpha,beta,sigma])와 내부루프(각속도 omega=[p,q,r])로 분리해 각각 다른 기법을 적용한다. 외부루프는 미스매치 외란·모델 불확실성의 상한 정보가 필요 없는 적응이득 다변수 일반화 슈퍼트위스팅(AMGST)으로 가상 제어입력을 생성하고, 내부루프는 고정시간 외란관측기(FTDO)로 매치 외란과 가상입력 미분을 추정한 뒤 critic-actor NN 기반 근사최적 제어(NOC)로 입력제약(tanh 포화)을 만족하는 제어모멘트를 산출한다. 외란상한이 미지인 상황에서 채터링 저감과 입력제약 동시 처리를 노린 통합 설계다.

**2. 알고리즘.** 외부루프: omega_c = R^{-1}(-k1*ceil(e_Theta)^{1/2} - k1*e_Theta + z + Theta_d_dot), z_dot = -k2*(0.5*ceil(e_Theta)^0 + 1.5*ceil(e_Theta)^{1/2} + e_Theta), 두 이득 k1,k2는 두 개의 서로 다른 적응법칙 (12)-(15)로 조정(상태/시간의존 외란 대응). 내부루프 FTDO: z0_dot, z1_dot에 HOSM형 고정시간 항(지수 alpha_j<1, beta_j>1)으로 Delta_omega와 omega_c_dot 추정, 이득 lambda_n은 Hurwitz 행렬 A1,A2로만 결정되어 외란상한과 무관. NOC: 성능지수 V2에 입력제약 페널티 Y_omega(M)=2*int(M_bar*tanh^{-1}(m/M_bar))^T*R dm를 두고 HJB로 M*=-M_bar*tanh(0.5*M_bar^{-1}*R^{-1}*I^{-T}*grad V2*), critic/actor NN 가중치를 경사하강+안정화항 Upsilon로 갱신. 안정성은 유한시간(외부)·고정시간(FTDO)·UUB(내부) Lyapunov로 증명.

**3. 비판적 검토.**
- 핵심 구성요소가 모두 기존 기법의 확장 조합이다: 슈퍼트위스팅(GSTA/AMST [27]), FTDO(HOSM), tanh 기반 입력제약 ADP([40][54][58])가 각각 선행연구에 존재하며 본문은 이를 RLV에 통합. 증명 다수가 인용 정리(Thm 4.2 of [50], HJB [59], constrained-input [40][54])에 의존.
- 시간 스케일 분리(병진동역학이 회전보다 느림, Assumption 2)와 분리원리(Remark 6) 가정이 강하며, 결합된 폐루프의 엄밀한 합성 안정성은 정리별 분리 증명에 그친다.
- 검증이 단일 재진입 시나리오·단일 외란프로파일이며 Monte Carlo나 파라미터 산포 분석이 없어 robustness 주장 대비 통계적 뒷받침이 부족하다.
- 온라인 NN 학습(critic-actor) + 1ms 스텝의 임베디드 실시간성/연산복잡도 분석이 없어 온보드 적용 근거가 정성적이다.
- AMGST는 타 기법 대비 파라미터 수가 많아(저자도 인정) 튜닝 부담이 크다.

**4. 시뮬/실험.** Matlab/Simulink, X-33 기반 RLV, 1ms 스텝, 0-200s (sim). 초기 Theta0=[75,-37,-52]deg, omega0=[57.3,45.8,34]deg/s, 미스매치/매치 외란(sin/cos 시변) 및 외부외란 Delta D 부여. 비교 baseline 3종(외부 문헌 기반): AMSTC [27], FGSTA [48], AMGST+ADP [58]. 결과: FTDO-NOC의 자세 추종오차 약 1.5e-4 deg로 타 기법보다 우수, 오버슈트 최소, 채터링 최소; FTDO 추정오차 약 0.1 deg, e1는 0.4s에 수렴. 성능지수(IAE, 제어모멘트 적분 에너지)에서 FTDO-NOC가 최소 에너지·최고 추종 달성. 별도 MC/HIL/실비행 없음.

**5. Contribution(delta).** 출판년(2022) 기준 delta: (i) AMST [27]에 두 개의 독립 적응법칙을 둔 AMGST로 상태·시간의존 외란을 동시 처리하고 2층 이득 r(t)의 과대추정([25] 대비)을 회피; (ii) 외란상한과 무관하게 Hurwitz 행렬만으로 이득을 선택하는 FTDO; (iii) 추가 보조시스템 없이 ADP 근사최적 제어에 입력제약(tanh)과 외란보상을 통합해 RLV 내부루프에 적용. 각 요소는 선행 대비 점진적 개선이며 진정한 신규성은 RLV 재진입에 대한 통합 프레임워크 구성에 있다.

**6. Limitation·Future work.** 저자: 자세·각속도 제약을 명시적으로 다루지 못함을 향후과제로 제시. 내가 보는 한계: 단일 시나리오·무 MC로 robustness 통계 근거 부족, 온라인 NN의 실시간 연산비용·임베디드 검증 부재, 분리원리/시간스케일 가정의 결합 안정성 엄밀성, 과다 파라미터 튜닝 부담, 코드 미공개. 분리 증명을 결합 폐루프 보장으로 강화하고 HIL/산포 검증을 추가하면 설득력이 커질 것.

**평가:** 70/100 (good) · 한줄평: 슈퍼트위스팅+FTDO+입력제약 ADP를 RLV 재진입에 견실히 통합하고 완전한 Lyapunov 증명과 외부 baseline 비교를 갖춘 견고한 in-domain G&C 논문이나, 구성요소의 점진적 성격과 단일시나리오·무 MC 검증이 상한을 제한한다.

---

## (2025, SysTol, Loss-of-Efficiency Fault Detection and Diagnosis for Launch Vehicle Thrust Vector Control Systems: Farì et al.)

**1. 핵심 아이디어.** 발사체 추력벡터제어(TVC)의 전기기계식 액추에이터(EMA)에서 발생하는 효율손실(Loss-of-Efficiency, LoE) 고장을 모델기반 FDD(고장검출·진단)로 잡는다. 드라이브트레인 효율 η∈[0,1]의 저하(공칭 0.9 → 부분손실 0.5 또는 jamming 0)를 대상으로, nullspace 기반 잔차생성 필터를 설계해 vehicle-induced 하중과 제어입력으로부터 잔차를 decouple하되 고장에는 민감하게 만든다. 핵심 설계결정은 차량/자세 동역학을 버리고 **액추에이터-로컬 동역학만** 쓰는 것 — 저자는 차량레벨(질량변동→gain-schedule batch + 불안정)에서는 online 효율추정이 "적용불가할 수 있다"고 명시하며 회피한다.

**2. 알고리즘.** 4상태 선형 TVC 모델 x=[θ̇,θ,β̇,β](모터각속도·모터각·노즐율·노즐각), 입력 u=[τ_m,τ_g](모터토크·vehicle-induced 짐벌하중). LoE를 가법적 추가입력 f(t), B_f=[0,0,J_e^{-1},0]^T로 모델링. nullspace 필터 Q(s)=N_2·N_1·N_0로 R_u=Q[G_u;1]=0(제어입력 완전 decouple)·R_f≠0(고장 민감). Ω={0}에서의 강건검출을 위해 invariant zero s_0=0에서 확장블록 G_e(s_0) 열랭크 손실(G_f≠0 + 추가행 F_e) 조건 분석 — τ_g를 unknown-disturbance로 두면 rank 손실 2로 고장까지 소거되므로, τ_g를 차량상태로 폐형식 추정해 measurable 입력으로 처리. 잔차평가는 Narendra r_θ + 임계값 J_th, 확인플래그 Ξ(검출∧|τ_m|≥J_min∧분산<J_σ). 효율은 ε̂=r/τ_m로 정량화(고장→잔차 정상이득=1).

**3. 비판적 검토.**
- nullspace FDI 기계(N_0/N_1/N_2 합성, invariant-zero 강건검출)는 Ossmann/Varga(FDITOOLS)에서 전량 수입 — delta는 발사체 TVC LoE라는 고장유형·도메인 적용과 τ_g 처리에 국한.
- τ_g(vehicle-induced load)를 **완전히 알려진 값으로 가정**(τ̂_g=τ_g). 저자도 β 의존이라 센서고장 시 부정확해질 수 있음을 인정하나 검증은 완전기지 하에서만 — 이 가정이 무너지면 decoupling이 깨진다.
- 액추에이터-로컬 한정으로 **공력이 잔차를 압도하거나 Mach-가변 효율이 얽히는 어려운 영역을 구조적으로 회피** — 손쉽게 검출되는 내부 전기기계 신호에 의존.
- 외부 baseline 부재(저자 선행 [12,13] 인접), 단일 축, 비선형모델 전환 시 평가로직 재튜닝 필요(마찰/Stribeck이 α=0 평가에서 masking).

**4. 시뮬/실험.** 전부 시뮬. (a) 선형 TVC MC 500회(파라미터 uniform 10~20% 섭동): 무고장 오경보 0, 부분 LoE(η→0.5, Case#1) 검출 ~[t_d,t_d+0.2s], jamming(η→0, Case#2) 더 빠른 [t_d,t_d+0.1s]. (b) 고충실 비선형(MPT-L6: Coulomb/Stribeck·backlash·preload, 3D full TVC) MC 100회: α=0,β=1,γ=1 재튜닝, 무고장 오경보 0, jamming 검출 [t_d,t_d+0.15s]. 참조명령 ±10° 계단열, 고장주입 t_f=7s. HIL/실HW 없음, 외부 baseline 없음.

**5. Contribution(delta).** 출판년(2025) 기준: 저자 선행이 EMA-TVC의 *센서* 고장 FDI였던 데 반해([12,13]), 본 논문은 nullspace 접근을 *액추에이터 LoE*로 확장. delta = (i) LoE를 가법입력으로 모델링하고 vehicle-induced load를 unknown이 아니라 measurable로 두어 invariant-zero 강건검출을 성립시킨 문제정식화, (ii) ε̂=r/τ_m 효율 정량화, (iii) 선형+고충실 비선형 이중 MC 검증. 합성이론은 도입형이고 재구성 없이 진단까지.

**6. Limitation·Future work.** 저자: τ_g 부정확 시 성능저하(센서고장 검출로 redundant 전환 필요), 비선형 unmodeled(마찰)로 재튜닝 필요, end-to-end 폐루프 고충실 검증이 향후과제. 추가(평가자): 액추에이터-로컬 한정이 공력-지배·Mach-가변 효율의 근본 식별문제를 회피, τ_g 완전기지 가정이 강함, 외부 baseline·온보드 실시간 타이밍 정량·재구성 부재, 단일 축.

**평가:** 66/100 (good) · 한줄평: nullspace FDD를 발사체 EMA-TVC의 LoE에 견실히 적용하고 선형+고충실 이중 MC로 검증한 in-domain FDD 논문이나, 도구가 수입형이고 τ_g 완전기지·액추에이터-로컬 한정·외부 baseline 부재가 상한을 good에 묶는다.
