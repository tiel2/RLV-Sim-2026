# Thread A — convex optimization / lossless convexification / successive convexification 결정론 backbone (19편)

> 스레드 A = RLV-GNC 결정론 궤적생성 backbone(convex/LCvx/SCvx 계보): LCvx 창시(2007)→제어제약 무손실(2013)→상태제약 일반화(2014)→6-DoF SCvx(2018)→통합 튜토리얼(2022), 그리고 SCP 수렴이론·변형(Remarks·SCvx-fast·SCvx*·seco·DACX), 이산/연속시간 이론(ct-SCvx·Revisiting-LCvx·DT-LCvx-Pointing), indirect hybrid(GRASHS), GPU-native SCP까지.
> 점수 = 재교정 후 오케스트레이터 confirmed(밴드: 85+ flagship · 75–84 strong · 65–74 good · 55–64 fair). venue Tier는 §3 ledger에서 별도 처리.
> 리뷰는 6섹션 비판 prose + 한 줄 verdict. 수식 Unicode/plain. 한국어.

---

## (2007, JGCD, Convex Programming Approach to Powered Descent Guidance for Mars Landing: Açıkmeşe·Ploen)

**1. 핵심 아이디어.** 화성 정밀착륙(pinpoint landing)의 동력하강 구간 최소연료 궤적최적화 문제를 유한차원 2차 원뿔계획(SOCP)으로 정식화한다. 핵심 난점은 추력 크기 하한(ρ1 ≤ ‖T_c‖)으로 인한 비볼록 제어제약인데, 저자들은 슬랙변수 Γ을 도입하고 ‖T_c‖ ≤ Γ로 완화한 뒤, 완화 문제의 최적해가 원래 비볼록 문제의 최적해와 동일함(무손실 볼록화, lossless convexification)을 Pontryagin 최대원리로 증명한다. SOCP는 내점법으로 다항시간·결정론적 수렴보장으로 풀리므로 온보드 실시간 구현 가능성을 연다(G-FOLD의 기반).

**2. 알고리즘.** 동역학: r̈ = g + T_c/m, ṁ = −α‖T_c‖. 비볼록 제약 ρ1 ≤ ‖T_c‖ ≤ ρ2를 슬랙 Γ로 완화: ‖T_c‖ ≤ Γ, ρ1 ≤ Γ ≤ ρ2 (Problem 2). 변수변환 σ=Γ/m, u=T_c/m, z=ln m으로 질량 비선형성을 분리: ż=−ασ, ‖u‖≤σ, ρ1·e^{−z} ≤ σ ≤ ρ2·e^{−z}. 후자의 비볼록 항은 e^{−z}의 2차/1차 Taylor 전개로 SOC·선형 근사(Problem 3), 오차상한 식(38)(39)로 보증. 시간을 N구간으로 이산화하고 제어를 기저함수(piecewise-constant)로 표현해 유한차원 SOCP(Problem 4)로 변환, t_f는 선탐색(line search)으로 결정. glide slope·thrust pointing 제약도 SOC로 추가(pointing은 휴리스틱 완화).

**3. 비판적 검토.**
- Lemma 1의 무손실 볼록화 증명은 엄밀하나 추력 *방향* 제약(pointing)은 무손실이 아니며 저자도 휴리스틱(식 62의 완화)으로 처리한다 — 이 부분은 보장이 없고 시뮬에서 초기 12–15초간 제약위반(최대 90deg)이 관측된다.
- 검증이 전부 (sim)이고 외부 baseline과의 정량 비교(연료·계산시간)가 없다. SOCP의 다항시간성은 이론적으로 강하나 실제 온보드 타이밍 측정은 future work로 미룬다.
- 불확실성(바람·공력)을 외란으로 명시 무시하고 nominal 궤적만 다룬다; 강건성/몬테카를로 분석 부재.
- Taylor 근사로 인해 풀린 것은 엄밀히는 원문제가 아닌 근사문제이며, "근사가 충분히 정확"이라는 주장은 식(38)(39) 상한과 Fig.4로 뒷받침되나 N→∞ 수렴의 엄밀 논의는 범위 밖으로 명시.

**4. 시뮬/실험.** MSL급 파라미터(m_wet=1905kg, I_sp=225s, T̄=3.1kN, n=6, φ=27deg, g_화성=3.71). 다중 시나리오(sim): (a) no-subsurface 미부과 시 지하비행 발생(t_f*=72s, 387.9kg), (b) 부과 시 표면접촉(75s, 390.4kg), (c) glide slope(θ̃=86deg) 부과 시 실용해(81s, 399.5kg), (d) pointing 제약 무/유(t_f*=69s/293.6kg vs 78s/303kg, 약 10kg 추가연료). bang-bang(max-min-max) 추력 프로파일이 Lemma 1과 부합함을 확인. 외부 baseline·MC 없음. SeDuMi/YALMIP(MATLAB) 사용.

**5. Contribution(delta).** 2005~2007 당시 동력하강 유도는 직접/근사 비선형계획(Topcu[6], Najson[7])이나 폐형해(1-D)에 의존했고, 비볼록 추력하한을 전역최적·결정론적 수렴 보장으로 푸는 방법이 없었다. 본 논문은 (i) 최소연료 PDG를 SOCP로 정식화하고 (ii) 추력하한 비볼록 제약을 *무손실 볼록화*함을 증명한 최초 결과로, "generic NLP은 온보드 부적합"이라는 문제를 정면 해결한다. 출판년 SOTA 대비 새로운 방향을 연 field-opening 기여이며 이후 G-FOLD·실비행 데모의 직접적 토대.

**6. Limitation·Future work.** 저자 명시: 전용 SOCP 솔버 미구현으로 실시간성 미입증, pointing 제약의 정량 보장 미흡, 낙하산 컷오프 시점 최적화·6자유도(병진+자세 결합) 확장·온보드 알고리즘과의 계산시간 비교가 future work. 내가 보는 추가 한계: 불확실성하 강건성 부재, 외부 baseline 정량비교 없음, Taylor 근사의 적정성이 파라미터 의존적일 수 있음.

**평가:** 86/100 (flagship) · 한줄평: 비볼록 동력하강 유도를 무손실 볼록화로 SOCP화한 분야 정초 논문으로, 이론적 엄밀성과 G&C 실용 임팩트(G-FOLD 기반)가 모두 최상급.

---

## (2013, TCST, Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem: Açıkmeşe·Carson·Blackmore)

**1. 핵심 아이디어.** 행성 연착륙(soft landing)의 연료최소 동력하강 유도 문제에서 제어입력에 존재하는 비볼록(nonconvex) 제약 — 추력 하한(ρ1>0), 그리고 추력 방향 포인팅 제약(편차각 θ가 π/2를 넘으면 비볼록) — 을 볼록 완화(relaxation)했을 때 그 완화가 무손실(lossless)임을 증명한다. 즉 완화된 볼록 문제의 최적해가 원래 비볼록 문제의 최적해와 일치함을 보장한다. 이로써 SOCP/IPM로 원 문제를 다항시간·실시간에 전역최적으로 풀 수 있게 된다.

**2. 알고리즘.** 동역학: ẋ=A(ω)x+B(g+T_c/m), ṁ=-α‖T_c‖. 슬랙변수 Γ 도입으로 ‖T_c‖≤Γ, ρ1≤Γ≤ρ2, n̂^T T_c ≥ Γcosθ 로 완화(볼록 half-space ∩ ball). 변수변환 σ=Γ/m, u=T_c/m, z=ln m 으로 1/m 비선형 제거 후 SOCP화(추력하한은 z 둘레 2차 근사). 2단계 우선순위: Problem 3(최소착륙오차)→Problem 4(최소연료). 핵심 정리(Theorem 1): Pontryagin 최대원리 + 정규(normal) 시스템 논증으로, 완화문제 최적추력이 U(Γ) 집합의 극점(extreme point)에 위치 → ‖T_c*‖=Γ a.e. → 원 제약 만족을 증명.

**3. 비판적 검토.**
- 무손실성 증명이 상태궤적이 가능집합 X의 내부에 strictly 있을 때(intX)로 가정됨. 경계와 교차(glide-slope·속도상한 active)하는 경우는 시뮬에서 성립을 관찰했다고만 하고 증명은 향후 과제로 미룸 — 이론적 완결성에 명시적 gap.
- 정리는 ω를 ω̂로 약간 섭동시켜야 일반적으로 성립(S(ω)n̂ 특이 경우 ε·n̂^⊥ 추가). 저자도 인정하듯 이는 약간 다른 동역학의 문제를 푸는 것이며, 실무상 무해하다 주장하나 엄밀히는 caveat.
- 강건성/불확실성 처리는 정성적 제안(추적 피드백, 재계산)에 그치고 실제 설계·검증 없음.

**4. 시뮬/실험.** Mars 단일 시나리오(m0=2000kg, Tmax=24000N, g=-3.71 m/s², ω=실제 화성 자전율). 포인팅 제약 3종(무제약/90°/45°) 비교(sim). Table I: 무제약 200.1kg·44.63s → 90° 201.8kg·46.96s → 45° 222.3kg·57.29s 로 제약 강화 시 연료·비행시간 증가 정량화. 두 번째 예제는 glide-slope·속도경계와 유한개 접촉하는 경우에도 무손실 성립 시연. MC 없음, 외부 baseline 비교 없음(자기 선행연구 [1] 대비 확장 논증).

**5. Contribution(delta).** 선행 [1](Acikmese-Ploen, 추력 하한만 볼록화)·[2]를 통합·확장하여 (a)비볼록 추력 포인팅 제약(θ∈[0,π/2] 포함), (b)추력 하한, (c)위치(glide-slope)·속도 제약, (d)행성 자전을 모두 동시에 다루는 무손실 볼록화를 최초 정립. 출판년(2013) 기준, 극점(extreme point) 기반의 새로운 기하학적 통찰(normal system 연결)로 무손실성을 증명한 점이 핵심 novelty이며, 이후 [27]의 일반화 및 G-FOLD 계열 연구의 직접적 토대가 됨.

**6. Limitation·Future work.** 저자: 경계 교차 시 증명, 강건 피드백 설계, 자세동역학 명시적 포함, 더 넓은 제어제약 클래스로의 일반화. 내가 보는 한계: 검증이 단일 결정론적 시나리오로 협소(MC·외부 baseline 부재)하여 C5가 약함; ω 섭동·intX 가정으로 무손실 보장이 조건부; 실시간성은 IPM 다항시간 경계로 논증되나 본 논문 내 실측 타이밍은 없음.

**평가:** 82/100 (strong) · 한줄평: 무손실 볼록화를 비볼록 포인팅 제약까지 확장한 G&C 분야의 기초 정형화로, 이론적 엄밀성과 온보드 적용성은 매우 높으나 실험 검증 폭은 제한적.

---

## (2014, Automatica, Lossless convexification of non-convex optimal control problems for state constrained linear systems: Harris·Açıkmeşe)

**1. 핵심 아이디어.** 비볼록 제어 제약(예: 하한이 있는 추력 크기 ρ1≤‖u‖≤ρ2)과 선형 상태 제약(x∈X)을 동시에 갖는 유한구간 최적제어 문제 P0에 대해, 볼록 완화 문제 P1의 최적해가 원래 비볼록 문제 P0의 전역 최적해가 됨을 증명한다(lossless convexification). 핵심 기여는 상태 제약이 유한 구간 동안 연속적으로 활성(active)인 경우까지 무손실 볼록화를 확장한 점으로, 기존 연구(Açıkmeşe&Blackmore 2011; Açıkmeşe&Ploen 2007)가 상태 제약을 유한 개 시점에서만 접촉하는 경우로 한정되던 한계를 넘는다. 이로써 행성 연착륙·랑데부 등 상태 제약이 부분공간 경계 위에서 작동하는 문제를 내부점법(IPM)으로 전역 최적·다항시간에 풀 수 있다.

**2. 알고리즘.** P0: min m(tf,x(tf))+∫ℓ(κ(u))dt, s.t. ẋ=Ax+Bu+Ew, 0<ρ1≤κ(u)≤ρ2, Cu≤d, x∈X, b(tf,x(tf))=0. 볼록 완화 P1은 슬랙변수 Γ(t)를 도입해 비볼록 제약을 ρ1≤Γ≤ρ2, κ(u)≤Γ로 분리(원뿔/SOCP화). 증명 골격: (i) 선형계가 부분공간 X에서 진화할 필요충분조건은 제어가 u=Fx+Gv+Hw 형태(F,G,H는 'friend')라는 Trentelman 등의 기하 제어 이론 도입, (ii) X가 강제어가능(strongly controllable) 부분공간임을 Criterion 1로 a priori 검증, (iii) Milyutin–Osmolovskii의 혼합 상태/제어 제약용 최대원리(비자명·점별최대·인접·정상성·상보슬랙·횡단성 조건)를 적용. P0→P2→P3 좌표 변환을 거쳐 Lemma 6–10으로, Criterion 1 하에서 최적해가 반드시 κ(u(t))=Γ(t) a.e.를 만족(슬랙이 tight)함을 보여 무손실성을 입증(Theorem 11).

**3. 비판적 검토.**
- 핵심 기계장치(혼합제약 최대원리는 Milyutin–Osmolovskii, friend/강제어가능 부분공간은 Trentelman et al.)는 인용·수입이며, 본 논문의 독자적 정리(Thm 2,4,11, Lemma 6–10)는 이를 새 문제군에 적용·연결한 것. 증명 자체는 빈틈이 적고 엄밀하나 'OWN proof, no gaps' 수준은 아님.
- Criterion 1(시불변+friend 존재 또는 시변+A-불변·선형독립 조건)은 강한 구조적 가정으로, X가 강제어가능 부분공간일 때만 무손실성이 보장된다. 실제 문제에서 이 조건이 깨지면(특이 호/adjoint 점프, 각주1) 결과가 적용되지 않는다 — 적용 범위가 명확히 한정적.
- 실증은 예시 2건뿐이며 외부 baseline·몬테카를로·통계가 전혀 없다. 무손실성을 수치로 확인(완화 후 슬랙이 tight한지)하는 정량 검증조차 도표 수준에 머문다.
- 외란 w(t)는 '알려진' 시변 입력으로만 포함되어 robustness 주장은 없다. 불확실성 하 보장은 본 논문 범위 밖.

**4. 시뮬/실험.** 예시 2건(순수 sim, SDPT3/Yalmip). (1) 화성 연착륙: 6상태(거리/고도/횡거+속도), 2≤‖u‖≤10 m/s², 정현파 외란, 45° 접근각 제약(x1−x2=0), 연료최소화 ∫‖u‖dt. 상태궤적·추력크기(상·하 호에서 상한 활성, 중간 호 내부) 제시. (2) 저추력 랑데부: Clohessy–Wiltshire–Hill 동역학, 일정고도(x1=0) 제약, 3≤‖u‖≤5 km/hr², 추력각 ≤45°, 최소시간 J=∫1dt. 두 예 모두 X가 강제어가능 부분공간임을 명시 검증. 정량 baseline/MC 없음, 그림 위주.

**5. Contribution(delta).** 출판년(2014) 기준: 선행 무손실 볼록화는 비볼록 제어 제약만(Açıkmeşe&Ploen 2007=G-FOLD 기반, Açıkmeşe&Blackmore 2011) 또는 상태 제약이 유한 시점만 접촉하는 경우로 한정. 본 논문은 상태 제약이 유한 구간 동안 연속 활성인 일반 선형계로 확장한 '최초의' 무손실 볼록화 결과로, friend/강제어가능 부분공간이라는 기하 제어 개념을 무손실 볼록화에 도입한 것이 새로운 방향 열기에 해당. 따라서 그 시점 novelty는 분명한 선두적 기여.

**6. Limitation·Future work.** 저자: 무손실성은 Criterion 1(강제어가능 부분공간) 충족 시에만 성립; tf 자유 문제는 별도 라인서치(golden section/bisection) 필요. 내가 보는 한계: (a) 비선형 동역학·비선형 상태 제약 미적용(선형계 한정), (b) 외부 baseline·MC·실시간 타이밍 측정 부재로 실증 강도 약함, (c) adjoint 변수가 ill-behaved한 특이 사례에서의 적용성 미해명, (d) 본 논문 자체에는 실시간/임베디드 실험 없음(다른 비행시험 인용에 의존).

**평가:** 78/100 (strong) · 한줄평: 무손실 볼록화를 구간-활성 상태 제약까지 확장한 이론적으로 단단하고 G&C(특히 행성 착륙·랑데부 유도)에 직결되는 control-theory 기여 — 다만 실증은 예시 수준이고 가정이 강하다.

---

## (2017, JGCD, Implementation and Experimental Demonstration of Onboard Powered-Descent Guidance: Scharf·Açıkmeşe·Dueri·Benito·Casoliva)

**1. 핵심 아이디어.** Lossless convexification 이론에 기반한 연료 최적·제약 충족 동력하강 유도(G-FOLD)를 실제 VTVL 로켓(Masten Xombie)에 탑재하여 비행 시연한 논문이다. ADAPT 프로그램 3년에 걸쳐 총 8회 비행했고, 이 중 5회는 G-FOLD를 온보드(1.4 GHz 프로세서)에서 실시간으로 실행해 하강 중 궤적을 직접 생성했다. 제약 충족형 convexified PDG가 실 하드웨어에서 동작함을 입증한 최초의 시연으로, G-FOLD를 향후 행성 착륙 미션에 적용할 준비를 갖추는 것이 목표다.

**2. 알고리즘.** 원 PDG 문제: min m_p = m0 - m(tf), s.t. r'=v, v'=g+T/m - ω×ω×r - 2ω×v, m'=-α||T||, 추력 한계 Tmin≤||T||≤Tmax, 추력각 콘 ẑ^T T ≥ ||T|| cosθ_T,max, glideslope, 속도 ||v||≤vmax. Lossless convexification: z=ln m, u=T/m, σ가 ||T||/m 대체 → 비볼록 추력 하한이 σ에 대한 SOC 제약 Tmin e^{-z}[1-(z-z0)+½(z-z0)²]≤σ로 변환되어 SOCP(min c^T x, Ax=b, x∈K)로 풀린다(IPM). ADAPT 추가 기여: (i) glideslope 콘을 다면체(3 half-space 피라미드)로 근사, (ii) 추력각 시변 한계 θ_T,max(t_k), (iii) ZOH/FOH 이산화에서 추력 보정계수 ρ_k=(1-e^{-α||ā_k||Δt})/(α||ā_k||Δt) 도입(질량 독립, 식 7), (iv) tf 격자 탐색을 보간으로 간소화, (v) 계산 지연 보상용 state projection.

**3. 비판적 검토.**
- 핵심 lossless-convexification 정리는 선행 연구(refs 5,9,15)에서 그대로 가져왔고, 본 논문에서 추가한 두 제약(시변 추력각, 추력 변화율)은 "lossless convexification 증명 없이" 채택되었다고 명시한다 — 이론적 신규성은 제한적이고 본질적으로 구현·시연 논문이다.
- baseline 비교가 약하다. polynomial PDG와의 비교는 정성적(PMF 0.48 vs 0.22, divert aspect ratio 0.5 vs 2.8)이며, 동일 시나리오에서 외부 경쟁 기법과 head-to-head 정량 비교는 없다.
- glideslope 제약은 실비행에서 한 번도 활성화되지 못했고(자원·안전 제약), 시뮬레이션으로만 검증되었다 — 형식화된 제약군 중 일부는 비행 검증 공백.
- 이산화 내부절점 위반(속도 <0.2%, 추력 하한 0.3~0.8%)은 작지만 "soft" 제약으로 처리·완화로 흡수했을 뿐 보장된 상한은 아니다.
- GPS 의존(센티미터급 항법)으로 미터급 정밀도를 달성했는데, 실제 화성/유로파에는 GPS가 없어 항법 정밀도가 직접 이전되지 않는다(저자도 인정, Doppler radar 필요 언급).

**4. 시뮬/실험.** 3년간 8회 실비행(real): year1 지상계획 basic divert 3회(500/650/750 m, 16.7~24.7 m/s), year2 온보드 basic/stress divert(750 m, 800 m, aspect ratio 2.8, 방향반전), year3 온보드 lofting divert 2회(300 m, TRN 연계). 온보드 G-FOLD 5회. 추종 정밀도: 위치 ±1 m, 속도 ±0.5 m/s (real), peak 위치오차 3.1~3.5 m. 온보드 런타임 평균 100 ms(<5 ms 변동), tf 최적화 ~17 ms, solver ~16 ms/최적화. 검증: ASIM 기반 1000-run MC로 비행 전 추종오차 예측이 실측과 정량 일치(Fig.13 vs Fig.20), 입력 10배/10% 손상 100,000+회 시뮬에서 항상 유효 궤적 반환. Table 1: G-FOLD 동역학 적분이 ρ_k 보정 시 최종 위치/속도/질량 오차를 기계 정밀도(~1e-8 m, 1e-11 m/s)로 검증. baseline: polynomial PDG 정성 비교.

**5. Contribution(delta).** 출판년(2017) 기준 SOTA: convexified PDG(G-FOLD) 이론과 IPM solver는 확립되었으나 모두 시뮬레이션 검증에 머물렀다. 본 논문의 delta는 (1) 제약 충족·연료 최적 대형 divert를 실 로켓에서 온보드 실시간 생성·비행한 최초의 실험적 입증, (2) MATLAB→C 구현 및 비행 SW 아키텍처(PAC/OFMS/ACS), (3) 이산화·내부절점 위반·ZOH/FOH 추력 보정(ρ_k)·tf 보간 등 실배치에 필수적인 엔지니어링 분석. 새 이론이 아니라 그 이론의 비행 검증 capstone이라는 점에서 G&C 분야에 결정적이다.

**6. Limitation·Future work.** 저자: glideslope 제약 비행 미활성, GPS 항법 의존(실미션엔 Doppler radar 필요), 두 신규 제약의 lossless 증명 부재(companion paper로 이연), 더 복잡한 추력 프로파일 시 state-projection 오차 증가. 내가 보는 한계: 외부 baseline 정량 비교 부재로 "convex가 polynomial보다 낫다"는 주장은 PMF/aspect-ratio 수치에 기대지만 동일 조건 직접 대결은 없음; 화성/유로파 환경(대기·중력·항법)과 Xombie 시연 조건의 차이가 커서 추종 정밀도의 미션 이전성은 제한적; 신규성이 구현에 집중되어 이론적 기여는 companion 논문들에 분산됨.

**평가:** 85/100 (flagship) · 한줄평: convexified PDG(G-FOLD)를 실 로켓에 온보드 시연한 최초의 비행 검증으로, 이론 신규성은 제한적이나 정밀 착륙 G&C의 실증적 정점이자 분야를 형성한 작업이다.

---

## (2018, AIAA GNC, Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time: Szmuk·Açıkmeşe)

**1. 핵심 아이디어.** 단일 짐벌 엔진으로 제어되는 6-DoF 강체 로켓의 최소시간 동력하강(powered descent) 문제를, 비행시간 t_f 자체를 최적화 변수로 두는 자유종단시간(free-final-time) 비볼록 최적제어 문제로 정식화한다. 시간 정규화(time-dilation)와 1차 테일러 선형화로 비볼록성을 제거하고, 이를 이산화하여 SOCP 부분문제 수열로 변환한 뒤 successive convexification으로 반복 수렴시킨다. 저자들의 선행 3-DoF 자유종단시간 및 6-DoF 고정종단시간 연구를 6-DoF 자유종단시간으로 확장한 것이 기여이다.

**2. 알고리즘.** 상태 x=[m, r_I, v_I, q_B/I, ω_B] (R^14), 제어 u=T_B. 정규시간 τ∈[0,1] 도입 후 dilation σ=(dτ/dt)^{-1}로 x'(τ)=σ f(x,u). 기준궤적 (x̂,û,σ̂) 주위 1차 선형화: x'(τ)=A(τ)x+B(τ)u+Σ(τ)σ+z(τ). 유일한 비볼록 제약인 추력 하한 T_min≤||T_B||를 선형화하여 T_min≤B_g(τ)u, B_g=û^T/||û||로 볼록화. 상태천이행렬 Φ_A로 이산화(first-order-hold)하여 x_{k+1}=Ā_k x_k+B̄_k u_k+C̄_k u_{k+1}+Σ̄_k σ+z̄_k. 비용 t_f→σ를 최소화하되, 신뢰영역(δx·δx+δu·δu≤Δ_k, ||δσ||≤Δ_σ)과 가상제어 ν_k(인공 비실현성 방지, w_ν=1e5로 강패널티)를 추가. 글라이드슬로프 콘, 틸트각(cosθ_max≤1−2(q2²+q3²)), 각속도, 짐벌각 제약은 모두 볼록 형태로 유지. 종단에서 T를 X축 정렬시켜 토크를 0으로 만든다.

**3. 비판적 검토.**
- 수렴성·최적성 보장은 본 논문이 직접 증명하지 않고 동저자들의 SC 이론 논문(Mao·Açıkmeşe 등)에 위임한다. 본문은 정식화·시연 중심으로, 신뢰영역/가상제어가 수렴을 담보한다는 형식적 명제가 없다.
- 검증이 무차원 notional 시나리오 2개(2D 평면, 3D 면외)뿐이고 외부 baseline·몬테카를로·실제 화성 파라미터가 없다. 저자도 "실제 시스템과 일치시키려는 의도가 아님"을 명시한다.
- 실시간성을 핵심 동기로 내세우면서도 onboard 타이밍 통계가 전혀 없다(향후 과제로 미룸). IPM/SOCP 구조상 타당하나 본 논문에서 입증되지 않았다.
- 공기역학을 무시(negligible)하여 화성 진입 후반·대기 효과를 다루지 못한다. 저자도 향후 단순 공력 도입을 언급한다.
- t_f 초기추정 강건성(10개 초기값 동일 수렴)은 좋은 결과지만, 이는 알고리즘 강건성이지 외란/불확실성 강건성이 아니다.

**4. 시뮬/실험.** MATLAB+CVX+SDPT3, K=50 이산점, 최대 15회 반복(sim). 2D 면내 기동과 3D 면외 기동 두 시나리오. 추력은 bang-coast-bang(T_min/T_max 모두 활성), 틸트각·각속도 한계 saturation 관찰. t_f 초기추정 1.0~10.0을 1.0 간격 10개로 스윕 → 2D는 6회, 3D는 9회 반복 내 모두 동일 t_f(편차 0.01 이내)로 수렴. baseline 비교·MC·정량 오차통계는 없음.

**5. Contribution(delta).** 선행 대비 delta는 6-DoF 동력하강 SC 프레임워크를 고정종단시간에서 자유종단시간으로 확장한 것. 시간 dilation σ를 최적화 변수로 직접 다루어 t_f에 대한 line search 없이 최소시간 해를 얻고, 이산 LTV 행렬을 정확히 계산해 수렴해가 원래 비선형 동역학을 정확히 만족시킨다. 2018년 SOTA 기준으로 듀얼쿼터니언 LoS 볼록화(piecewise-affine 근사로 정확도 저하, 고정시간 한정) 대비 명확한 진전이며, 이후 정전화된 Szmuk/Açıkmeşe 6-DoF SCvx 계열의 토대 정식화 중 하나.

**6. Limitation·Future work.** 저자: 수렴 특성 추가 연구, 실시간 타이밍 통계 제시, 단순 공력 효과 6-DoF 통합. 추가로 내가 보는 한계: (a) notional 무차원 검증으로 화성 임무 도메인 grounding 부족, (b) 외란·항법오차 하 강건성/불확실성 분석 부재(C4·C5 약점), (c) 본 논문 자체로는 수렴/최적성 보장이 외부 인용에 의존, (d) onboard 실시간성 정량 미입증.

**평가:** 72/100 (good) · 한줄평: 6-DoF 자유종단시간 successive convexification을 깔끔히 정식화·시연한 핵심 powered-descent G&C 논문이나, 무차원 시연·baseline/MC 부재·타이밍 미보고로 검증 깊이가 아쉽다.

---

## (2022, IEEE CSM, Convex Optimization for Trajectory Generation: A Tutorial on Generating Dynamically Feasible Trajectories Reliably and Efficiently: Malyuta et al.)

**1. 핵심 아이디어.** 비볼록 궤적생성 최적제어 문제를 볼록 최적화로 신뢰성·효율적으로 푸는 세 가지 주요 방법(무손실 볼록화 LCvx, 순차 볼록 프로그래밍 SCvx, 보장형 순차궤적최적화 GuSTO)을 한 편의 튜토리얼로 통합 정리한다. LCvx 개발자 그룹(Acikmese 등) 자신이 저자로서, 행성 착륙·로켓 착륙·드론·우주정거장 자유비행 로봇 등 실제 응용을 관통하는 통일된 관점과 오픈소스 SCP Toolbox(Julia)를 제공한다. 핵심 메시지는 비볼록성을 볼록 솔버가 다룰 수 있는 형태로 '재정식화'하는 것이 온보드 실시간 궤적생성의 열쇠라는 것이다.

**2. 알고리즘.** (i) LCvx: 입력 하한·포인팅 등 비볼록 입력제약을 슬랙 입력 σ로 'lifting'하여 볼록화하고(예: ρ_min ≤ σ ≤ min(σ,ρ_max), LCvx 등식제약 g(u) ≤ σ), Pontryagin 최대원리로 완화 문제의 전역최적해가 원문제에서 등식으로 활성화됨을 Condition 1-8 + Theorem 1-7로 보장. (ii) SCvx: 비볼록항을 기준궤적 주변 1차 선형화(43)→trust region(45)으로 artificial unboundedness 처리, virtual control(46)로 artificial infeasibility 처리, 정확도지표 ρ=실제개선/예측개선(61)으로 신뢰영역 갱신(Fig.16), KKT 기반 초선형 수렴. (iii) GuSTO: 동일 골격을 연속시간 Pontryagin 원리로 분석, trust region·제약을 soft penalty(h_λ, 식69-72)로 처리, ρ를 동역학·비용 선형화오차의 정규화 합(82)으로 정의. 3-DoF 착륙은 z=ln(m), u=T_c/m 변수치환(97)으로 SOCP화.

**3. 비판적 검토.**
- 명시적으로 튜토리얼/서베이로서, Theorem 1-9의 증명은 모두 저자들의 선행 논문([45],[86],[92]-[94],[99],[109],[49],[50],[72] 등)으로 외주되어 본문 자체의 독자적 정리 증명은 없다. 기여는 '새 방법'이 아니라 '통합·교육·재정식화'에 있다.
- 세 알고리즘 비교가 모두 저자 그룹 자신의 방법(SCvx vs GuSTO)끼리이며, NLP/SQP/직접법 등 외부 baseline과의 정량적 head-to-head(수렴률·해 품질·런타임)는 제시되지 않는다. SQP/SLP의 단점은 서술적으로만 다뤄진다.
- 불확실성·강건성은 본질적으로 범위 밖이다. 모든 예제가 공칭(deterministic)이며, 외란 w는 식에 등장하나 MC/강건 보장 검증은 없다. 'reliably'라는 제목의 신뢰성은 수렴 보장·전역성에 대한 것이지 외란 강건성이 아니다.
- LCvx의 한계가 솔직히 드러난다: 일반 볼록 상태제약은 활성구간이 비영(Fig.S6(b))이면 무손실성이 깨지고(Theorem 5의 isolated activation 조건 필요), 비선형 동역학은 PWA 근사+MIP에 의존(Theorem 6, Condition 7는 일반적으로 검증 곤란).

**4. 시뮬/실험.** 세 개의 상세 수치예제(모두 sim, SCP Toolbox로 재현 가능). (a) 3-DoF 연료최적 로켓 PDG: g=-3.71 m/s², m_dry=1505 kg, ρ∈[4.97,13.26] kN, t_f*=75 s, glideslope·포인팅·속도제약 하 전역최적해, LCvx 등식이 tight함을 Fig.22로 검증, Masten Xombie 비행시험([37])·Falcon 9·New Shepard 헤리티지 인용. (b) 쿼드로터 장애물회피(SCvx vs GuSTO): 30노드 FOH, ECOS 솔버, Intel i5-7200U에서 15회 반복 수렴, 50회 실행 런타임 통계, 유사 문제 임베디드 40 ms 인용. (c) 6-DoF 자유비행 로봇(ISS 유사 환경): 13차원 상태+쿼터니언, SDF softmax 비볼록 비행공간, vanishing-gradient 이슈와 terminal cost(161) 해법 제시. baseline은 내부 두 방법 비교(외부 baseline 없음).

**5. Contribution(delta).** 선행 대비 delta는 (1) LCvx·SCvx·GuSTO를 단일 우산 아래 처음으로 통합 서술하고 공통/차이를 체계화한 점, (2) embedded LCvx, 고정종료시각 LCvx([109]), 일반 볼록상태제약·비선형동역학으로의 확장을 교육적으로 정리한 점, (3) 완전 재현 가능한 오픈소스 도구와 3개 신규 응용 정식화(특히 SDF 기반 6-DoF 비행공간)를 제공한 점이다. 2022년 SOTA 기준 novelty는 '새 G&C 방법' 축에서는 점진적(구성요소가 이미 출판됨)이나, 통합 참조문헌·재정식화 가이드 축에서는 분명한 가치가 있다. 원조 LCvx(Acikmese & Ploen)의 field-shaping에 비해 본 논문은 그 방법론을 집대성한 정전(canon)이다.

**6. Limitation·Future work.** 저자가 인정: LCvx 가능 문제군이 좁고(affine 상태제약은 2014년 수준), 일반 비볼록성 확장은 미해결 연구과제이며 'modify first, prove later' 권고; SCP는 국소최적·재계획 시 recursive feasibility 미보장; vanishing gradient 등 수치 민감성. 내가 보는 한계: 외부 baseline 부재로 SCP의 우위가 정량적으로 입증되지 않음; 강건성·불확실성 전파가 본질적으로 다뤄지지 않아 실비행 환경의 외란·모델오차 대응은 후속 과제; 모든 정리가 외주되어 단독으로는 이론적 자기완결성이 없음.

**평가:** 80/100 (strong) · 한줄평: 증명은 외주된 튜토리얼·종합이라 자체 novelty/rigor는 flagship에 못 미치나, 로켓 착륙 PDG를 중심으로 한 핵심 G&C 도메인의 사실상 정전 참조문헌이며 오픈소스·실시간성·인용영향력(≈60/yr) 모두 최상급이다.

---

## (2024, arXiv, Remarks on "Successive Convexification: A Superlinearly Convergent Algorithm for Non-convex Optimal Control Problems": Luo·Elango·Açıkmeşe)

**1. 핵심 아이디어.** 본 노트는 비볼록 이산 최적제어용 SCP 알고리즘인 SCvx(Mao et al. 2018)의 강수렴(strong convergence) 증명에 존재하는 오류와 모호성을 식별하고, 더 엄격한 가정 하에서 강수렴을 새 방법으로 재증명한다. 원논문의 약수렴(부분수열 수렴)에서 전체 수열 수렴으로 넘어가는 핵심 보조정리(Lemma 3.11) 적용의 오류를 지적하고, 목적함수의 콤팩트 레벨셋 가정을 도입해 우회 증명을 제시한다. 별도의 새 알고리즘이나 시뮬레이션 없이 순수 이론적 정오표(remarks/errata) 성격의 짧은 노트다.

**2. 알고리즘.** 목적함수 J(z)=ψ(G(z)), ψ는 볼록·G는 C^1. 가정 1: {z | J(z)≤J(z^0)}가 콤팩트. 핵심 보조결과 — Lemma 4.5: z̄ 근방 N(z̄,δ)에서 J(z)−J(z̄) ≥ β‖z−z̄‖; Lemma 4.6: ψ(G(z̄)+∇G(z̄)^T d) ≥ ψ(G(z̄))+γ‖d‖. Theorem 3: z^k이 z̄에 충분히 가까우면 신뢰영역 부분문제 해 d^k는 ‖d^k‖<ε를 만족(Lipschitz 연속성 + Lemma 4.6 결합). Theorem 4: 이로부터 ‖z^{k+1}−z̄‖ ≤ (J(z^{k+1})−J(z̄))/γ 와 ‖z^{k+1}−z̄‖<‖z^k−z̄‖+‖d^k‖≤δ를 귀납적으로 적용해 전체 수열 lim z^k=z̄ 증명. Zhang [4] Theorem 2.1을 인용하면 초선형 수렴율까지 회복.

**3. 비판적 검토.**
- 기여가 본질적으로 정오표(correction)다: 새 G&C 방법·정식화가 아니라 기출판 증명의 오류 수정과 더 강한 가정 하 재증명이라, 독창적 델타는 얇다.
- 강수렴을 원논문의 약한 가정 그대로 복원하지 못하고 콤팩트 레벨셋·LICQ 등 더 엄격한 가정을 요구한다 — 저자 스스로 "완전한 수정은 아니다"라고 인정.
- 핵심 기계(Lemma 4.5/4.6, Theorem 3.13, Zhang [4]의 초선형율)는 외부에서 그대로 수입하며, 새로 증명한 부분은 전체수열 수렴 논증에 국한된다.
- Remark 5에서 Assumption 4.2(|I_ac|+|J_ac|≥n_u(N−1))가 LICQ와 결합 시 등호를 강제하여 실무에서 만족하기 어렵다는 점을 정직하게 지적하나, 이 한계 자체에 대한 완화책은 제시하지 않는다.
- 시뮬레이션·수치예제가 전무하여 수정된 가정이 실제 동력하강/궤적최적화 문제에서 얼마나 제약적인지 정량 검증이 없다(설계상 이론 노트이므로 감점 요소라기보다 범위 한정).

**4. 시뮬/실험.** 없음(설계상 순수 이론 노트). 수치 시나리오·몬테카를로·baseline 비교 일절 없음. 검증은 전적으로 수식 유도(Theorem 3, Theorem 4의 귀납 증명)로 대체.

**5. Contribution(delta).** 출판년(2024) 기준, 널리 채택된 SCvx 강수렴 증명의 구체적 오류 7건+α(Definition 3.10, Theorem 3.9의 KKT vs local minimizer, eq.(3.19) 부호, Bolzano-Weierstrass 적용, eq.(3.33)/Lemma 3.11 오용 등)를 명시하고, 콤팩트 레벨셋 가정 하에서 전체수열 강수렴을 재정립한 점이 델타다. 새 방법론이 아닌 기존 결과의 이론적 정합성 보강이며, 초선형율은 Zhang [4] 인용으로 회복.

**6. Limitation·Future work.** 저자: (i) 원논문의 약한 가정 하 강수렴은 미해결로 남김, (ii) 초선형율에 필요한 가정이 실무에서 드묾, (iii) Assumption 4.2의 등호 요구가 이산화·Bang-Bang 상황에서 비현실적. 추가로 본인이 보는 한계: 수치 검증 부재로 수정된 가정의 보수성을 정량화하지 못하며, 기여 범위가 단일 알고리즘의 증명 교정에 국한되어 G&C 방법론 자체의 전진은 아니다.

**평가:** 58/100 (fair) · 한줄평: 동력하강·실시간 SCP 유도의 기반인 SCvx 수렴 이론의 오류를 정직하고 엄밀하게 교정한 가치 있는 정오표이나, 새 방법·검증이 없는 control-theory 인접 노트로 G&C 직접 기여는 간접적이다.

---

## (2021, arXiv, SCvx-fast: A Superlinearly Convergent Algorithm for A Class of Non-Convex Optimal Control Problems: Mao·Açıkmeşe)

**1. 핵심 아이디어.** SCvx 후속 컨벡스화(Successive Convexification) 프레임워크의 확장으로, 신뢰영역(trust region)에 의존하지 않고 비볼록 최적제어 문제를 푸는 SCvx-fast 알고리즘을 제안한다. 핵심은 각 제약과 (정확 벌점화로 부등식화한) 동역학을 현재점의 투영점에서 선형화하는 project-and-convexify 절차이며, 이를 통해 보수성을 줄이면서 더 큰 스텝을 취해 초선형(superlinear) 수렴을 달성한다. 비실현(infeasible) 초기점에서 한 스텝 만에 실현영역으로 복귀하는 전처리 루틴도 포함한다.

**2. 알고리즘.** 이산 최적제어 문제를 단일변수 y로 묶고, 동역학 등식 g(y)=0을 정확 벌점함수 P(y)=J(y)+λ||g(y)||_1 로 목적함수에 흡수(λ≥||μ̄||_∞ 일 때 정확성 보장). 키프아웃/동역학 제약을 q_j(y)≥0 으로 통합. 각 반복에서 (1) 현재점 z를 각 제약 경계로 투영하여 투영점 z̄_j = argmin{||z−y||_2 | q_j(y)≤0} 획득(Hilbert 투영정리, 유일성), (2) 그 투영점에서 선형화 l_j(y,z)=∇q_j(z̄_j)(y−z̄_j) (비매끄러운 경우 z−z̄ 에 직교하는 지지초평면 사용), (3) 컨벡스 하위문제 z^(k+1)=argmin{P(y) | y∈F_z^(k)} 를 SOCP로 풀어 갱신. 신뢰영역 갱신 메커니즘 없음.

**3. 비판적 검토.**
- 핵심 가정 2.1(동역학 성분 f_j 가 x,u 에 대해 볼록)이 상당히 제한적이다. 저자는 이중적분기·상수속도 항력을 예로 들지만, 일반 비선형 6-DoF 동역학·회전운동 등 다수 G&C 문제는 이 가정을 만족하지 않아 적용범위가 좁다.
- 초선형 수렴은 정리 4.2로 증명되나, 초록은 "supporting both claims"라 표현하고 증명에 일반화 자코비안·o(·) 논거와 G(·)∈C^1 가정이 섞여 있어 비매끄러운 제약 하에서의 rate 주장과의 정합성이 다소 느슨하다.
- 수치검증이 단일 시나리오(다중로터 장애물 회피)에 그치고 외부 baseline·몬테카를로가 없다. 전신(SCvx) 대비 속도 우위를 정량 비교하지 않고 수렴 이력 그림으로만 제시한다.
- 실시간/온보드 적용을 주장하나 실제 타이밍·임베디드 벤치마크가 없고, "sub-millisecond 투영"은 일화적 언급이다.

**4. 시뮬/실험.** 3-DoF 다중로터 최소연료 장애물 회피(이중적분기 동역학, N=20, t_f=15s, u_max=13.33 m/s², 추력원뿔 30°, 타원체+다면체 장애물). 비실현 초기궤적에서 Algorithm 1로 한 스텝 실현화 후 11개 SOCP를 순차적으로 풀어 수렴. 비용 Σ||u_i|| 가 초기 282.41 → 수렴해 221.76 로 감소(sim). 수렴 이력 그림이 초선형 패턴(후반 급가속) 제시. 외부 baseline·MC·통계 없음, 자기-baseline(초기궤적) 뿐.

**5. Contribution(delta).** 선행 SCvx(Mao 2016/2017) 대비 세 가지 delta: (i) 비실현 초기점 1-스텝 복귀 전처리, (ii) 제약의 매끄러움 조건 제거로 비매끄러운 다면체 키프아웃 포함, (iii) 신뢰영역 제거 + 초선형 수렴 rate 증명. 출판년(2021) 기준 SCP 계열에서 trust-region-free 초선형 수렴 증명과 project-and-convexify 투영 선형화는 분명한 이론적 진전이며, 방향을 새로 여는 수준은 아니나 최근접 선행 대비 명확한 개선이다.

**6. Limitation·Future work.** 저자: 가정의 실용적 제약(LICQ/Slater 실패 사례), 보수성 도입 불가피. 내가 보는 한계: 동역학 볼록성 가정으로 일반 비선형 G&C로의 일반화가 막혀 있고, arXiv 프리프린트로만 남아 정식 심사·후속 검증이 없으며(5년 3회 인용, 0.6/yr), 단일 예제·무 baseline으로 SCvx 대비 속도 우위가 정량 입증되지 않았다. 실제 온보드 타이밍 측정과 다양한 시나리오 MC 검증이 필요하다.

**평가:** 70/100 (good) · 한줄평: SCP 계열에 trust-region-free 초선형 수렴 증명을 더한 견실한 이론적 기여이나, 제한적 동역학 가정·단일 예제·무 baseline·낮은 실측 영향으로 G&C 적용 폭과 임팩트가 좁다(near-domain 제어이론).

---

## (2023, CDC, Successive Convexification with Feasibility Guarantee via Augmented Lagrangian for Non-Convex Optimal Control Problems: Oguri)

**1. 핵심 아이디어.** 비볼록 최적제어를 푸는 SCvx(순차 볼록화)는 KKT 점으로의 수렴은 보장하나 원 문제(비볼록)에 대한 실현가능성(feasibility) 보장이 없고, "충분히 큰" 페널티 가중치를 시행착오로 찾아야 하는 한계가 있다. 본 논문은 SCvx 반복을 증강 라그랑지안(AL) 기법의 원-쌍대(primal-dual) 틀에 통합한 SCvx*를 제안한다. 최적화 변수 z뿐 아니라 라그랑지 승수 λ,μ까지 함께 갱신하여(ADMM 정신), 페널티 가중치 수동 튜닝 없이 원 문제의 feasible 국소최적해로의 강한 전역수렴과 승수의 선형/초선형 수렴률을 이론적으로 확보한다.

**2. 알고리즘.** AL 함수 L_w(z,λ,μ)=f0(z)+λ·g_all+ (w/2)g_all·g_all + μ·[h_all]_+ + (w/2)[h_all]_+·[h_all]_+ 를 페널티 관점 J(z)=f0(z)+P(g,h,w,λ,μ)로 재해석. 각 반복에서 비affine 제약 g,h를 기준점 z̄ 주위로 선형화(g̃,h̃)하고 인공 비실현성 회피를 위해 가상제어 ξ, 가상버퍼 ζ로 완화(g̃=ξ, h̃≤ζ)한 볼록 부분문제(Problem 4)를 신뢰영역 ‖z̄−z‖∞≤r 하에서 푼다. 실제/예측 감소 ΔJ, ΔL로 스텝수용(ρ=ΔJ/ΔL≥ρ0)을 판단하고, 수용 시 승수 갱신 λ←λ+w·g_all, μ←[μ+w·h_all]_+, w←βw (β>1). 핵심으로 ΔL의 재정의(Remark 1)로 ΔL≥0을 보장(Lemma 3)하여 수용·정상성을 SCvx로부터 상속. 승수 갱신 기준을 |ΔJ|≤min{δ, ηχ}로 강화하면 초선형 수렴(Prop. 2 of [22]) 달성.

**3. 비판적 검토.**
- 이론은 견고하나 핵심 기계는 대부분 인용/수입에 의존한다. Lemma 1(AL 수렴)은 Bertsekas의 Prop. 2.14/3.1/3.2를 직접 적용, 다수 보조정리가 "[2]의 증명을 따른다"로 전개된다. 즉 SCvx와 고전 AL 이론의 결합이며 새 증명 부담은 제한적이다.
- 외부 독립 baseline이 부재하다. 비교 대상이 본인이 확장한 SCvx 단 하나로, GuSTO 등 경쟁 SCP나 직접 NLP(SNOPT/IPOPT) 비교가 없어 우월성 주장이 self-baseline에 머문다.
- Monte Carlo·통계 처리·실시간/온보드 타이밍 분석이 전무하다. 결과는 반복횟수(예: 33 vs 비수렴)와 수렴 곡선뿐이며 G&C 적용의 핵심인 계산시간·임베드 가능성 논의가 없다.
- 실현가능성 "보장"은 Assumption 1(국소해·LICQ·2차 충분조건)과 AL의 점근적 정확 최소화에 강하게 의존한다. 실제 비볼록 문제에서 이 가정 충족 여부는 검증되지 않는다.

**4. 시뮬/실험.** 2개 수치예제(모두 sim, CVX+Mosek). (1) 2D 비볼록 toy 문제(crawling 현상): SCvx*는 모든 w^(1)에서 수렴(예 w=1일 때 33회), SCvx는 w=10^-1·10^0·10^3~10^5에서 N/A(비수렴). (2) 쿼드로터 경로계획(SCvx 문헌 [1]에서 차용, 6상태/4제어, N=31, t_N=5s, 장애물 2개): w^(1)=10^4에서 SCvx* 11회 vs SCvx 15회 수렴, w=10^-1에서 SCvx는 비수렴(N/A)인 반면 SCvx*는 w값과 무관하게 일관 수렴. baseline은 SCvx 단일.

**5. Contribution(delta).** 출판년(2023) SOTA인 SCvx[1,2]/GuSTO[8] 대비 명확한 delta: (i) 원 비볼록 문제에 대한 feasibility 보장을 SCP에 부여(SCvx에 없던 속성), (ii) 페널티 가중치 수동 튜닝 제거(승수 자동 갱신으로 대체), (iii) GuSTO가 요구하는 control-affine 제약 없이 최소 가정으로 작동, (iv) 승수의 선형/초선형 수렴률 분석 제공. SCvx 반복을 AL 원-쌍대 틀에 통합한 첫 포괄적·엄밀 수렴분석이라는 점에서 incremental을 넘는 방법론적 진전이나, 새 방향을 여는 수준은 아니다.

**6. Limitation·Future work.** 저자: 수렴률은 승수 반복에 대한 것이지 반복지수 k에 대한 것이 아님, w_max 유한 시 선형 수렴에 그침을 명시. 내가 보는 한계: 불확실성/강건성 처리 부재(본 논문은 결정론적; 강건 설계는 예비판 [21]로 분리), 외부 baseline·MC·온보드 타이밍 부재, 가정 의존적 보장, 단일 저자 소규모 검증. Future: 대규모 6-DoF 동력하강 등 실제 G&C 시나리오에서 계산비용·실시간성 검증과 GuSTO/직접 NLP 대비 정량 비교가 필요.

**평가:** 71/100 (good) · 한줄평: SCvx에 AL 기반 실현가능성 보장을 더한 이론적으로 탄탄한 SCP 개선이나, 검증이 toy+자기-baseline에 머물고 온보드/불확실성 논의가 없어 G&C 직접 적용 근거는 아직 제한적인 near-domain(control-theory) 기여.

---

## (2023, IFAC, Real-Time Sequential Conic Optimization for Multi-Phase Rocket Landing Guidance: Kamath et al.)

**1. 핵심 아이디어.** 다상(multi-phase) 로켓 착륙 유도 문제를 혼합정수/비볼록 제약 없이 단일 최적화로 풀기 위한 sequential conic optimization(SeCO) 패러다임을 제안한다. 행렬 분해·역행렬이 전혀 없는 1차(first-order) 원추(conic) 최적화(PIPG)와 순차 볼록 계획(SCP, PTR)을 결합하여 비선형 동역학·자유 전이시간·자유 종단시간 문제를 실시간으로 푼다. Starship형 4상(coast/high-thrust/low-thrust/terminal) 착륙 시나리오로 실증하며 ECOS 대비 2.7배 빠른 13.7 ms 실행시간을 보고한다.

**2. 알고리즘.** (1) time-interval dilation: 각 부구간 [t_k, t_{k+1}]을 [0,1)로 정규화하는 아핀맵 τ_k(t)와 dilation factor s_k = t_{k+1}^- - t_k를 결정변수로 두어 free-transition-time·free-final-time을 단일 샷에 인코딩(Eq.2-3). (2) STM 기반 exact discretization으로 연속/이산 궤적 오차가 해석적으로 0(Eq.6-11). (3) single-crossing compound STC: 트리거 함수 g가 단조·단일교차이면 트리거를 waypoint(Eq.14b)로 부과해 STC 전체가 볼록(기존 Szmuk/Reynolds의 비볼록 STC와 대비). (4) virtual state ξ로 인공 비가용성 처리(동역학 manifold·원추 집합 형상 보존, Prop.1로 페널티 볼록성 증명). 종단 부분문제는 SOCP min ½z^T Q z + <q,z> s.t. Hz=h, z∈D를 xPIPG(Eq.17, 외삽 가속·ℓ2 전처리)로 warm-start 반복.

**3. 비판적 검토.**
- 분산/몬테카를로 없는 단일 명목 시나리오만 제시. 초기조건 산포, 항력계수 불확실성, 추력/김벌 외란 하 강건성 검증이 전무하여 폐루프 정밀착륙 주장(C4 약점)을 뒷받침하지 못한다.
- 폐루프가 아닌 개루프 피드포워드 궤적 생성에 그친다. terminal phase를 "tight tracking 가능하게" 설계했다고 하나 피드백 추종 실험은 future work로 미룬다.
- 검증 동역학이 평면(planar) 3-DoF 수준이며 공력모델도 단순 다항식. 본 프레임워크의 6-DoF 확장(저자 자신의 dual-quaternion 라인)이 본 논문에서 직접 입증되지 않는다.
- SeCO/PIPG의 수렴 보장은 자기인용(Yu et al.)으로 import되며 SCP 전체 수렴/exactness 정리는 본문에서 재증명되지 않는다(이론적 완결성은 Prop.1 한 개에 국한).
- 일부 제약(글라이드슬로프 box화, δ_maxTD 추가 강화)이 closed-form projection을 위한 ad-hoc tightening으로 정당화가 경험적("works well in practice").

**4. 시뮬/실험.** N=16 노드(coast 1 + 3상 각 5)의 단일 다상 착륙 시나리오. 초기 1 km 고도·86.28 m/s, 100회 반복 평균 PIPG 실행시간 13.7 ms로 ECOS 37.1 ms 대비 2.7배 빠름(ECOS는 MOSEK/GUROBI보다 빠르다고 명시)(sim). 수렴 해의 추진제 소비 차이 0.02%, 수렴 SeCO 반복 7회. 동역 490.34 m·86.28 m/s에서 powered phase 자동 점화 선택. 외부 솔버 baseline은 강하나 MC/산포 sweep·다중 시나리오·실/HIL은 없음.

**5. Contribution(delta).** 출판년(2023) SOTA 기준: (i) free-transition-time 다상 궤적을 혼합정수·비볼록 STC 없이 단일 샷으로 푸는 time-interval dilation, (ii) STC를 볼록화하는 single-crossing 공식화(Szmuk 2019/2020, Reynolds 2020b의 본질적 비볼록 STC 대비 명확한 진전), (iii) 동역학 manifold·원추형상을 보존하는 virtual state(virtual control/buffer 대비 개선), (iv) 행렬역 없는 1차 PIPG로 임베디드 친화 SOCP 솔버를 다상 로켓 유도에 통합. Lee&Lee(2022) 다상 SCP가 비볼록 STC·inexact(사다리꼴) 이산화를 쓴 것과 달리 exact 이산화+볼록 STC를 제공. pseudospectral 다상 대비 실시간성 우위.

**6. Limitation·Future work.** 저자: 평면 모델·명목 단일 해 제시, 6-DoF 및 폐루프 추종은 별도. 추가로 내가 보는 한계: 불확실성 정량화·MC 부재로 정밀착륙 강건성 미입증, 비행/HIL 미실증(타이밍만), 코드 비공개, single-crossing 가정이 일반 STC 대비 표현력 제한(저자도 generality trade-off 인정), 일부 제약 강화의 정당화가 경험적. Future: 6-DoF 통합, 산포 하 폐루프 검증, 임베디드 하드웨어 실증.

**평가:** 79/100 (strong) · 한줄평: 행렬역 없는 1차 원추 솔버로 자유전이시간 다상 착륙을 실시간(13.7 ms) 단일 샷으로 푸는 견고한 core-G&C 기여 — 볼록 STC·exact 이산화·온보드 타이밍은 우수하나 불확실성/MC 검증 부재가 명확한 약점.

---

## (2024, Astrodynamics, State-dependent trust region for successive convex programming for autonomous spacecraft: Bernardini et al.)

**1. 핵심 아이디어.** 자율 우주선 궤적 최적화를 위한 축차 볼록 최적화(SCVX)에서, 모든 변수에 동일하게 적용되던 균일 신뢰영역(trust region)을 비선형성 지표(NLI) 기반의 변수별·노드별 상태의존 신뢰영역으로 대체한다. 미분대수(DA, differential algebra)로 비선형 동역학을 고차 다항식 전개하여 각 결정변수에 대한 제약의 비선형성을 정량화하고, 비선형성이 큰 변수는 작은 신뢰영역, 작은 변수는 큰 신뢰영역을 자동 부여한다. 동시에 DA를 전사(transcription)·선형화 자동화 도구로 사용해 해석적 변분방정식이 없는 문제에도 적용한다. 이 알고리즘을 DACX로 명명한다.

**2. 알고리즘.** 시간자유 연료최적 문제를 τ∈[0,1] 시간고정 문제로 재정의(σ=t_f−t_0)하고 추력 제약을 무손실 볼록화(slack 변수, ‖u‖≤ū≤1)한다. DA 적분으로 각 노드를 다항식 맵 [z_{i+1}]=T^p(δz_i,δu_i,δσ)로 전개, 1차항을 추출해 A_i z_i − z_{i+1} + B_i u_i + c_i σ = −d_i 의 아핀 제약을 얻는다. 비선형성 지표는 NLI_j = (Σ|a_{ij,k} δz_j|)/(Σ φ̄_ij) = ν_{i,j}|δz_j| ≤ δ_s 로 정의되고, 여기서 상태의존 신뢰영역은 Δ_{i,j}=1/ν_{i,j} 로 설정된다(비선형 클수록 좁음). 인공 무한계·비실현성 방지를 위해 가상제어 ν1,ν2(벌점 λ)와 적응 스케일 δ_s를 도입하며, ρ=ΔJ/ΔL 비율로 δ_s를 확대(β)/유지/축소(1/α)한다.

**3. 비판적 검토.**
- 수렴 보장이 없다. 저자들도 SCVX의 강수렴성이 상실됨을 명시하며, NLI→Δ=1/ν 사상은 잘 동기화되어 있으나 수렴을 보존한다는 형식적 증명은 제시되지 않는다. δ_s=1, Δ=1/ν 같은 핵심 상수·사상 선택이 휴리스틱이다.
- 핵심 기계장치(DA, NLI 정의[Junkins&Singla 2004], 다항식 바운딩[Fossà 2022])가 대부분 인용·수입된 것으로, 본 논문의 자체 이론 기여는 이들의 SCVX 신뢰영역 결합과 자동화에 한정된다.
- 불확실성 모델링이 없다(결정론적 명목 최적화). '강건성'은 동역학 불확실성이 아니라 초기 신뢰영역·노드 수 무작위화에 대한 수렴 성공률로 해석되어야 한다.
- 온보드 주장 대비 검증은 데스크톱 MATLAB(i9)이며 DA 고차전개 비용·임베디드 실현성은 분석되지 않았다.

**4. 시뮬/실험.** 3개 문제군: (1) 진자 제어(2D 벤치마크), (2) Earth→Dionysus 저추력 전이(MEE, T_max=0.32N), (3) CRTBP 내 halo-to-halo 및 butterfly-to-butterfly 전이(혼돈 환경). 외부 baseline 3종: 간접법(연료질량 2719.34 vs 2720.07kg, 0.01% 차), 직접법 GPOPS-II(48s vs 74s; 진자 1.522/1.539s vs 2.24/3.41s), 표준 SCVX. 100회 무작위 MC(초기 TR 0.1–1, 노드 50–200): 수렴 성공률 DACX vs SCVX — Dionysus 98% vs 90%, halo 98% vs 39%, butterfly 86% vs 21%. 반복수·비용은 SCVX와 유사하거나 약간 우위, 핵심 우위는 robustness(수동 튜닝 제거·실패율 제거).

**5. Contribution(delta).** 표준 적응 신뢰영역(균일·전 변수 동일, Mao 2018) 대비 변수별·노드별로 제약 비선형성에 따라 자동 적응하는 신뢰영역을 도입한 것이 핵심 delta다. 출판년(2024) 기준 SCVX 신뢰영역은 균일 스칼라가 표준이었고, DA-NLI를 SCVX 신뢰영역 설계에 직접 연결한 것은 명확한 진보이나 새로운 방향을 여는 수준은 아닌 정련(refinement)이다. 부가 기여로 DA 기반 전사·선형화 자동화로 해석적 도함수 불필요화를 보였다.

**6. Limitation·Future work.** 저자: 단순 케이스에서 GPOPS-II가 더 다수의 국소해를 찾는 경우 존재, mass 신뢰영역 상·하한 클리핑 필요. 평가자: 형식적 수렴 보장 부재, 6-DoF/경로제약·상태제약 미포함, 불확실성·외란 하 강건성 미검증, 실제 임베디드 실시간 실증 부재, NLI→TR 사상의 최적성 미정립. Future: 수렴성 이론화, 임베디드/FPGA 타이밍, 불확실성 인지형 확장.

**평가:** 74/100 (good) · 한줄평: SCVX 신뢰영역 수동 튜닝이라는 실제 난점을 DA-NLI로 자동화한, 다중 외부 baseline·MC로 잘 검증된 견실한 core-GNC 정련 — 수렴 보장 부재와 점진적 delta가 상한을 누른다.

---

## (2025, Automatica, Continuous-time successive convexification for constrained trajectory optimization: Elango et al.)

**1. 핵심 아이디어.** 직접법(discretize-then-optimize) 궤적최적화가 본질적으로 겪는 inter-sample(노드 사이) 제약 위반 문제를 해소하고, 연속시간 제약만족(continuous-time constraint satisfaction)을 보장하는 SCP 기반 실시간 프레임워크 ct-SCvx를 제안한다. 경로제약을 외부 페널티 함수의 적분으로 변환해 보조 동역학(상태)으로 누적시키고, 주기적 경계조건을 부과하면 경로제약 만족이 적분값=0인 2점 경계값 문제와 등가가 됨을 보인다(Lemma 2, Corollary 3). 여기에 generalized time-dilation, multiple-shooting 이산화, l1-exact penalization, prox-linear 방법을 결합하여 수렴 보장을 갖춘 convex-composite 최소화로 푼다.

**2. 알고리즘.** 경로제약 g<=0, h=0 을 부드러운 외부 페널티 q_i,p_j 로 변환해 Lambda(t,x,u)=sum q_i(g_i)+sum p_j(h_j) 정의. 보조상태 y' = Lambda 를 추가하고 y(0)=y(t_f) 부과 → 적분이 0이면 경로제약 a.e. 만족. time-dilation으로 자유종단시간을 tau in [0,1] 고정문제로 변환, dilation factor s(tau)=dt/dtau 를 연속 제어입력으로 취급. multiple-shooting으로 노드 구간별 적분형 이산화(식 15a). LICQ 위반을 피하기 위해 제약을 epsilon-완화(식 17c). l1-exact penalty Theta_gamma (식 22)로 비볼록 제약을 penalize, indicator로 볼록 제약 처리. prox-linear(식 28-29)로 H∘G 를 선형화한 볼록 부문제를 반복(Algorithm 1), PIPG/ECOS로 해결.

**3. 비판적 검토.**
- 이론은 매우 충실하나(Lemma 2/6/12/13/20/23, Cor 3/18, Thm 10/15/16/21/22 모두 증명, Appendix에 pointwise 위반 한계 증명), 핵심 수렴정리 Thm 17은 Drusvyatskiy-Lewis(2018)에서 그대로 import한 것이고 normal cone/KKT 기계는 Clarke/Beck에서 가져온다. 저자가 정직히 인용하나 자체 증명은 아니다.
- 베이스라인이 'node-only'(자기 방법의 비-개정판) 내부 비교에 한정. 외부 경쟁 SCP(SCvx, GuSTO 등)나 collocation(GPOPS, ICLOCS) 대비 정량 비교가 없어 우월성 주장이 내부 대조에 의존한다.
- 페널티 적분 변환 자체는 1960년대(McGill 1965 등)부터 존재함을 저자도 인정. 신규성은 '이를 SCP+연속시간 가능성+수렴보장으로 통합'한 데 있으며, 단일 신개념은 아니다.
- epsilon 튜닝, gamma/rho 하이퍼파라미터, 제약 스케일링에 성능이 민감(저자도 Sec 7에서 인정). epsilon~1e-4 같은 경험적 선택이 1% 위반 보장의 근거.

**4. 시뮬/실험.** 3개 예제(모두 G&C): (i) 동적/정적 장애물 회피, (ii) 6-DoF 로켓 착륙(종단비용 ct-SCvx 180.5kg vs node-only 170.1kg — node-only가 inter-sample 위반 대가로 비용 낮음), (iii) 3-DoF lossless convexification 착륙(352.4kg vs 351kg). node-only 해는 확대 inset에서 glideslope/추력하한 inter-sample 위반을 명확히 보임(sim). 실시간성: C 코드(SCvxGEN+ECOS) 1000회 solve 평균 동적장애물 59.4ms(23 iter), 6-DoF 49.5/42.5ms(14 iter)(sim, 실측 타이밍). Monte Carlo는 타이밍 통계용이며 불확실성 검증은 아님. 외부 baseline 없음.

**5. Contribution(delta).** 출판년(2025) SOTA 기준: 이산시간 SCvx(Mao 2019), trust-region SCvx(Luo-Elango-Acikmese 2024), Oguri(2023) variant 대비, ct-SCvx는 (a) 희소 이산화 격자에서도 mesh-refinement 없이 연속시간 제약만족을 보장하고, (b) 재정식화된 경로제약이 표준 conic solver의 정준형으로 표현 불가능함을 규명하면서도 prox-linear의 exact penalization+수렴보장을 동시에 확보한 점이 명확한 delta. 볼록 문제에서는 더 강한 전역수렴(Thm 22)을 추가로 증명. lossless convexification 그룹의 정식 후속으로 연속시간 가능성을 SCP에 결합한 첫 일반 프레임워크.

**6. Limitation·Future work.** 저자: gamma/rho 튜닝, prox-linear 초기화, 제약 스케일링, 부문제 선조건화 필요; 연속시간 LCvx의 직접 적용(식 36)은 미해결. 추가로 내가 보는 한계: 불확실성/강건성 처리 부재(결정론적 명목 최적제어), 외부 경쟁법 대비 정량 벤치마크 결여, epsilon-위반 보장이 well-scaled 제약과 경험적 epsilon 선택에 의존. 최근 논문이라 G&C 채택 영향력은 잠재적 평가.

**평가:** 87/100 (flagship) · 한줄평: LCvx 원조 그룹의 정직하고 이론적으로 탄탄한 연속시간 SCP 프레임워크로, 실제 로켓착륙·장애물회피에 코드생성 실시간 배치까지 보인 high-quality core-G&C 기여(외부 baseline 부재가 유일한 큰 약점).

---

## (2026, Automatica, Revisiting Lossless Convexification: Theoretical guarantees for discrete-time optimal control problems: Luo·Echigo·Açıkmeşe)

**1. 핵심 아이디어.** Lossless Convexification(LCvx)은 입력 제약의 비볼록성을 슬랙 변수로 완화하여 비볼록 최적제어를 볼록 SOCP로 무손실 변환하는 기법이나, 기존 이론은 연속시간에서만 등가성이 보장되어 실제로 수치 해를 얻기 위한 이산화 단계의 보장은 비어 있었다. 본 논문은 이산시간 비볼록 최적제어에 LCvx를 직접 적용했을 때의 이론적 보장을 정립한다. 문제를 normal/long-horizon으로 분류하고, normal 경우에는 동역학에 임의로 작은 섭동(perturbation)을 가하면 비볼록 제약 위반이 최대 n_x − 1개 격자점으로 제한됨을, long-horizon 경우에는 bisection 탐색으로 동일 보장을 회복함을 증명한다.

**2. 알고리즘.** 이산 완화 문제(Problem 4): min m(x_{N+1}) + Σ l_i(σ_i) s.t. x_{i+1}=Ax_i+Bu_i, ρ_min ≤ σ_i ≤ ρ_max, g(u_i) ≤ σ_i, G(x_{N+1})=0. Lagrangian의 KKT 조건에서 dual η_i = A^{T(N−i)} η_N^*가 0이 아니면(=normal) Theorem 10에 의해 u_i가 V(σ_i)의 경계에 놓여 LCvx가 성립. η_N^*=0인 long-horizon은 σ_i=ρ_min으로 무너짐. Normal 보장: A를 Jordan 형태를 보존하는 섭동 Ã(q)=Q^{-1}J̃(q)Q로 치환(Problem 8)하고, q를 단위 큐브 εD 위 균등분포로 뽑으면 S(q)=[Ã^{P_1}B … Ã^{P_{n_x}}B]가 확률 1로 full rank → 위반점 ≤ n_x − 1 (Theorem 18, Vandermonde·귀납 논증). Long-horizon: [0,t_s] 구간 제어를 g(u)=ρ_min로 고정, t_s를 v(t_s) < m* + t_f·l(ρ_min) + ε가 되도록 bisection(가치함수 연속성 Theorem 8 활용)으로 결정 후 두 번째 위상에 normal LCvx 적용(Algorithm 1).

**3. 비판적 검토.**
- 핵심 보장(n_x − 1)이 "동역학에 인위적 섭동을 가한 뒤"의 확률 1 명제다. 저자 스스로 Remark 20에서 섭동은 실무에서 불필요하고 순수히 이론적 장치라 명시한다. 즉 무섭동 원문제에 대한 결정론적 보장이 아니라, 일반성(genericity) 논증에 가까워 "discrete-time LCvx가 성립한다"는 강한 주장과는 결이 다르다.
- n_x − 1개 위반이라는 상한은 제어차원이 1일 때만 tight하며, 위반점에서 추력 제약 위반은 실제 비행에서는 여전히 문제다. 저자도 보정 제어 û_i를 제안하나 이는 LCvx 본래의 "무손실" 정신을 벗어난 사후 투영이다.
- Theorem 21의 위반 영향이 O(t_f/N)로 감소한다는 결과는, 결국 "충분히 촘촘히 이산화하면 위반이 무시할 만하다"는 것으로, 본 논문이 제기한 문제의 실무적 심각성을 스스로 완화시킨다(Remark 22). 기여의 임팩트가 다소 이론 내적이다.
- long-horizon bisection은 switch time t_s를 바꾸며 horizon·이산화가 함께 변하므로 이산 Problem 3과 직접 비교 불가, 연속 Problem 1을 기준으로 우회 비교한다(설득력은 있으나 간접적).

**4. 시뮬/실험.** 3개 예제(모두 CVXPY). (1) 달 착륙 well-conditioned(질량 1000kg, T_max 7500N, 추력 [0.3,0.8]T_max, g=−1.62, 60s 비행): N=10에서 위반 1점, N=30·300에서 1점, N=50·500에서 0점 → 위반 수가 이산화 수준에 무관함을 확인(sim). (2) 인공 3차원 normal 예제(N=10, A=diag(1.2,−2.2,1), 음의 고유값으로 Theorem 18의 행렬을 특이하게 설계): 섭동 전 3점 위반 → 섭동 후 2점 위반으로 감소(sim). (3) long-horizon 달 착륙(N=20, t_f=200): 모든 |u|<ρ_min 확인, bisection(ε=10^-2)이 7단계 만에 t_s≈98.4s로 종료, 위반 1점(sim). MC 없음, 외부 baseline 없음(목적이 이론 검증이므로 illustrative).

**5. Contribution(delta).** 출판년(2025/2026) 기준 SOTA 대비: (i) 기존 LCvx 이론(Açıkmeşe & Ploen 2007; Açıkmeşe & Blackmore 2011; Malyuta & Açıkmeşe 2020)은 연속시간 등가성만 보장 — 본 논문은 실제 수치해가 사는 이산시간 영역에서 위반점 수에 대한 tight한 상한(n_x − 1)을 최초로 제시. (ii) 검증이 까다로운 strict transversality 가정(Condition 2)을 제거 → 실무 적용성 제고. (iii) Pontryagin 최대원리 대신 유한차원 선형대수·볼록해석으로 LCvx 메커니즘을 재해석, 새로운 시각 제공. (iv) 기존 LCvx가 실패하던 long-horizon을 bisection으로 해결. 전반적으로 새 방법이라기보다 LCvx 계열의 이론적 토대를 이산시간으로 확장·정련하는 foundational 기여.

**6. Limitation·Future work.** 저자: 비볼록성이 입력 크기 하한 단일 원천인 단순 문제에 한정(일반 비볼록 제약은 future work), Slater 조건의 모든 t_s 검증법 부재, 연속시간 제약 만족 보장은 별도(Elango et al.). 내가 보는 한계: 핵심 보장이 섭동 후 확률 1 명제라 무섭동 결정론적 보장이 아니며, n_x − 1 위반이 제어차원>1에서 비-tight하고 실제 추력 제약 위반은 남는다. 온보드 실시간성(타이밍/복잡도)은 bisection이 로그라는 언급 외 정량 분석 없음. 외부 baseline·MC 부재로 정량적 우위 입증은 제한적.

**평가:** 79/100 (strong) · 한줄평: LCvx(G-FOLD 기반)의 이산시간 이론 공백을 메우는 엄밀한 foundational 기여로 G&C 관련성이 높으나, 핵심 보장이 인위적 섭동 후의 확률 1 명제이고 검증이 illustrative에 머물러 flagship에는 못 미친다.

---

## (2025, IEEE LCSYS, Discrete-Time Lossless Convexification for Pointing Constraints: Luo·Spada·Açıkmeşe)

**1. 핵심 아이디어.** 연속시간 무손실 볼록화(LCvx)는 이산화(영차 홀드)되면 등가성이 깨져 완화된 볼록 해가 원 비볼록 제약을 위반할 수 있다. 본 레터는 저자들의 선행 이산시간 DT-LCvx 프레임워크[19]를 (i) 고전적 포인팅 제약(자세 방향 제한)과 (ii) 제어가 0이거나 환형(annular) 섹터 내로 제한되는 반연속(semi-continuous)·이중모드(dual-mode) 혼합정수 제약으로 확장한다. Slater·가제어성·횡단성(transversality) 가정 하에서, 볼록 완화해가 원 비볼록 제약을 위반하는 시간격자점 수가 포인팅의 경우 n_x−1, 새 R² 정식화의 경우 2n_x−2 이하임을 증명한다.

**2. 알고리즘.** 비볼록 제약 ξᵀuᵢ ≥ γ‖uᵢ‖ (포인팅) 또는 uᵢ∈{0}∪{환형 섹터} (P2)를 보조변수 σᵢ로 들어올려 볼록 문제 P3/P4로 완화: min Σlᵢ(σᵢ) s.t. xᵢ₊₁=Axᵢ+Buᵢ+zᵢ, ρ_min≤σᵢ≤ρ_max, ξᵀuᵢ≥γσᵢ, σᵢ≥‖uᵢ‖. 지시함수 I_V로 P5를 재정식화하고 라그랑지안 듀얼 ηᵢ*=Aᵀ(N−i)η_N*를 이용. LCvx 유효 충분조건: −Bᵀηᵢ* ∉ {λξ}(P3), {λξ₁}∪{λξ₂}(P4). 가제어성(Assumption 3·5)으로 마지막 n_x 격자점 중 최소 1점 유효 → 위반 수 상한 도출.

**3. 비판적 검토.**
- 핵심 듀얼 구조 정리(Th.1,2)와 변분/볼록해석 기계(Clarke[24], Bauschke[23])는 외부·선행연구[19]에서 가져오며, 본 레터의 고유 기여는 그 위에 포인팅/반연속 사례를 끼워 맞춘 확장 증명이다 — 엄밀하나 토대는 import.
- 이론 보장이 R²로 한정. R³(현실 3축 추력)에서는 보장이 깨지며 "휴리스틱으로 잘 작동한다"는 경험적 진술에 그친다(Remark 3). 실제 우주선/착륙선은 3D이므로 핵심 한계.
- 수치 검증이 단일 시나리오(쿼드로터 착륙)·MC 없음·외부 baseline 없음. P3(포인팅)는 본 레터에서 검증조차 없이 선행[7][8]에 위임.
- 위반 발생 시 실현가능집합 투영(projection)으로 메우되 O(t_f/N) 종단오차 발생 — 실무 보정책이나 격자 적응 기준은 미제시.

**4. 시뮬/실험.** 정규화된 2D 쿼드로터 착륙 1건(sim), N+1=50 고정, 중력·선형항력 포함 LTI, QOCO 콘 솔버. 결과: 2D P4에서 LCvx 위반이 t≈0.6 부근 1점에서만 발생(이론 상한 2n_x−2 대비 훨씬 적음), 충분조건 ‖−Bᵀηᵢ×ξ‖≠0이 위반점에서 0으로 떨어짐을 확인(Fig.2a). 3D 확장 시 위반 1점(동일 격자), 엔진 셧다운 유도 패널티(가중 10·lᵢ) 추가 시 t≈1.4,2.8 2점(Fig.2b). 정량 baseline·통계 없음, 예시는 정성적 일치 확인용.

**5. Contribution(delta).** 선행 CT-LCvx[7](포인팅 연속시간)과 DT-LCvx[19](일반 이산시간) 대비: 이산시간 포인팅 제약에 대한 LCvx 유효성·위반격자수 상한(n_x−1)을 최초 정식 증명; on-off/최소임펄스 추력기(dual-mode actuator)를 표현하는 반연속 환형 정식화(P2/P4)와 그 보장(2n_x−2)을 신규 도입. 2025년 SOTA 기준 — DT-LCvx 토대 자체는 신규가 아니나, 새 액추에이터 제약 클래스로의 확장과 그 위반 상한은 가장 가까운 선행 대비 분명한 진전.

**6. Limitation·Future work.** 저자: R³ 이론 보장 부재(polyhedral cone로 복원 가능성 언급), Assumption 4(횡단성)는 사후 검증 필요, 위반은 드물지만 N 조정·적응 필요. 추가로 내가 보는 한계: 실시간/온보드 타이밍·복잡도 분석 전무(LCvx 계보상 가능성은 높으나 본 레터 미입증), 단일 예시·MC·외부 baseline 부재로 실증 강도 약함, 피드백 자세제어와의 결합(추력이 비현실적 음수가 되는 문제)은 범위 밖으로 남김.

**평가:** 75/100 (strong) · 한줄평: 증명이 완결적이고 코드·파라미터가 공개된 견실한 LCSYS 이론 레터로, on-off 추력기 G&C 제약을 LCvx로 끌어올린 점은 가치 있으나 R²-한정 보장과 단일 예시·baseline 부재가 strong 하단에 머물게 함.

---

## (2025, ASR, Indirect optimization of multi-phase Mars entry, descent, and landing trajectory involving arbitrary discrete logic: Saranathan)

**1. 핵심 아이디어.** 다단계(EDL) 궤적 최적화를 간접법(necessary conditions) 틀에서 풀 때, 비행 구간 전환을 지배하는 불리언 논리가 AND만 있는 경우는 기존 RASHS로 처리되었으나 OR/NOT가 섞인 임의 논리는 다룰 수 없었다. 본 논문은 임의 불리언 논리를 논리곱 정규형(DNF)으로 변환한 뒤, AND는 시그모이드 곱, OR는 minterm 합의 signum을 tanh로 평활화하여 운동방정식·라그랑지안에 내장하는 GRASHS 기법을 제안한다. 이로써 다점경계치문제(MPBVP)가 이점경계치문제(TPBVP)로 축소되어 내부점 경계조건을 명시적으로 강제할 필요가 없어진다. 낙하산 전개가 속도-OR-고도 조건으로 작동하는 화성 EDL 예제로 시연한다.

**2. 알고리즘.** predicate g_{i,j,k}<0를 좌향 단위계단 [1-u(g)]로 표현하고 시그모이드 1/(1+e^{s·g})로 평활화(AND minterm = predicate들의 곱). OR은 minterm들의 합에 sgn(·)을 취하고 tanh(ζ·)로 평활화. 결과: Ẋ = Σ_k tanh(ζ·Σ_i Π_j [1/(1+e^{s·g_{i,j,k}})]) f_k, 라그랑지안도 동형. s,ζ→∞일 때 원래 piecewise 시스템에 수렴. 단일 minterm뿐이면 RASHS(식14)로 환원. NCO(λ̇=-(∂H/∂X)^T, ∂H/∂U=0)는 식(20) TPBVP로 collapse되고, 사소한 비행시간 문제에서 출발해 경계조건과 s,ζ를 5단계 homotopy(각 500~1000 iter, bvp4c/ode45)로 점진 연속화하여 해를 얻는다.

**3. 비판적 검토.**
- 평활화 해가 참 MPBVP 해로 수렴한다는 주장은 s,ζ→∞ 직관과 그림(Fig.1,2)으로 제시될 뿐 형식적 수렴 증명이 없다. 핵심 최적성 기계(Bryson-Ho NCO, RASHS)는 인용·차용된 것으로 본 논문 고유의 정리는 DNF-평활화 구성에 한정된다.
- baseline이 사실상 self-reference다: RASHS는 저자 본인 선행연구이고, 비교용 MPBVP는 GRASHS 해에서 얻은 사전지식으로 OR을 제거해 구성한 것이라 독립적 외부 baseline이 없다. 일치성(consistency) 검증은 정당하나 강한 비교라 보기 어렵다.
- OR 포화를 sgn(minterm 합)으로 구현하는 것은 합이 비음수임에 의존하며(논문도 명시), 평활화된 tanh가 0 입력에서 정확히 0을 주는 점이 시그모이드(0.5) 대비 오차를 줄이는 핵심인데, 이 선택의 일반성·수치 안정성은 더 분석될 여지가 있다.
- 불확실성/분산(dispersion) 처리가 전무하다. EDL은 본질적으로 강한 분산 환경인데 monte-carlo나 robustness 논의가 없어 설계도구로서의 실효성 검증이 부족하다.
- 간접법+homotopy 경로는 trial-and-error에 의존(논문 자인)하며, 더 발전된 adaptive/multistage continuation은 채택하지 않았다. 직접법(convexification, SCP-STC)이 운용 G&C의 주류로 이동한 점을 고려하면 niche가 좁다.

**4. 시뮬/실험.** MSL 유사 화성 EDL 3-DOF, 진입질량 3152 kg, 진입 h=120 km·v=5.9 km/s. 두 mission profile(낙하산 전개고도 h_P=3.5 km, 6.5 km)으로 시연. 첫 프로필은 속도-trigger(v_P=408 m/s에 먼저 도달), 둘째는 고도-trigger가 작동함을 GRASHS가 사전지식 없이 자동 식별. GRASHS/RASHS/MPBVP 세 해의 낙하산전개·PDI·접지 시각이 일치(예: profile1 접지 376.74~376.80 s, profile2 387.44~387.52 s). 상태·뱅크각·추력·연료소모·costate 이력이 실용상 일치(costate λ_θ,λ_φ,λ_ψ는 평활화 오차로 소폭 편차, s·ζ 증가 시 감소). 정량 baseline은 모두 내부(자기참조) 해. MC·외부 baseline 없음. (sim)만.

**5. Contribution(delta).** 선행 RASHS(AND-only)와 CSC를 임의 불리언 논리(AND/OR/NOT)로 일반화하여, 간접법에서 OR 논리 trigger를 사전지식 없이 자동 처리하는 첫 틀을 제시. 직접법 쪽 OR 처리(Malyuta-Acikmese homotopy, Szmuk STC)는 있었으나 간접 프레임의 임의 이산논리 처리는 출판년 기준 공백이었고 이를 메운다. 핵심 delta: trigger event(속도 vs 고도)에 대한 사전가정 없이 동일 운동방정식(식31,32) 재사용 — RASHS는 trigger 가정이 바뀔 때마다 식(35-39)처럼 재정식화가 필요. 다만 DNF 표현·시그모이드 평활화 자체는 표준 도구의 조합이라 개념적 도약보다는 견고한 확장에 가깝다.

**6. Limitation·Future work.** 저자: homotopy 파라미터 선택의 trial-and-error 의존, adaptive/multistage continuation 미적용. 추가로 내가 보는 한계: (i) 평활화 수렴의 형식적 증명 부재, (ii) 외부 baseline·MC·불확실성 검증 부재로 EDL 설계도구로서 robustness 미입증, (iii) 다수 minterm·고차원 논리에서의 수치 conditioning·계산비용 미보고, (iv) homotopy+bvp4c 기반으로 onboard/real-time 적용은 비대상(offline 설계 전용), (v) 자기참조 baseline으로 비교 강도가 약함. Future: 분산 환경 평가, 적응형 연속화 결합, 더 복잡한 논리/구간 수에서의 확장성 검증.

**평가:** 66/100 (good) · 한줄평: 간접법 다단계 궤적최적화에서 임의 이산논리(OR/NOT)를 자동 처리하는 공백을 깔끔하고 재현가능하게 메운 견실한 core-G&C 기여이나, 외부 baseline·불확실성·onboard 검증 부재와 좁은 niche가 영향력을 제한한다.

---

## (2026, arXiv, Parallel-in-Time Nonlinear Optimal Control via GPU-native Sequential Convex Programming: Zou et al.)

**1. 핵심 아이디어.** CPU 기반 직렬 선형대수(KKT 희소 분해, Riccati 재귀)에 묶여 GPU를 충분히 활용하지 못하는 비선형 제약 궤적 최적화 문제를, 시간축 분할(temporal splitting)과 consensus-기반 ADMM으로 분해하여 전 과정을 GPU에서 실행하는 "ucenter" 솔버를 제안한다. 각 collocation 노드를 독립 엔티티로 보고 이웃 시점과 dynamic consistency를 협상시키는 구조로, 전역 희소 분해 없이 per-node dense solve와 closed-form/projection 업데이트만으로 SCP 부분문제를 푼다. Jetson Orin 엣지 플랫폼에서 quadrotor 민첩 비행과 화성 동력하강을 검증하고, 12-core CPU 대비 4.1x 처리량과 51% 에너지 절감, 100Hz 초과 재계획률을 보고한다.

**2. 알고리즘.** 외부 루프: 비선형 OCP를 명목궤적 (x_bar^k, u_bar^k) 주변 1차 Taylor(동역학)/2차 전개(비용)로 convex QP로 선형화 (x_{i+1}=A_i x_i + B_i u_i + d_i, trust-region 박스 제약). 내부 루프: QP를 3-블록 변수 분할 ADMM으로 분해 — Physical (x,u), Dynamic auxiliary (z, x_i=z_i), Geometric mirror (x_hat,u_hat, 부등식/trust-region을 indicator로). Augmented Lagrangian L_rho에 rho_eq/rho_dyn/rho_geo 페널티. 4-스텝: Step1 Physical update는 H_i ξ_i=g_i 선형계 (H_i가 rho 정규화로 SPD라 unpivoted Cholesky, SCP iter 내 캐싱); Step2 Dynamic update는 closed-form 가중평균 z_i^{k+1}=(rho_eq(x^{k+1}+rho_eq^{-1}λ)+rho_dyn(d_{dyn}-rho_dyn^{-1}μ))/(rho_eq+rho_dyn); Step3 Geometric은 proximal projection(박스면 element-wise Clamp); Step4 dual ascent. 각 시점이 GPU thread block에 매핑, JAX vmap/jit로 배치 연산.

**3. 비판적 검토.**
- 이론적 기여가 약하다. 핵심 수렴 보장(부분 수렴된 inner ADMM과 outer SCP의 상호작용)을 저자 스스로 "future work"로 미루고, 3-블록 ADMM이 2-블록으로 환원된다는 주장도 구조적 분리성에 대한 서술적 논증일 뿐 정식 증명이 없다. 비볼록 SCP/inexact 수렴에 대한 정리가 전무하다.
- 핵심 빌딩블록(consensus ADMM[Boyd], operator splitting[O'Donoghue], successive convexification[Mao/Szmuk], proximal projection[Parikh-Boyd])이 모두 import이며, 새로움은 "GPU 친화적으로 시간축을 분할해 조립"한 시스템 측면에 집중된다.
- baseline이 약하다. 비교 대상이 자체 구현한 numba/multiprocessing 12-core iLQR뿐으로, 외부 SOTA GPU 솔버(예: 인용한 MPCGPU, cuRobo, Chari et al. PIPG-GPU)와의 직접 정량 비교가 없다. 4.1x speedup의 의미가 baseline 구현 품질에 좌우된다.
- 불확실성 처리가 scenario-based robust MPC(K=15, non-anticipativity 제약)로 제시되나 형식적 보장(튜브 불변성, chance 제약 만족)은 없고 2σ "prediction tube"는 경험적 산포에 가깝다.

**4. 시뮬/실험.** 전부 Jetson AGX Orin 64GB 상 JAX 구현 (sim/HIL 성격, 실제 비행 없음). (1) Quadrotor 13-state 6-DOF, N=40~50, 구형 장애물 3개, B=1000 randomized에서 93.9% success, 평균 dynamics defect 0.007. (2) Mars 14-state 6-DOF 동력하강(Szmuk 적응), N=30, glide-slope 10°/tilt 20° 제약, B=1000 (5% 상태섭동)에서 99.8% success, bang-bang thrust 관찰. (3) Scalability: B=1~5000, GPU 최대 101.1Hz(B=5000) vs CPU 포화 24.6Hz → 4.1x; 에너지 119.03J vs 243.58J(51% 절감); GPU 활용률 96%+. (4) Robust MPC: 크로스윈드 하 K=15 시나리오, step40 수렴, 안정화 후 ~200ms/step. baseline=자체 12-core iLQR(외부 baseline 부재).

**5. Contribution(delta).** 선행 GPU 궤적최적화(MPCGPU의 preconditioned CG, cuRobo의 sampling, Chari et al.의 PIPG-in-SCP)와 비교해, 전역 KKT 분해/Riccati 재귀를 완전히 제거하고 SCP의 inner convex QP를 3-블록 consensus ADMM으로 분할하여 per-node dense solve + closed-form + projection만으로 GPU에서 100% 실행하는 점이 delta다. 출판년(2026) 기준 novelty는 "GPU-native end-to-end SCP+ADMM의 시간축 병렬화 + 배치 시나리오 robust MPC로의 자연스러운 확장"으로, 알고리즘 원천성보다 아키텍처/시스템 통합과 엣지 실시간성 실증에 있다. Acikmese류 lossless/successive convexification이 *무엇을* 풀지를 정의했다면, 본 논문은 그것을 GPU에서 *어떻게* 빠르게 푸는가에 대한 공학적 기여다.

**6. Limitation·Future work.** 저자: (i) inner ADMM 부분수렴과 outer SCP 상호작용의 엄밀한 수렴 보장 부재, (ii) 물리 하드웨어(실비행) 실험 부재. 추가로 내가 보는 한계: (iii) 외부 SOTA GPU 솔버와의 직접 정량 비교 부재로 speedup 주장의 절대적 우위 입증이 약함, (iv) 코드가 review 후 공개 예정이라 현재 재현 불가, (v) rho 페널티/trust-region 튜닝 민감도(ablation에서 rho_f, ADMM iter 수에 따른 defect/비용 trade-off 관찰됨)에 대한 자동화/이론적 가이드 부재, (vi) ADMM의 well-known한 ill-conditioning 시 저정밀 수렴 문제를 SCP inexactness로 흡수하는데 안전필수 G&C에서의 제약 위반 상한 보장이 없음.

**평가:** 66/100 (good) · 한줄평: SCP+ADMM을 GPU-native로 시간축 병렬화한 견고한 시스템/엣지 실시간 기여이나 이론적 수렴 보장과 외부 baseline이 약하다; 화성 동력하강 검증으로 G&C 인접성은 높으나 본질은 control-theory/solver 도메인.

---

## (2021, AST, Multi-phase trajectory optimization of aerospace vehicle using sequential penalized convex relaxation: Zhao·Shang·Dong·Wang)

**1. 핵심 아이디어.** 다단(multi-phase) 항공우주 궤적 최적화에 내재된 세 종류의 비볼록성 — 비볼록 함수, phase-linkage 제약, 자유 종단/자유 연결시간 — 을 볼록최적화 틀에서 동시에 처리하는 SPCR(Sequential Penalized Convex Relaxation) 방법론을 제안한다. 자유 시점들을 정규화 시간축으로 사상하는 time-projection으로 자유 종단·자유 linkage 시간을 고정시간 문제로 변환하고, 남은 비볼록 동역학을 다항식화→QCQP→SDP 완화로 볼록화한 뒤 성능지수에 페널티항을 넣어 참해로 순차 수렴시킨다. 4단 발사체 상승과 다우주선 협력 랑데부 두 사례로 검증한다.

**2. 알고리즘.** (i) Time-projection: Π(t)=(t−t_i)/(t_{i+1}−t_i)+i 로 자유 시점 t_j 를 고정값 τ_j 로 변환, 설계변수를 [x,u,t_1..t_Nphase] 로 증강하고 동역학에 (t_{i+1}−t_i) 인자를 곱해 고정시간 OCP P1 생성. (ii) 다항식화: 기본함수(BEF)를 정리 1–3으로 다항식 표현→POP(P2). (iii) 차수 축소: 보조변수 α=(S_i1ŷ)(S_i2ŷ)로 차수를 2까지 내려 QCQP(P3) 도출. (iv) SDP 완화: Y=yyᵀ 등식을 반정부호 부등식 Y−yyᵀ⪰0 로 완화하고, 추가로 W(y,Y)=(Ay+b)(Ay+b)ᵀ⪰0 로 과완화 억제(P4). (v) 페널티: J에 η{trace(Y)−2ŷᵀy+ŷᵀŷ}+ζ‖ŷ−y‖ 를 더한 P5를 ŷ←y* 로 갱신하며 |trace(Y)−2ŷᵀy+ŷᵀŷ|≤ε 까지 반복(Algorithm 1). 수렴 시 P0의 KKT점으로 수렴(Theorem 4, Lemma 1–4).

**3. 비판적 검토.**
- 핵심 볼록화 기계(SDP 완화, IRM, penalized conic relaxation)는 [35],[38] 등에서 가져온 것이며, 본 논문의 고유 기여는 time-projection과 페널티 SDP의 결합·일반 다단 OCP로의 확장에 있다. 수렴 보장도 "적어도 국소적으로 P0의 KKT점으로 수렴"에 그쳐 전역 최적성·losslessness 보장은 아니다.
- 초기추정 의존성: SDP/SCP 계열의 고질적 약점인 초기값 민감도를 페널티로 완화한다고 주장하나, 파라미터(η_0,ζ_0,ε)와 초기 collocation 선택의 영향에 대한 정량적 민감도 분석이 없다("hard-to-detect environment" 인정).
- 불확실성·강건성 처리가 전무하다. 대기밀도·공력계수·추력 분산, 모델 오차에 대한 MC나 분산 분석이 없고 결정론적 공칭 궤적만 제시한다.
- 검증이 사례별 단일 인스턴스에 한정된다. GPOPS 대비 편차곡선은 제시하나 시나리오 다양화·통계 처리가 없어 일반성 주장이 경험적으로 약하다.
- 차수 축소·다항식 증강이 변수 차원을 크게 키워 SDP 행렬 크기를 증가시키는데(저자도 nonlinearity 증가 인정), 대규모 문제에서의 확장성·수치 컨디셔닝 논의가 부족하다.

**4. 시뮬/실험.** 두 사례 (sim) — (1) 4단 자유시간 발사체 상승(끝점 r_f,v_f 지정, 단별 질량·Isp·최대추력 Table 4): SPCR 8.96 s, 종단시각 964.99 s, GPOPS-SNOPT와 위치/속도/추력방향 곡선 거의 일치(상대편차 Δ 곡선 제시), 저자 주장 GPOPS의 약 1/10 계산시간. (2) 3우주선 협력 랑데부(미지 랑데부점, 2단계, t_f=3000 s 고정): SPCR 7.83 s, GPOPS 대비 편차 매우 작음. 모두 Intel i7-6700, MATLAB 2016b + MOSEK. baseline = GPOPS-SNOPT(외부 pseudospectral NLP, 강한 baseline). MC·통계 없음.

**5. Contribution(delta).** 출판년(2021) 기준 선행 대비 delta: (a) 자유 종단과 자유 linkage 시간을 동시에 다루는 일반 다단 OCP를, projection만의 차원·비선형성 증가 문제([39],[21])와 bisection의 free-linkage 부적용 문제([23])를 회피하며 볼록화. (b) IRM([35])·penalized conic([38])이 단일 범주 비볼록성에 머문 데 비해 세 비볼록 범주(함수·linkage·종단)를 동기적으로 볼록화하면서 페널티로 수렴을 회복하는 통합 파이프라인 제시. (c) 일반 다단 OCP에 대한 KKT 수렴의 형식적 증명(Lemma 1–4, Theorem 4). 새 방법이라기보다 기존 요소들의 비자명한 결합·일반화에 가까운 명확한 advance.

**6. Limitation·Future work.** 저자: 알고리즘 파라미터의 정량적 영향 분석 부재, 더 최적화된 구현 시 실시간 폐루프 가능성(현재는 ~10 s로 미달), 좋은 초기값이 있을 땐 SOCP 기반이 속도 면에서 유리할 수 있음을 인정. 내가 보는 한계: 전역 최적성·losslessness 보장 없음, 불확실성/강건성 검증 전무, 단일 인스턴스 검증으로 통계적 일반성 미흡, SDP 차원 확장성·수치 컨디셔닝 미검토, 공개 코드 없음. Future: 분산/강건 확장(chance-constrained), 대규모 문제 확장성, 온보드 실시간 구현.

**평가:** 70/100 (good) · 한줄평: time-projection과 페널티 SDP 완화를 결합해 일반 다단 항공우주 궤적 최적화의 세 비볼록성을 동시에 처리하고 GPOPS 대비 ~10x 가속을 보인 견실한 in-domain G&C 기여이나, 국소 수렴 보장·불확실성 검증 부재·단일 인스턴스 검증이 상위권 진입을 막는다.

---

## (2023, JSR, Direct-Indirect Hybrid Strategy for Optimal Powered Descent and Landing: Spada·Sagliano·Topputo)

**1. 핵심 아이디어.** 3-DOF 정점착륙(pinpoint landing) PDG 문제를 직접법과 간접법을 결합한 하이브리드 최적제어 알고리즘으로 푼다. 먼저 볼록 의사스펙트럼(hp Radau LGR) 직접 솔버로 기준해를 얻고, 정확한 공변량 사상정리(CMT)로 이산 라그랑주 승수를 연속시간 costate로 사상하여 간접 단일슈팅 primal-dual BVP의 초기추정으로 사용한다. 비교군은 순수 간접법(호모토피 연속화) 방식이며, 볼록-기반 추정 생성이 호모토피보다 우월함을 정량적으로 보인다.

**2. 알고리즘.** OCP를 Hamiltonian 증강(H = L + λᵀf)하고 PMP로 dualize하여 TPBVP(Eq.7,18) 구성. 추력은 primer vector 이론에 따라 bang-bang(Eq.15,16: i_T*=-λ_v/λ_v, S=T_max/m(1-λ_v)-λ_m·α). 간접 단계는 8차 Dormand-Prince 고정스텝 적분 + 스위칭 검출(Eq.27: Newton-Raphson zero-finding으로 t_sw 정확 위치 후 STM 점프 Ψ(t_sw) 적용, Eq.25-26)으로 bang-bang을 비강성 단일스텝 통합. 보정은 STM 기반 해석적 Jacobian으로 Newton-Raphson(Eq.20,23). 직접 단계는 무손실 볼록화 + 순차볼록화(SCvx, Eq.52-57)된 LGR PS NLP, 그 KKT 승수를 LGR-CMT(Eq.49-51)로 costate에 정확 사상. 순수 간접 비교군은 2차(quadratic, energy-optimal ε=0)에서 AO(ε=1)로 가는 호모토피(Eq.28-30).

**3. 비판적 검토.**
- 핵심 기계장치(LGR-CMT [35,36], 무손실 볼록화 [6,7], PMP, primer vector)는 모두 인용/수입이며, 본 논문의 고유 기여는 이들을 PDG 하이브리드로 통합·결합한 것이다. CMT 자체의 LGR 유도는 [36]에 위임.
- baseline이 저자 자신이 구현한 순수 간접 호모토피 방식뿐으로 외부/실비행 baseline 부재. 단일 벤치마크 시나리오(공력 무시, path/state 제약 미포함, 일정 중력).
- 회전 DOF를 포함한 고충실도 모델로의 확장은 PMP 복잡성 증가로 범위 밖이라 명시 — 즉 제시 방법의 실제 6-DOF 적용성은 미검증.
- 결론부에서 "offline guidance strategies"로 자리매김 — onboard 실시간이 아니라 사전 해 생성용임을 저자가 인정.

**4. 시뮬/실험.** 3-DOF 정점착륙(Isp=320s, u_T∈[0.3,1], 무차원 BC 명시) 단일 벤치마크. 계산시간(Table2, Dell XPS i7-9750H): 하이브리드 전체 ~2.3s(볼록추정 0.515s+CMT 8.3e-4s+간접 1.66s) vs 순수 간접 ~25s+(호모토피 24.7s) — 약 10배 이상 가속. costate 정확도: 볼록 추정이 호모토피 7번째 스텝 수준(Fig6). MC #1(tf 분산 0.15tf, 500 runs): 96%+가 6회 최적화 내 수렴, 17개 비수렴은 가상제어로 해결 가능한 problem-posing 이슈로 규명. MC #2(초기조건 분산, Table4): trust-region ξ에 따라 AO shooting 성공률 93%(ξ=1)→100%(ξ=0.25)(Table5). 외부 baseline 없음.

**5. Contribution(delta).** 출판년(2023) 기준 delta: (1) 볼록 직접 단계 + 정확 CMT + 간접 슈팅을 PDG 정점착륙에 통합한 하이브리드 알고리즘 신규 제시, (2) 최신 순수 간접 호모토피 방식 대비 볼록-CMT 조합의 계산시간 우월성 정량 입증. 개별 구성요소(CMT, 무손실 볼록화, SCvx, primer-vector bang-bang shooting)는 출판 전 이미 확립되어 있어, 기여는 "새 방향 개척"보다 "근접 선행 대비 명확한 통합적 진전"에 해당.

**6. Limitation·Future work.** 저자: 고충실도(회전 DOF) 모델은 PMP 복잡성으로 범위 외; path/state 제약(glideslope, 추력방향) 미포함; offline 용도. 추가로 본인 관점: 외부/실비행 baseline 부재로 절대 성능 비교 약함; 단일 시나리오로 일반화 근거 제한; onboard 실시간성 미입증; CMT 정확성이 무제약·bang-bang 가정에 강하게 의존하여 제약 활성 시 확장성 불확실.

**평가:** 73/100 (good) · 한줄평: PDG 정점착륙을 위한 직접-간접 하이브리드를 정밀한 CMT·스위칭검출로 견고하게 구성한 양질의 in-domain G&C 논문이나, 기여가 기존 구성요소의 통합이고 baseline이 자체 구현에 그쳐 flagship에는 못 미친다.

---

## (2024, JGCD, Rocket Landing Guidance Based on Linearization-Free Convexification: Yang·Liu·Song)

**1. 핵심 아이디어.** 공기력이 무시될 수 없는 지구 환경에서 재사용 로켓의 동력하강(powered descent) 착륙 문제를 풀되, 기존 SCO/SCvx 계열이 의존하는 어떠한 선형화에도 기대지 않고 비볼록 문제를 볼록화한다. 핵심은 일부 상태변수(질량 m, 그리고 비행경로각 γ·방위각 ψ에 대응하는 3D 착륙경로)를 사전에 결정·예비계획하여 동역학의 비선형성을 크게 줄인 뒤, 변수치환 V̂=V²로 상태방정식을 정확히 볼록화하고 목적함수를 이론적으로 동치인 선형식으로 대체하는 것이다. 각 반복의 해가 원래 비선형 동역학을 '완전히' 만족하므로 신뢰영역 제약 없이 보통 1회 반복으로 ms 단위에 수렴한다.

**2. 알고리즘.** 무차원 3DOF 동역학에서 τ=1−y를 독립변수로 잡아 식 (3)으로 변환. (i) 중력선회 연착륙 문제 P_soft를 1-파라미터 근(root)탐색으로 풀어 근사 질량 m̄ 확보(1/m̄ 비선형 제거). (ii) Bernstein 다항식 x(τ)=Σζ_x,i b_i,n(τ), z(τ)=Σζ_z,i b_i,n(τ)로 3D 경로를 매개변수화, 초기/말단 제약을 ζ의 닫힌식(24)–(28)으로 흡수하고 BFGS 무제약 최적화 P_path로 잔여계수(n=4)를 결정→γ,ψ도 확정. (iii) V̂:=V²로 상태식을 선형화 없이 볼록 제약 (44)–(46)으로 재서술; 비볼록 추력 콘 제약 √(T1²+T2²+T3²)≥T_min+δ_T는 T1≥T_min+δ_T로 근사(보수성 D_V<1.5% 정량화); 목적함수 J_nc=−∫T1/(√V̂ cosψ sinγ)dτ를 J_c=−∫V̂ dτ와 동치임을 Proposition 1(부록 B, 측도론적 모순법)로 증명→SOCP P2. (iv) 질량근사 오차를 제어 (T1c,T2c,T3c)=식(63)–(65)로 보상. 유도식: T_cmd=√(T1²+T2²+T3²), α_cmd=sin⁻¹(T2/T), β_cmd=tan⁻¹(T3/T1), 종방향 가속도 추종 PID로 ΔT 보정 T_cmd←T+ΔT.

**3. 비판적 검토.**
- 질량 프로파일을 중력선회 연착륙 해로 '근사'하는 것이 방법론의 핵심 가정인데, 이는 증명이 아니라 "최적 추력이 일반적으로 coast-burn 구조와 유사"라는 경험적 관찰에 의존한다. 오차보상으로 메우지만 최적 추력이 이 구조에서 크게 벗어나는 임무에서는 보수성·반복 증가 위험이 남는다.
- 경로를 사전계획하므로 횡방향 기동이 경로 곡률(제한 횡가속)에 묶인다. Fig. 11의 controllability 영역(원형)은 이 사전계획이 발산능력을 제한함을 그대로 보여준다—임의 초기조건에서 보편적 최적성이 보장되지 않는다.
- SCO 대비 연료 손실 2.2–2.3%는 작지만, 비교 대상이 단일 baseline [23](Szmuk·Acikmese) 한 종에 국한된다. G-FOLD류 lossless convexification(공기력 무시)과 직접 비교가 없어 '선형화 회피'가 주는 실질 이득(수렴 신뢰성 vs 최적성)의 경계가 다소 일면적이다.
- 자세 동역학을 무시한 점질량 3DOF이며 추력방향=종축 정렬을 가정. 실 로켓의 자세제어 지연·짐벌 한계가 α_cmd/β_cmd 추종에 미치는 영향은 다루지 않는다.
- MC는 충실하나(1000회) 형식적 확률 보증(chance constraint 등)은 없고 경험적 통계에 그친다.

**4. 시뮬/실험.** 26 t급 로켓(T_max=310 kN, η_max=20°), ECOS SOCP(51노드, 204변수). 4개 임무 정밀착륙(sim): 수직 접근, TAOA→0, α/β→0. 1000+ 무작위 (x0,z0) controllability 맵(원형 영역). SCO[23] 대비: 연료 2.2–2.3% 증가(예 4220.7 vs 4128.4 kg), 계산시간 3.1 ms vs 281.3 ms(약 90배), SOCP 규모 204변수 vs 818변수, 1회 반복 vs 7회. 3DOF 폐루프(10 Hz, Δt=1 s, PID Kp=−1.5e4,KI=−1e5) 1000회 MC(sim): 초기상태 3σ 250 m/25 m/s, 공기계수 12%, 추력 3%, 질량 300 kg, 바람 10 m/s 외란. 결과 √(xf²+zf²) 평균 3.71e−4 m, Vf 평균 0.502 m/s(max 0.631), γf≈−90°, ψf≈0, 연료 평균 4.27e3 kg—전 시행 안전착륙. 실시간 C(Intel i7-7700, 16 GB).

**5. Contribution(delta).** 2024 SOTA 기준 핵심 delta는 '선형화를 전혀 쓰지 않는' 볼록화 패러다임이다. 지배적이던 SCvx/SCO는 비선형 동역학을 선형화→신뢰영역으로 수렴을 강제하고 각 반복해는 선형화된 동역학만 만족한다. 본 논문은 (a) 상태 사전결정(질량·경로)으로 비선형성을 축소하고 (b) 변수치환으로 잔여 동역학을 '정확히' 볼록화하여 각 반복해가 원래 비선형 동역학을 완전히 만족하게 만든다. 따라서 신뢰영역·초기 궤적 추정이 불필요하고 보통 1회 반복으로 수렴—임베디드 잠재력이 본질적으로 높다. 목적함수 동치 증명(Prop. 1)과 추력콘 보수성 정량화(D_V<1.5%)가 이 주장을 뒷받침한다.

**6. Limitation·Future work.** 저자 한계: 점질량 3DOF·자세 무시, controllability 영역 제한, P2 infeasible 시 ω 감소로 비행가능경로 탐색 필요. 추가로 보는 한계: 질량근사의 경험적 정당화가 이론적 보증 부재의 약한 고리이며, baseline이 SCO 단일 비교에 그쳐 lossless convexification류와의 최적성 trade-off가 미검증. 자세동역학·짐벌제약을 포함한 6DOF 확장, coast-burn에서 크게 벗어나는 임무에 대한 강건성, 형식적 확률 보증(chance-constrained) 도입이 후속 과제.

**평가:** 82/100 (strong) · 한줄평: 선형화 없이 매 반복해가 실 비선형 동역학을 만족하게 만드는 신선한 볼록화로, 외부 SCO baseline 대비 90배 빠른 ms급 온보드 잠재력을 정량 입증한 견고한 core-G&C 기여(다만 경로 사전계획이 발산능력·일반성을 제한).
