# Thread F 비판 리뷰 — 임베디드 conic solver 6편

> *onboard 실시간 해법*: SCP/SOCP/QP를 위성·MCU·SoC급에서 푸는 도구 6편(FSQP·Conic-TinyMPC·DiscreteREAP·Clarabel·QOCO·KAIST-IPM).
> 공정성: solver는 *시스템 기여*가 본령이므로 정당히 인정하되, 알고리즘이 차용/복제면 C2 절감·핵심 수렴증명이 차용이면 C1 절감. C4(위험모델 충실도)는 설계상 결정론→대개 N/A·재정규화; robust/early-termination을 *주장*하면 그 한도로 채점. C7(온보드)이 본 스레드 핵심 강점 축 — codegen·MCU·실측 solve-time·메모리 풋프린트를 적극 가점(실측 임베디드=5, 데스크톱 전용=2).

---

## (2024, RA-L, Inherently Robust Suboptimal MPC for Autonomous Racing With Anytime Feasible SQP: Numerow et al.)

**1. 핵심 아이디어.** 미니어처 자율 레이싱을 위한 inexact MPC로, 어떤 반복 횟수에서 중단하더라도 항상 feasible한 중간해를 산출하는 anytime feasible SQP를 채택한다. 여기에 한 바퀴 주기 궤적을 오프라인으로 미리 계산해 만든 time-varying terminal manifold를 결합하여, 외란·솔버 실패·지연 하에서도 recursive feasibility를 보장한다. 즉 (local) 최적성을 포기하는 대신 제약 만족(안전성)을 우선하고, 계산 부담을 크게 낮춘다.

**2. 알고리즘.** 동적 자전거 모델(Pacejka 타이어, 상태 x=(p_x,p_y,γ,v_f,v_l,ω,τ,δ,θ), 입력은 τ·δ·θ의 시간미분)을 RK4로 이산화. MPCC 기반 stage cost l(x,u)=q_C e_C^2+q_L e_L^2+q_dτ(dτ)^2+q_dδ(dδ)^2+q_dθ(dθ-v̄)^2, 트랙 경계는 soft 제약 π(x)=(p_x-p̄_x(θ))^2+(p_y-p̄_y(θ))^2-(D/2)^2≤ξ. 핵심은 feasible SQP(Alg.1): 내부 QP (14)에서 gradient 근사 a(ỹ,ŷ)=∇f(ỹ)+P(ŷ-ỹ)를 쓰되 비선형 제약 g,h는 매 inner 반복마다 재선형화하여 feasibility를 유지; outer 반복에서만 미분을 재계산. P를 정확 Hessian으로 두면 M≠P여도 이차 수렴 회복. terminal 집합 X_f(t)={x:π_{N,t}(x)≤0}는 (16)으로 푼 주기 궤적 x_f(t)의 단일점 manifold이며, k+1 시점 feasible 궤적은 (18)의 단순 shift로 얻음. Prop. V.6: Lipschitz 가정(19)(20)+deadbeat controller 존재(Asm V.2)+Asm V.4 하에서, ||w_i||≤δ면 M-step open-loop recursive feasibility 성립.

**3. 비판적 검토.**
- 핵심 구성요소가 대부분 기존 것이다. feasible SQP는 [7](Zanelli 박사학위), terminal-manifold 기반 recursive feasibility는 [11,13], suboptimal MPC 강건성 이론은 [11]에서 가져왔다. 본 논문의 delta는 이들의 종합과 racing 적용, 그리고 솔버 실패에 대한 M-step 확장이다.
- 보장이 가정에 크게 의존한다. Asm V.2(k-step deadbeat controller 존재)와 Asm V.4는 강한 가정이며, Remark V.3/V.5에서 충분조건을 제시하나 실제 트랙·동역학에서 ρ, σ, M̄ 같은 상수가 어떻게 정해지는지 정량적 검증이 없다.
- 통계적 엄밀성이 약하다. sim은 단일 시드 재사용, 하드웨어는 3회(HW1-3)뿐이고 Monte Carlo·신뢰구간 없이 평균±range만 제시한다. TAES/TASE 기준으로는 표본·통계 처리가 부족하다.
- terminal manifold가 단일점 주기궤적으로 고정되어, lap progress를 실시간 최적화하지 못한다(저자도 5장에서 인정). 연속 manifold 확장은 future work로 남김.

**4. 시뮬/실험.** 시뮬: Python, 노이즈 수준 4종(SM1/2/4/8, 최대 변위 1/2/4/8 cm), 각 10바퀴(1600 스텝), 동일 시드. 하드웨어: Kyosho Mini-Z 1:28 RC카, 46 cm 폭 트랙, VICON+EKF 상태추정, CRS(C++), 30 Hz, Intel i7-9750H. baseline은 외부 솔버 RTI(표준 SQP 1회)와 IPOPT. 정량 결과(real): FSQP 평균 계산 8.00 ms로 33 ms 예산 내 안정 동작, lap time 5.50 s(165스텝). FSQP/RTI runtime≈1.54-1.58×, IPOPT/FSQP≈3.3-3.8×(즉 FSQP가 IPOPT보다 ~3.5배 빠름). FSQP 수렴률 96.0-98.2%, FSQP/RTI cost≈0.91-1.0(약간 낮음). RTI는 constraint violation(28) 발생(Fig.7), FSQP는 violation 제거. 비수렴 소수 케이스는 이전 스텝 shift 해 사용.

**5. Contribution(delta).** 출판년(2024) 기준, RTI류 early-stopping이 feasibility 보장 없이 제약 위반을 일으키는 문제를, anytime feasible SQP를 racing MPC에 적용하고 미리 계산한 주기 lap 기반 time-varying terminal set과 결합하여 해소한 점이 delta다. 특히 솔버가 최대 M회 연속 실패/지연해도 recursive feasibility가 유지됨을 (18) shift 후보로 증명한 M-step open-loop recursive feasibility는 [11]의 정적 결과를 실용적 inexact-solver 시나리오로 확장한 것으로, 기존 SOTA 대비 명확한 진전이다. 다만 새 방법론 발명보다는 기존 요소의 견고한 종합·증명·실증에 가깝다.

**6. Limitation·Future work.** 저자: terminal manifold가 이산 단일점이라 실시간 lap-progress 최적화 불가(연속 manifold 확장 필요); 모델 불일치(배터리에 따른 토크·조향 효과 변동)로 동역학이 실험 간 비일관; hard 제약이 infeasible해지면 violation 가능(soft로 완화). 내가 보는 한계: Asm V.2/V.4 상수의 실측·검증 부재, MC·통계 미흡(시드 1개, HW n=3), terminal set의 보수성이 cost에 주는 영향 미정량, aerospace G&C(예: 동력하강·재진입)로의 직접 이전성은 미입증.

**평가:** 78/100 (strong) · 한줄평: 안전 보장형 anytime inexact MPC를 실하드웨어에서 30 Hz로 깔끔히 실증한 견고한 control-theory 연구이나, 검증 대상이 RC 레이싱이라 G&C 직접 관련성은 간접적이고 핵심 요소들이 기존 문헌의 종합이다.

---

## (2024, arXiv, Code Generation and Conic Constraints for Model-Predictive Control on Microcontrollers with Conic-TinyMPC: Mahajan et al.)

**1. 핵심 아이디어.** 자원이 제한된 마이크로컨트롤러(MCU)에서 동작하는 임베디드 MPC 솔버인 TinyMPC를 2차 원뿔(second-order cone, SOC) 제약까지 확장하여 SOCP를 풀 수 있게 한 Conic-TinyMPC를 제안한다. ADMM의 slack 업데이트가 본질적으로 실현가능 집합으로의 투영(projection)이라는 점을 활용하여, 박스 제약 투영을 SOC의 폐형(closed-form) 투영 연산자로 교체하기만 하면 구조-활용/캐싱 기반 고속 솔버가 그대로 원뿔 제약을 지원하게 된다. 더불어 Python/MATLAB/Julia 인터페이스에서 의존성 없는 C++ 코드를 자동 생성하는 코드젠 도구를 제공한다.

**2. 알고리즘.** ADMM 3-스텝 반복: (i) primal x-update는 무한지평 LQR 게인 K_inf, 비용-투-고 헤시안 P_inf를 미리 캐싱(C1..C4)하여 O(n^2) 행렬-벡터 곱과 백워드 패스(식 17)+포워드 롤아웃(식 2)으로 단순화, (ii) slack z/w-update는 실현가능 집합으로의 투영 proj(식 14), (iii) dual y/g-update는 스케일드 그래디언트 상승(식 15). 핵심 추가는 SOC 투영 연산자(식 21): ||v||2 <= -a 이면 0, ||v||2 <= a 이면 z 그대로, 그 외 0.5(1+a/||v||2)[v; ||v||2]. 페널티 ρ는 OSQP식 휴리스틱으로 사전 캐싱된 행렬 집합 사이를 온라인 전환.

**3. 비판적 검토.**
- 핵심 알고리즘 골격(캐싱, 무한지평 근사, ADMM 분할)은 선행 TinyMPC[5]에서 그대로 가져온 것이며, 본 논문의 알고리즘적 델타는 박스 투영을 SOC 투영으로 교체하고 코드젠을 붙인 것에 국한된다. 원뿔 투영 자체는 표준 결과(Boyd & Vandenberghe).
- 수렴/실현가능성/안정성 보장에 대한 자체 증명이 없다. 무한지평 LQR 게인 근사로 인한 시변 Riccati와의 오차, 그로 인한 최적성/제약 만족 영향에 대한 정량적·이론적 분석이 부재하며 [18],[29]로 인용 처리.
- 고정 선형화에 의존(저자도 인정)하여 비선형 동역학에서의 보장이 없다. Crazyflie를 호버 근방으로 선형화한 점-질량+쿼터니언 모델은 공격적 기동에서 모델 불확실성을 그대로 노출한다.
- 불확실성/강건성에 대한 형식적 처리가 없다(MC 캠페인, 노이즈/외란 모델 부재). Early-termination 실험이 시간예산 강건성을 보여주지만 이는 솔버 속도 이점일 뿐 정량적 강건성 보장은 아니다.

**4. 시뮬/실험.** MCU 벤치마크: STM32F405(168MHz/128kB)에서 OSQP 대비 최대 20.4배(상태차원 변화)·7.2배(지평 변화) 속도 향상, 메모리 절감으로 N=100/n=32까지 확장(OSQP는 N=32, n=28에서 한계)(sim/embedded). Teensy 4.1(600MHz)에서 SOCP rocket soft-landing을 ECOS/SCS와 비교, 평균 13.8배(SCS)·142.7배(ECOS) 가속, N=256까지 확장(ECOS/SCS는 N=64에서 메모리 초과). Early-termination(표 II): 2/10/20/1000ms 예산에서 ECOS는 warm-start 불가로 대부분 실패, Conic-TinyMPC는 landing error 0.87로 SCS 대비 1.6~2.4배 우수, 제약 위반도 최소. 하드웨어: 27g Crazyflie 2.1에서 50Hz·최대 20 ADMM iter로 (1) predictive safety filtering(박스 ±0.6m), (2) thrust-cone(±0.2/0.25 rad) attitude regulation, (3) 45° glideslope 원뿔 제약 나선 착륙을 onboard 실시간 시연(real). Baseline: Brescianini/Mellinger 반응형 제어기는 cone 제약 위반/실패.

**5. Contribution(delta).** 출판년(2024) 기준, 임베디드 MCU 타깃·warm-start·구조활용 솔버 중 SOCP를 네이티브로 지원하면서 코드젠까지 제공하는 사례는 사실상 최초(표 I에서 TinyMPC는 QP만, OSQP/CVXGEN도 QP만, ECOS/SCS/Clarabel은 임베디드/MPC-맞춤 아님). 따라서 델타는 "SOC 투영 한 줄 교체"라는 단순한 알고리즘 변경이지만, 그 결과로 마찰/자세/추력 원뿔 같은 G&C에서 빈번한 제약을 28g급 MCU에서 실시간으로 처리 가능하게 만든 실용적·시스템적 기여가 분명하다. 추가로 다언어 코드젠 패키지는 재현성/보급 측면의 실질적 가치.

**6. Limitation·Future work.** (저자) 고정 선형화 의존으로 비선형 시스템 일반화 한계 → reachability 기반 비선형-선형 갭 모델링[51] 예고, MicroPython 인터페이스, 세밀한 ρ 적응[41] 통합 예정. (추가) 형식적 수렴/안정성/실현가능성 보장 부재, 무한지평 근사 오차의 정량 분석 부재, 불확실성하 MC 검증 부재, SDP 등 더 일반적 원뿔은 원리적 언급에 그침. 항공우주 자체 하드웨어가 아닌 쿼드로터 대리 검증(단, rocket soft-landing/glideslope를 명시적으로 다룸).

**평가:** 77/100 (strong) · 한줄평: SOC 투영 한 줄 교체라는 단순한 알고리즘 델타지만 28g MCU에서 원뿔 제약 MPC를 실시간 구현해낸 강한 시스템·배포 기여 — 항공우주 자체 검증은 아니나 rocket soft-landing/glideslope/추력-원뿔을 직접 다뤄 G&C 관련성이 높은 near-domain 임베디드 솔버.

---

## (2025, SCL, Practical Considerations for Implementing Robust-to-Early Termination Model Predictive Control: Amiri·Hosseinzadeh)

**1. 핵심 아이디어.** REAP(Robust-to-Early Termination MPC)는 MPC 최적해를 primal-dual gradient flow 가상 동역학계에 임베딩하여 프로세스와 병렬로 구동하고, 가용 연산시간이 끝나면 언제든 종료해도 실현가능·준최적해를 보장하는 기법이다. 그러나 기존 REAP는 연속시간 가정 하에서 이론이 전개되어, 실제 구현에 필요한 이산시간(Euler) 적분 시 이산화 오차로 제약집합 불변성이 깨질 수 있다. 본 논문은 매 연산 스텝마다 KKT 파라미터 σ(τ|t)(진화율)를 적응적으로 선택해 이산시간에서도 anytime 실현가능성과 수렴성을 유지하는 조건을 제시·증명한다.

**2. 알고리즘.** 가상계 Euler 업데이트: û(τ|t)=û(τ−1|t)−σ(τ|t)·dτ·∇_û B(·), λ̂(τ|t)=λ̂(τ−1|t)+σ(τ|t)·dτ·(∇_λ B(·)+Φ(τ−1|t)). 여기서 B는 modified barrier function, Φ는 projection operator. 핵심은 Theorem 1의 σ 상한: 0 ≤ σ(τ|t) ≤ min{ min_i σ_i^û(τ|t), min_i σ_i^λ(τ|t) }, σ_i^û=δ_i^û/(dτ·‖η_i‖·max{‖∇_û B‖,ψ}) (식 16). δ_i^û는 현재 점과 i번째 제약 사이 Euclidean 거리. 이 σ 선택이 제약집합 D_û, D_λ의 불변성을 매 스텝 보장한다(Theorem 1). Lyapunov 함수 W=‖[û−u*; λ̂−λ*]‖²로 boundedness/수렴(Theorem 2), Theorem 3에서 anytime 실현가능성을 증명.

**3. 비판적 검토.**
- 강건성(robustness)이 '조기 종료(연산 부족)'에 대한 것이며 프로세스/측정 잡음·모델 불확실성에 대한 정량적 특성화는 없다. 드론 실험에 잡음은 존재하나 불확실성 하의 형식적 보장은 다루지 않는다.
- baseline이 모두 내부 변형(고정 σ CASE I~IV, 연속시간 근사)으로, 경쟁 기법(anytime MPC, fixed-iteration MPC, dynamically embedded MPC 등) 대비 외부 비교가 없다. ablation 성격의 검증이다.
- Remark 3·5에서 저자 스스로 worst-case(모든 제약에 대한 최소값) 채택으로 인한 보수성을 인정한다. tightening factor β, warm-start 등 완화책은 future work로 미룸.
- 핵심 기계(strong convexity 부등식 식 22, Alexandrov 정리)는 [25,39]에서 import하여 연속시간 REAP 위에 구축된 확장이다.
- 검증 모델이 6-state 선형 위치 동역학으로, 비선형/고차원 시스템에서의 σ 계산 부담·보수성은 미평가.

**4. 시뮬/실험.** Parrot Bebop 2 드론(샘플링 0.2s, 6-state 선형 위치 모델), OptiTrack 10대 Prime^x13 카메라, i9-13900K. 시뮬: 동일 초기조건(α∈[−0.5,0.5] 무작위) 1000회 실험, 5개 σ 케이스 제약 위반율 비교 — CASE I(σ≈2.22e-16, 연속시간 근사)=0%, CASE II(0.5)=100%, CASE III(0.05)=85.2%, CASE IV(0.005)=48.9%, CASE V(식 15 상한)=0% (Table 1)(sim). 성능 저하: 가용시간이 샘플링주기 절반이어도 <0.3%(Fig 8)(sim). 실제 실험(real): 드론 위치제어, 평균 37 연산스텝/순간으로 제약(pitch/roll≤0.05rad, vertical≤0.6) 만족하며 목표 추종(Fig 9), σ 범위 [0,63.21], 평균 0.2392. 외부 기법 baseline 없음.

**5. Contribution(delta).** 출판년(2025) 기준 delta: 연속시간으로만 전개되던 REAP[25]를 이산시간 구현으로 확장하고, 이산화 오차로 인한 제약 위반을 막는 적응형 KKT 파라미터 선택 규칙(식 15-16)과 그에 따른 불변성·수렴·anytime 실현가능성을 해석적으로 증명한 것이 새롭다. 고정 σ가 위반(최대 100%)을 유발함을 보이고 적응 σ가 0% 위반을 달성함을 정량 입증. 오픈소스 MATLAB 패키지(DiscreteREAP) 제공. 다만 새로운 G&C 방법/방향 개창이 아니라 기존 방법의 실용적 구현 가능화(practical considerations)에 해당한다.

**6. Limitation·Future work.** 저자: worst-case 채택으로 인한 보수성, β·warm-start를 통한 완화는 future work; Remark 4의 σ stall 가능성. 내가 보는 한계: (i) 외부 경쟁 기법 대비 비교 부재로 상대적 우위 미입증; (ii) 잡음/모델 불확실성 하 강건성 형식 보장 부재; (iii) 선형·저차원 검증으로 비선형 G&C(동력하강·재진입 등)로의 직접 적용성 미평가 — 매 스텝 min over 제약의 σ 계산 비용이 고차원에서 부담; (iv) anytime MPC 계열 대비 실시간 이점의 정량 비교 부족.

**평가:** 77/100 (strong) · 한줄평: 이론적으로 엄밀하고 드론 실시간 검증·오픈소스까지 갖춘 견고한 anytime-MPC 이산구현 논문이나, 기존 REAP의 실용적 확장이며 외부 baseline이 없고 항공우주 G&C로의 관련성은 (제약 onboard-MPC 측면에서) 간접적이다.

---

## (2026, Math. Prog. Comput., Clarabel: An Interior-Point Solver for Conic Programs with Quadratic Objectives: Goulart·Chen)

**1. 핵심 아이디어.** 2차 목적함수를 가진 일반 원뿔(conic) 최적화 문제 P (min ½xᵀPx+qᵀx s.t. Ax+s=b, s∈K)를 풀기 위한 범용 primal-dual 내부점(interior-point) 솔버를 제시한다. 핵심은 단조상보문제(MCP)용으로 개발된 동차 임베딩(homogeneous embedding)을 2차 목적함수 문제에 특화시킨 H 정식화를 직접 푸는 것으로, 기존 HSDE처럼 2차항을 epigraph+SOC 제약으로 치환하지 않고 ½xᵀPx 항을 임베딩 안에 유지한다. 대칭/비대칭 원뿔(비음, SOC, PSD, 지수, power)을 모두 지원하며 Rust/Julia 오픈소스로 배포되고 CVXPY의 기본 솔버다.

**2. 알고리즘.** 동차 임베딩 H에서 비선형계 G(v)=0 (v=(x,z,s,τ,κ))의 근을 self-concordant barrier 기반 중심경로 G(v)=μG(v⁰), s=-μ∇f*(z), z=-μ∇f(s) 를 따라 Newton형 방향으로 추적한다. 각 반복에서 축소된 quasidefinite KKT계 [[P+εI, Aᵀ],[A, -(H+εI)]]=LDLᵀ 를 정적/동적 정규화와 함께 직접 인수분해하고, 공통 LHS에 대해 predictor(affine)+corrector(centering) 두 RHS를 푸는 Mehrotra형 보정을 적용한다. 대칭 원뿔은 NT 스케일링, 비대칭 원뿔은 BFGS 기반 rank-3 업데이트 스케일링을 사용. epigraph 정식화 대비 P^½ fill-in을 피해 인수분해 효율을 높인다는 점이 핵심 이득.

**3. 비판적 검토.**
- 이론 기여(Lemma 2.1, 2.2)는 H의 점근적 가해성·상보성과 비가해성 인증서 회수를 다루나, 핵심 동차 임베딩 골격은 Andersen-Ye [50] 및 [53,54]에서 가져온 것으로 "[54],(4.1)과 동일"이라고 명시한다. 즉 새 수렴/복잡도 정리는 없고 기존 기계의 2차 목적함수로의 특화·구현이 본질이다. 알고리즘 측면의 신정리(반복복잡도 등)는 제시되지 않는다.
- 성능 우위의 근거가 다소 정성적이다. epigraph 정식화의 fill-in 증가를 식 (13)으로 지적하나, 실제 fill-in/인수분해 비용을 정량 분해한 ablation은 없고 end-to-end 벤치마크 시간으로만 입증한다.
- 비교 대상이 기본 설정·single-thread로 통일되었으나, Mosek/Gurobi 같은 상용 솔버의 멀티스레드·전처리를 끈 조건이 공정성에 어떤 영향을 주는지(특히 ECOS의 P^½ Cholesky 실패가 본질적 약점인지 정식화 강제 때문인지)에 대한 통제가 제한적이다.
- 정확도 임계값(ε_f=10⁻⁸, 약 10⁻⁵)과 실패 처리(실패 시 최대 시간 할당)는 shifted GM을 실패율에 민감하게 만든다 — 우위가 속도인지 강건성(낮은 실패율)인지 표마다 섞여 있다.

**4. 시뮬/실험.** 6개 표준 벤치마크군에서 ECOS, HiGHS, Mosek, Gurobi, Hypatia 대비 평가(all sim, 단일 Xeon 8268 스레드). QP: Maros-Meszaros 138문제(Clarabel shifted-GM 1.0~1.02 최고, 실패율 8.8% vs Mosek 59.9%/ECOS 48.2%), SuiteSparse 최소제곱 46문제(Clarabel/Gurobi만 full-acc 전부 해결), 최적제어 72문제(Clarabel 유일 100% full-acc, Mosek 511.01배). LP: NETLIB 117 feasible/29 infeasible(상용과 대등), CBLIB 지수원뿔 39문제(Mosek와 유사~약간 빠름). SOCP-OPF(PGLib): Clarabel 성공률 99%+ vs ECOS/Mosek 50%+ 실패. SDP: SDPLIB 64문제, chordal decomposition으로 일부 ARCH 문제 약 100배 가속, Mosek 9.95배. 다운로드 수백만 회, CVXPY 기본 솔버.

**5. Contribution(delta).** 출판년(2024 arXiv) 기준 delta: (i) 단조상보문제용 동차 임베딩([50,54])을 널리 쓰이는 내부점 원뿔 솔버에서는 처음으로 2차 목적함수에 직접 적용 — 기존 솔버(ECOS, SCS, Mosek HSDE)는 2차항을 epigraph+SOC로 치환하던 것을 회피. (ii) 대칭+비대칭 원뿔 통합 지원, chordal decomposition(COSMO 기법 차용)까지 포함한 프로덕션급 오픈소스 구현. (iii) QP/대규모 SOCP/희소 SDP에서 SOTA 상용·오픈소스 대비 실측 우위. 알고리즘 골격 자체의 신규성은 중간 정도이고, 본질적 기여는 특화+구현+실증의 결합이다.

**6. Limitation·Future work.** 저자: clique-merge 점수법 개선 여지, chordal decomposition 구현 추가 최적화 여지, Julia/Rust 간 고난도 문제에서 미세한 거동 차이. 내가 보는 한계: (a) 새 수렴/복잡도 이론 부재 — 기여가 공학·실증에 치우침; (b) 비대칭 원뿔 BFGS 스케일링·중심경로 근접성 휴리스틱의 이론적 보장 미약; (c) G&C 관점에서 직접 적용 사례(실시간 강하유도, 온보드 MPC의 timing/임베디드 검증)는 없고 최적제어 벤치마크도 오프라인 풀이 시간 비교에 그침 — onboard 실시간성은 plausible하나 미입증.

**평가:** 75/100 (strong) · 한줄평: 강건성·속도가 검증된 프로덕션급 원뿔 솔버로 powered-descent SOCP·MPC·covariance steering 등 G&C 수치 핵심을 직접 떠받치는 near-domain 도구이나, 신규 이론보다 특화·구현·실증이 기여의 본질이고 항공우주 온보드 검증은 부재(optim-tool, G&C 관련성 높음·직접 검증 없음).

---

## (2026, Math. Prog. Comput., QOCO: A Quadratic Objective Conic Optimizer with Custom Solver Generation: Chari·Açıkmeşe)

**1. 핵심 아이디어.** 2차 목적함수를 가진 2차 원뿔계획(QO-SOCP, min ½xᵀPx+cᵀx s.t. Gx⪯_K h, Ax=b)을 슬랙변수로 선형목적 SOCP로 변환하지 않고 직접 푸는 오픈소스 C 기반 primal-dual 내부점 솔버 QOCO를 제안한다. 추가로, 문제의 희소성 패턴이 고정된 응용(MPC, SCP)을 겨냥해 해당 패턴에 특화된 커스텀 선형대수 루틴을 가진 C 솔버를 자동 생성하는 코드 생성기 QOCOGEN을 함께 제공한다. 후자는 동적 메모리 할당이 없어 임베디드 배포에 적합하다.

**2. 알고리즘.** Mehrotra predictor-corrector를 갖춘 primal-dual 내부점법(Algorithm 1). 비대칭 콘(NN-orthant×SOC) 처리를 위해 Jordan product와 Nesterov-Todd 스케일링(W: ∇²φ_K(w)s=z, ∇²φ_K(w)⁻¹=WᵀW)을 도입해 대칭 KKT계 (19a): [[P Aᵀ Gᵀ],[A 0 0],[G 0 −WᵀW]][Δx;Δy;Δz]=[−r_x;−r_y;−r_z+Wᵀ(λ\r_s)]. 견고화를 위해 정적 정규화(ε_s I, 부호 분리)로 KKT를 quasidefinite화하여 어떤 순열에서도 LDLᵀ가 존재하게 하고, AMD 순서화로 fill-in 최소화, 동적 정규화(D_ii를 0에서 ε_d만큼 밀고 부호 보정)와 반복 정련(iterative refinement)으로 무정규화 원계를 고정밀로 회복. 초기화는 등식제약 QP의 KKT를 풀어 (x,y) 설정 후 s,z를 콘 내부로 시프트.

**3. 비판적 검토.**
- 알고리즘 골격이 cvxopt의 coneqp, Nesterov-Todd 스케일링, Mehrotra predictor-corrector, 정적/동적 정규화(Clarabel·CVXGEN·OSQP 선례), AMD/QDLDL 등 기존 기법의 신중한 조합으로, 알고리즘적 신규성보다 견고한 구현·엔지니어링 기여에 가깝다. 저자도 "Clarabel과 동일 알고리즘"임을 명시.
- 수렴 정리·반복 복잡도·정규화 오차 한계에 대한 자체 정리/증명이 본문에 없다. 핵심 기계(NT 스케일링, central path 수렴 τ→0)는 인용으로 대체되어 이론적 깊이는 응용수학 저널(Math. Prog. Comput.) 기준 중간 수준.
- 강점이 "small-to-medium" 규모로 명시적으로 한정되며, 대형 문제에서 Gurobi(멀티스레드)·Mosek에 뒤처짐. QOCO_custom의 우위는 데스크톱 캐시(L2 16MB) 내에 바이너리가 들어갈 때로 한정되고, CM4(L2 1MB)에서는 ~4000 이상 코드 크기로 인한 명령어 캐시 미스로 QOCO에 역전됨 — 코드 생성 방식의 본질적 확장성 한계.
- 동종 솔버 생성기 BSOCP·CVXGEN과의 직접 속도 비교가 라이선스 문제로 부분적(작은 oscillating-mass 40개에서만 CVXGEN 동급). 가장 직접적인 경쟁 도구와의 정량 비교가 제한적.

**4. 시뮬/실험.** 광범위한 벤치마크(sim). 자체 5개 클래스(robust Kalman, group lasso, lossless-convexified powered descent(lcvx), Markowitz portfolio, oscillating masses) 각 10크기×20인스턴스=1000문제; MPC 64문제([61], CM4 포함); Maros–Mészáros 138 QP; SuiteSparse 46 최소제곱. 외부 baseline: ECOS, Clarabel, Gurobi, Mosek, CVXGEN. 정량: 벤치마크 shifted-GM에서 QOCO 1.0 vs Clarabel 1.6/ECOS 2.5/Mosek 1.8/Gurobi 10.2, 실패율 전부 0%; 커스텀 포함 시 QOCO_custom 1.0 vs QOCO 2.0. MPC(데스크톱) shifted-GM QOCO_custom 4.4/QOCO 4.5/Clarabel 1.0/ECOS 31.4/Gurobi 1.7/Mosek 2.5(63문제 한정 비교에서 custom 최속). Maros–Mészáros QOCO 2.4(Gurobi 1.0), 실패율 QOCO 6.5%. 100회 최소 런타임 보고. 코드/바이너리 크기·생성/컴파일 시간 표 제공.

**5. Contribution(delta).** 출판년(2026) SOTA 대비: (i) QO-SOCP를 P^{1/2} 재정식화 없이 직접 푸는 오픈소스 C 솔버 — Clarabel(Rust)이 유사 기능을 하나, 레거시 항공우주 소프트웨어가 선호하는 C로 제공하고 homogeneous embedding 대신 직접 KKT를 풀어 small/medium에서 더 빠르고 견고함을 실증. (ii) QP만 다루던 CVXGEN, 라이선스 닫힌 BSOCP와 달리, 2차 목적 SOCP용 오픈소스 커스텀 솔버 생성기(QOCOGEN)를 최초로 제공하고 정적 메모리 할당으로 임베디드 적합. delta는 새 알고리즘이 아니라 "open-source + C + QO-SOCP 직접해법 + SOCP 코드생성"의 빈자리를 채우는 도구적 기여.

**6. Limitation·Future work.** 저자: 대형 문제 열세, 코드 생성·컴파일 시간(>10분 for >10k, springMass_1은 코드생성 2521s), 캐시 초과 시 성능 저하, 고정 희소성 패턴에만 custom 적용. 내가 보는 한계: 자체 수렴/정밀도 이론 부재, 가장 직접적 경쟁(BSOCP/CVXGEN) 정량 비교 제한, 항공우주 적용은 lcvx 벤치마크·동기 서술 수준이며 실제 비행/HIL 검증은 없음(도구 자체는 임베디드 배포 가능성만 제시). 향후: 무한차원/지수콘 확장, 대형 멀티스레드 인수분해, 실제 G&C 온보드 배포 실증이 영향력 확대에 필요.

**평가:** 71/100 (good) · 한줄평: 견고하게 구현된 오픈소스 QO-SOCP 솔버+SOCP 코드 생성기로 SCP/MPC/착륙유도 파이프라인에 직접 쓰일 도구적 가치가 크나, 알고리즘 신규성·자체 이론은 제한적인 near-domain(optim-tool) 기여.

---

## (2026, IEEE TAES, Customized Interior-Point Methods Solver for Embedded Real-Time Convex Optimization: Jang·Lee)

**1. 핵심 아이디어.** 임베디드 실시간 G&C 응용을 겨냥한 맞춤형 2차 원뿔(SOCP) 솔버를 제안한다. 예측-수정형 PDIPM에 동차 임베딩(homogeneous embedding) 프레임워크를 결합하되, 기존 self-dual 임베딩과 달리 2차 비용항(quadratic cost)을 슬랙 변수·SOC 제약으로 재정식화하지 않고 MCP의 동차 임베딩(HMCP)에 직접 수용하여 sparsity 손실을 방지한다. 여기에 문제군(problem family)의 sparsity 패턴을 오프라인 분석해 외부 의존성 없는(math.h만 사용) C 맞춤 솔버 코드를 자동 생성하는 코드 생성 도구를 함께 제공한다.

**2. 알고리즘.** 대상 문제는 quadratic-cost SOCP: minimize (1/2)x'Qx+q'x s.t. Ax=b, Gx+s=h, s∈K. KKT를 단조 상보성 문제(MCP)로 쓰고, 2차항을 명시적으로 포함한 HMCP(식 11)로 lifting하여 원문제 가능성과 무관하게 항상 가능한 중심경로(central path)를 확보, 최적성/비가능성 인증서(τ*>0 / κ*>0)를 제공한다. 탐색방향은 Nesterov-Todd 스케일링(W=H(w)^{-1/2}) 하에 Mehrotra 예측-수정으로 계산하며, SOC 스케일링 행렬의 diagonal-plus-rank-one 구조(식 43)를 sparse하게 전개하여 KKT 행렬 K를 quasi-definite로 정적 정규화(δ_s I) 후 부호 변화 없는 LDL^T 분해(PK̃P'=LDL', 동적 정규화 + 반복 정련). 모든 RHS가 같은 K를 공유하므로 분해는 반복당 1회. 코드 생성기는 AMD ordering·symbolic factorization을 오프라인 수행, 정적 메모리 할당(no malloc)·상삼각만 사용·loop 기반으로 문제 크기에 거의 무관한 코드를 생성.

**3. 비판적 검토.**
- 이론적 신규성의 상당 부분(HMCP의 가능성·인증서 성질, NT 스케일링, Mehrotra 휴리스틱)은 [35],[36],[41]에서 가져온 것으로, 본문의 정리는 인용에 의존하고 자체 증명/수렴률 분석은 없다.
- 가장 가까운 선행인 Clarabel이 이미 quadratic-cost HMCP를 IPM으로 구현·검증했음을 저자도 인정한다. 본질적 delta는 임베디드 C·코드생성·정적할당이라는 구현/배포 측면이며, 알고리즘적 차별성은 제한적이다.
- 벤치마크가 강력하나 솔버 자체 코드/데이터 공개 언급이 없어 외부 재현이 어렵다(파라미터는 충분).
- Mars landing·quadcopter 결과는 결정론적 nominal 케이스로, 외란·모델오차·MC 등 G&C 강건성 검증은 범위 밖이다(설계상 비가능성 검출이 robustness의 한 형태이긴 함).
- 대규모(n>500) 문제에서 IPM의 본질적 확장성 한계로 ADMM(SCS) 대비 성능 저하를 저자도 인정.

**4. 시뮬/실험.** (sim) 표준 벤치마크: Maros-Mészáros QP 138문제, 포트폴리오 100 인스턴스(비영 338~54,355), LASSO 100 인스턴스(비영 108~1,395,786); 외부 baseline ECOS·MOSEK·SCS와 performance/absolute profile 비교에서 소·중규모 전반 우위. (real/embedded) AMD Zynq-7000 ARM Cortex-A9 650MHz/512MB, -O3. Mars 동력하강(lossless convexification, 식 72) N=25~500: 자체 해와 ECOS 차이 10^{-7}~10^{-1}, 잔차 10^{-11}~10^{-8}, 비가능 케이스(t_f=25s)도 7~8회 반복 내 κ*>0,τ*≈0로 비가능성 검출. Quadcopter MPC N=15~100: ECOS 대비 차이 10^{-5}~10^{-3}, 임베디드에서 ECOS·SCS보다 빠른 solve time. ECOS와는 소규모서 유사, N≥150 대규모서 본 솔버가 우위.

**5. Contribution(delta).** 출판년(2026) 기준으로, (i) 2차 비용 SOCP를 재정식화 없이 동차 임베딩에 직접 수용해 sparsity 저하를 막는 HMCP를 임베디드 맞춤 IPM에 적용, (ii) 외부 의존성 없는 C 코드와 parsing 정보까지 자동 생성하는 코드생성 워크플로를 제시한 점이 핵심. CVXGEN·ECOS·QOCO(선형비용 한정 codegen)와 Clarabel(범용·Julia/Rust)의 간극, 즉 "quadratic-cost + 임베디드 C + 정적할당 + codegen"을 동시에 메운다. 다만 알고리즘 자체는 기존 구성요소의 견고한 조합에 가깝다.

**6. Limitation·Future work.** 저자: IPM 확장성 한계로 대규모 문제 성능 저하, 일부 코드 크기 증가 완화는 loop 기반에 의존. 추가로 보는 한계: 자체 수렴/복잡도 증명 부재, 코드 비공개로 재현성 제약, robustness(외란·MC) 검증 부재, Clarabel 대비 정량 비교 부재(직접 baseline 아님), warm-start·실시간 갱신(MPC 반복) 성능의 정량 분석 부족. Future: 대규모용 hybrid(IPM+1차법), 공개 구현, 실비행/HIL 확장.

**평가:** 72/100 (good) · 한줄평: 견고하게 엔지니어링된 임베디드 quadratic-cost SOCP 솔버로 실제 ARM 하드웨어에서 동력하강·MPC를 실시간 처리해 온보드 G&C 도구로서 직접적 관련성이 높으나, 알고리즘 신규성은 선행(Clarabel 등) 위의 구현 중심 delta에 머문다.
