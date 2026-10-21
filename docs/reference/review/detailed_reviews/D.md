# Thread D — Hybrid·Switched Systems · Saltation · State-Triggered Constraints (병합·재교정)

> 스레드 D 통합본(D1+D2+D3, 15편). 'phase 전환 트리거(guard 임계 θ)를 결정변수로' 잡는 것의 결정론·확률 선행과 saltation 도구·hybrid trajopt 응용을 한 묶음으로 본다.
> 재교정: D1/D2/D3 part별로 달랐던 기준(D2의 all-4 인플레이션 포함)을 단일 엄격 앵커로 통일했다 — 5=분야선도(드묾)/4=진짜 강함/3=견실·보통/2=약함. 증분·조합은 신규성(C2)을 누르고, 미측정 실시간·niche 컨퍼런스·단일 toy 예제는 정직히 반영한다.
> 공정성: 품질≠명성·venue offset 금지·post-hoc fudge 금지. 일반 로보틱스/hybrid 도메인은 RLV 직접성이 아니라 *그 분야 영향*으로 C8을 매기고, 서베이/튜토리얼은 reference value를 인정한다. 결정론-by-design 논문은 위험모델(C4)을 N/A로 재정규화하되, 불확실성/chance를 *주장*하면 채점한다(Gaussian·1차근사=3). 수식 Unicode/plain.

---

## (2005, DEDS, Optimal Control of Switching Surfaces in Hybrid Dynamical Systems: Boccadoro et al.)

**1. 핵심 아이디어.** 스위치드모드 하이브리드 시스템에서 모드 전환을 결정하는 것은 상태궤적이 상태공간 내의 "스위칭 곡면(switching surface)"을 가로지르는 사건이다. 본 논문은 전통적으로 설계변수로 다루어 온 스위칭 *시각(time)* 대신, 이 곡면을 매개화하는 유한차원 파라미터 a_i를 설계변수로 채택한다. 즉 곡면 파라미터가 상태 피드백 형태의 스위칭 법칙을 정의하며, 비용함수 J를 이 파라미터의 함수로 보고 코스테이트(costate) 기반의 기울기 공식을 유도한 뒤 경사하강법으로 최적화한다.

**2. 알고리즘.** 시스템은 x_dot=f_i(x), 곡면은 g_i(x,a_i)=0, 비용은 J=∫_0^T L(x)dt. 핵심 결과는 (i) dJ/da_i = -(1/L_i)(dJ/dτ_i)(∂g_i/∂a)  (Prop. 2.1, L_i := (∂g_i/∂x)f_i는 g_i의 f_i 방향 Lie 미분), (ii) dJ/dτ_i = p_i(τ_i)^T R_i  (R_i := f_i(x_i)-f_{i+1}(x_i)는 전환시 벡터장 점프), (iii) 코스테이트는 구간별로 p_dot = -(∂f_{i+1}/∂x)^T p - (∂L/∂x)^T, 경계조건 p_N(T)=0, 그리고 전환점에서 불연속 점프 p_{i-1}(τ_i)=(I - (1/L_i)R_i ∂g_i/∂x)^T p_i(τ_i)  (식 40). 절차: 식(2)로 상태를 전향 적분 → 식(39),(40)으로 코스테이트 후향 적분 → 식(38),(10)으로 기울기 산출 → Armijo 스텝 경사하강.

**3. 비판적 검토.**
- 이론 전개는 완결적이다. Lemma 2.1~2.3, Prop. 2.1~2.2가 모두 자체 귀납 증명을 갖추고 천이행렬 Φ와 보조행렬 Θ_{j,i}의 재귀관계까지 명시적으로 닫힌다. 핸드웨이빙이 거의 없다.
- 그러나 핵심 가정 2.2 (모든 i에 대해 L_i≠0)는 본질적 한계다. 궤적이 곡면에 접(tangent)하게 접근하면 L_i=0이 되어 비용함수 자체가 불연속·미분불가능해지고 기울기 공식이 붕괴한다. 저자도 §3 말미에서 이를 인정하며 "진행 중 연구"로 미룬다. 즉 가장 흥미로운(가장 까다로운) 경계 사례에서 방법이 작동하지 않는다.
- 검증이 약하다. 2D unicycle 단일 시나리오, 파라미터 2개(a_1,a_2)만 최적화, 외부 baseline 없음, 몬테카를로 없음. 알고리즘이 "잘 작동한다"는 것을 보일 뿐, 스위칭 시각 기반 기존법 대비 정량 비교가 전혀 없다.
- 모드 시퀀스가 고정되어 있다고 가정한 채(곡면 파라미터 섭동이 시퀀스를 바꾸지 않는 국소 영역) 미분을 전개한다. 이는 수학적으로는 정당하나 실제 설계에서 시퀀스 변경이 빈번한 영역에서는 적용성이 제한된다.

**4. 시뮬/실험.** 단일 수치예제 (sim): 장애물 회피 unicycle 로봇, 동심원 두 개(반경 a_1,a_2)를 가드로 사용. 파라미터 C_1=1.2, C_2=0.5, T=3, α=10, β=10, 목표(2.25,2), 장애물(1,1), 초기(0,0,0). Armijo 경사하강 70 반복으로 비용 J가 9.62→9.2로 감소, 기울기 크기 0.57→~0으로 수렴(Fig.2). a_1은 거의 불변, a_2는 0.7→0.61로 수렴. baseline 비교·통계·MC 없음.

**5. Contribution(delta).** 출판년(2005) 기준, 선행 연구(Egerstedt et al. 2003; Xu & Antsaklis 2002; Shaikh & Caines)는 모드 시퀀스가 주어졌을 때 스위칭 *시각*을 설계변수로 다루었다. 본 논문의 delta는 설계변수를 스위칭 *곡면의 파라미터*로 전환하여, 결과적으로 상태 피드백(폐루프) 스위칭 법칙을 얻고 초기조건 변동에 대한 강건성 이점 및 가드를 여러 번 통과해도 단일 파라미터로 표현 가능한 점을 제공한 것이다. dJ/da를 dJ/dτ로 연결(Prop.2.1)하고 이를 코스테이트로 닫는 공식화는 당시로서는 명확한 진전이다.

**6. Limitation·Future work.** (저자) L_i=0 접선 접근 시 비용 불연속 문제 미해결, 향후 장애물 밀집 환경의 항법 설계로 확장. (추가) 검증이 toy 단일 예제에 그쳐 일반 비선형/고차원 시스템에서의 거동·수렴성·계산비용이 미검증, 외부 baseline 부재, 모드 시퀀스 고정 가정의 실용적 제약, 실시간/온보드 적용성 분석 전무. 강건성·불확실성 처리는 설계상 다루지 않음.

**평가:** 70/100 (good) · 한줄평: 하이브리드 최적제어의 곡면-파라미터 기울기 공식을 엄밀히 유도한 견실한 control-theory 기여이나 검증이 toy 수준이며, 항공우주 G&C와는 (스위칭 곡면 설계라는 점에서) 방법론적 근접성은 있으나 직접 관련성은 간접적이다.

---

## (2006, IEEE TAC, Transition-Time Optimization for Switched-Mode Dynamical Systems: Egerstedt et al.)

**1. 핵심 아이디어.** 모드 시퀀스(modal sequence)가 고정된 스위치드 동역학 시스템에서, 모드 전환이 일어나는 스위칭 시각 τ_i를 최적화하여 비용범함수 J를 최소화하는 문제를 다룬다. 핵심 기여는 비용범함수의 스위칭 시각에 대한 기울기(gradient)를 코스테이트(costate)로 표현하는 매우 단순한 닫힌형 공식을 유도하고, 이를 경사 투영(gradient-projection) 알고리즘에 사용하는 것이다. 나아가 고정 시퀀스에 새로운 모드를 삽입(insertion)할 때의 일방향 미분 공식까지 확장하여, 모드 스케줄링이라는 혼합정수 문제에 연속적 민감도 정보를 제공한다.

**2. 알고리즘.** 자율 스위치드 시스템 ẋ = f_i(x), t∈[τ_{i-1},τ_i], 비용 J = ∫_0^T L(x(t))dt. 코스테이트는 역방향 ODE ṗ = -(∂f_{i+1}/∂x)^T p - (∂L/∂x)^T, p(T)=0 (구간별)으로 정의. 스위칭 시각 기울기는 dJ/dτ_i = p(τ_i)^T ( f_i(x(τ_i)) - f_{i+1}(x(τ_i)) ) (식 22). 모드 삽입(시각 τ에 g_α 삽입)의 일방향 미분은 dJ/dλ^+(0) = p(τ)^T ( g_α(x(τ)) - g_{α(i)}(x(τ)) ) (식 25). 제약 0=τ_0≤τ_1≤…≤τ_N≤τ_{N+1}=T 하에서 KKT 최적성 조건을 유도하고, Armijo 스텝의 경사 투영 알고리즘(Algorithm 3.1)으로 강하 방향 ĥ(τ̄)를 계산한다.

**3. 비판적 검토.**
- 기울기 공식 유도는 엄밀하나, 핵심 기술 보조정리 Lemma 2.1은 Polak(Ch. 5.6.2) 및 [13]에서 "동일한 논거"라며 가져왔고, ĥ가 -q̄의 투영임을 보이는 증명은 "본론에서 벗어난다"며 생략되었다 — 자체 증명의 완결성에 일부 공백이 있다.
- 수치 예제가 2모드 2차원 선형 시스템 단 하나로, 저자 스스로 "academic nature"임을 인정한다. 외부 baseline 비교, 몬테카를로, 통계 처리가 전무하다.
- 비자율(non-autonomous, 입력 u(t) 포함) 경우는 다루지 않고 자율 시스템에 한정된다. 또한 NP-hard인 일반 모드 스케줄링은 미래 과제로 미루고 단순 삽입만 제시한다.
- 알고리즘은 명백히 오프라인 최적화이며, 실시간/온보드 연산 비용이나 복잡도 분석이 없다(선형 수렴만 언급).
- 강하 방향은 국소 최적(KKT)만 보장하며 전역성·다중 국소해 문제는 논의되지 않는다.

**4. 시뮬/실험.** 단일 학술 예제: 2모드 선형 시스템(A1, A2는 2×2, 각각 양/음 고유값 1개), x0=(1,1)^T, T=10, 비용 J=(1/2)∫_0^10 ‖x‖^2 dt, 그리드 Θ는 200점. Algorithm 3.1로 J를 113.74(무스위칭) → 24.53(2스위치, 9회 반복, ‖∇J‖ 99.94→0.18)로 감소시키고, 모드 삽입(Step 2) 후 4모드 스케줄에서 24.45 → 22.40(17회 반복, ‖∇J‖=0.26)까지 감소. 모두 (sim), baseline/MC/실측 없음.

**5. Contribution(delta).** 출판년(2006) 기준, 선행 Xu-Antsaklis [21]의 비선형 프로그래밍식 기울기·헤시안 계산보다 "더 단순한" 코스테이트 기반 기울기 공식을 제시한 것이 1차 delta다. 더 중요한 2차 delta는 모드 삽입 일방향 미분(식 25/26)으로, 이는 이후 스위치드 시스템의 모드 스케줄링(mode-insertion gradient) 분야의 기초 도구가 되었다. 고정 시퀀스 스위칭-시각 최적화에서 가변 시퀀스(삽입)로 가는 연속적 민감도 다리를 놓은 점이 당시 SOTA 대비 명확한 진전이다.

**6. Limitation·Future work.** 저자 인정: 일반 모드 스케줄링(시퀀스·개수가 변수)은 혼합정수·NP-hard로 별도 논문에 미룸; 예제가 학술적임. 내가 보는 한계: 비자율 시스템·입력 결합 미처리, 불확실성/강건성 다룸 전무, 단일 toy 검증으로 실용성 입증 부족, 온보드 실시간성 분석 부재, 전역 최적성 미보장. G&C 관점에서는 직접적 항공우주 검증이 없어 도메인 거리가 있다.

**평가:** 72/100 (good) · 한줄평: 스위칭-시각 및 모드-삽입 기울기의 깔끔한 코스테이트 공식으로 하이브리드 최적제어에 지속적 영향을 남긴 견고한 control-theory 논문이나, 단일 toy 검증·불확실성 부재로 G&C 직접 관련성은 간접적이다.

---

## (2008, Automatica, Probabilistic reachability and safety for controlled discrete time stochastic hybrid systems: Abate et al.)

**1. 핵심 아이디어.** 제어 입력을 갖는 이산시간 확률 하이브리드 시스템(DTSHS)에 대해 유한 구간 확률적 도달성(reachability) 문제를 정식화한다. 시스템 궤적이 안전 집합 A 안에 머무를 확률을 최대화하는 문제를 확률적 최적제어 문제로 임베딩하고, 이를 곱셈형(multiplicative) 비용과 최대(max) 비용이라는 두 상보적 해석으로 환원한다. 두 해석 모두 동적계획법(DP)으로 풀려 '최대 안전(maximally safe)' Markov 정책과 안전수준별 maximal probabilistic safe set을 산출한다. 더 나아가 regulation/stabilization 문제를 (시변) 안전 문제로 재해석할 수 있음을 보인다.

**2. 알고리즘.** DTSHS H=(Q,n,U,Σ,Tx,Tq,R)를 임베디드 controlled Markov process로 환원한다. 안전 확률 p^π_{s0}(A)=E[∏_{k=0..N} 1_A(s(k))] (곱셈형, 식12), 또는 p^π_{s0}(Ā)=E[max_{k∈[0,N]} 1_{Ā}(s(k))] (max형, 식13). 곱셈형 DP(Thm1): V*_k(s)=sup_{(u,σ)} 1_A(s)H(s,(u,σ),V*_{k+1}), 종단 V*_N=1_A; 여기서 H는 stochastic kernel Ts에 대한 적분 연산자(식15). max형 DP(Thm2): W*_k(s)=inf_{(u,σ)} (1_{Ā}(s)+1_A(s)H(s,(u,σ),W*_{k+1})). Lemma3: W^π_k(s)=1−V^π_k(s)로 두 표현의 동치성을 증명. 최적 정책 μ*_k(s)∈arg sup_{(u,σ)} H(s,(u,σ),V*_{k+1}).

**3. 비판적 검토.**
- 측정가능성(universal measurability, lower-semianalytic) 처리와 최대안전 정책 존재성에 대한 compactness 충분조건까지 자체 증명을 제시하나, 핵심 증명 골격은 Bertsekas & Shreve(1996, Prop 7.45-7.50)에서 수입하여 하이브리드·곱셈/max 비용 케이스로 확장한 것이다.
- 연속 상태에 대한 DP는 격자화(gridding)를 요구하여 차원의 저주에 노출되며, 저자도 차원 증가가 trade-off임을 명시한다. 실제 알고리즘적 계산 가능성에 대한 정량 분석(격자 해상도-정확도, 계산복잡도)이 없다.
- 검증이 r=2 방 온도조절이라는 단일 toy 예제에 국한되고 외부 baseline·비교가 전무하다. 방법의 본질적 가치 대비 실증 폭이 얇다.
- max-cost와 multiplicative-cost의 동치성은 우아하나, 실무적으로 어느 표현이 수치적으로 유리한지에 대한 논의는 제한적이다(논문은 multiplicative 채택).

**4. 시뮬/실험.** r=2 방 온도조절(Δ=10min, N=60, 600min). 안전구간 [17.5,22]°C, 파라미터 명시(a12=a21=0.022, b1=0.0167, b2=0.022(대칭), c1=0.8, c2=0.9333, ν²=0.06, xa=6). 전이확률 0.8/0.1/0.1 (지연·결함 모델 포함). MATLAB로 DP 수행, maximally safe 정책 하의 샘플 경로(Fig.2)와 안전수준별(0.5~0.97) maximal safe set 등고선(Fig.3), k=0 최적 전이입력 영역(Fig.4) 제시. 정량 baseline 없음, MC 비교 없음 (sim).

**5. Contribution(delta).** 출판년(2008) 기준, *제어 입력을 갖는* DTSHS에 대한 확률적 도달성·안전성 문제를 최초로 체계적으로 정식화하고, 이를 곱셈형/max형 DP의 dual 해석으로 푸는 통일된 프레임을 제시했다는 점이 핵심 delta다. 기존 CTSHS·uncontrolled 도달성(Bujorianu&Lygeros, Prandini&Hu) 및 deterministic reachability-optimal control 연결(Lygeros 2004)을 넘어, regulation/stabilization을 안전 문제로 환원하는 관점을 도입했다. 이는 후속 stochastic reachability 문헌의 토대가 되었다.

**6. Limitation·Future work.** 저자: 무한구간·practical stabilization 확장, 시변 안전집합 처리, 추가 상태(예: commutation 횟수 제한)로 인한 차원 증가 trade-off. 내가 보는 한계: 연속상태 DP의 확장성(scalability)과 onboard/실시간 적용 불가, 단일 toy 검증과 외부 baseline 부재, G&C(항공교통·충돌회피)로의 적용은 저자들의 별도 연구를 통한 간접적 연결에 그쳐 본 논문 자체는 도메인 일반 제어이론 결과다.

**평가:** 76/100 (strong) · 한줄평: 제어형 확률 하이브리드 시스템 도달성·안전성의 토대를 세운 이론적으로 견고한 논문이나, 검증은 toy에 그치고 G&C 관련성은 항공교통·확률적 안전제어로의 간접 전이를 통해 성립한다.

---

## (2020, JGCD, Dual Quaternion-Based Powered Descent Guidance with State-Triggered Constraints: Reynolds et al.)

**1. 핵심 아이디어.** 단위 이중 사원수(unit dual quaternion)로 강체의 자세와 위치를 동시에 표현하여 6자유도 자유종단시간 연료최적 동력하강 유도 문제를 정식화한다. 회전과 병진을 동시에 구속하는 제약(접근콘, 틸트각, 시선각)을 이중 사원수 형태로 자연스럽게 표현하고, 특정 사거리 구간에서만 광학센서 시선(LoS) 제약을 활성화하는 slant-range-triggered LoS 제약을 새로 도입된 state-triggered constraint(STC)로 모델링한다. 비볼록 최적제어 문제를 순차 볼록화(successive convexification)로 SOCP 시퀀스로 풀어 실시간 구현을 노린다.

**2. 알고리즘.** 운동학은 dq/dt = (1/2) q⊗ω(이중 사원수 운동학식 17), 동역학은 J ω' + ω⊘Jω = Φ u_B + m g_B(식 19, dual inertia J는 항상 가역). 질량은 m'=-α||u_B||(식16). 제약: 접근콘 c_g=−q^T M_g q + ||2E_d q||cosγ_max≤0, 틸트 c_t=q^T M_t q + cosθ_max≤0, 추력 u_min≤||u_B||≤u_max, 짐벌 ||u_B||≤sec δ_max z_B^T u_B. STC는 트리거 g(z)<0 ⇒ 제약 c(z)≤0 을 연속형 h(z)=σ(z)c(z)≤0, σ(z)=−min(0,g(z))로 변환; compound-and 형은 h=(∏ σ_i)·c≤0. 해법: 시간 정규화(τ∈[0,1], dilation s)→1차 테일러 LTV 선형화→RK4로 상태천이행렬·이산화(식45)→trust region(식48) + virtual control(식51) + affine scaling(식53). 신규: defect Δ_i(식47)로 trust-region 가중치 w_tr를 적응적으로 선택(식50). 종료조건 max ||P_x^{-1}(x_i−x̄_i)||_∞ < δx_tol.

**3. 비판적 검토.**
- 제약 함수들의 볼록성(c_g, c_t, c_l)과 STC 연속형 등가성은 모두 선행연구[36,37,41,42]에서 인용되며 본 논문에서 재증명되지 않는다. 순차 볼록화의 수렴성 보장도 본 조합에 대해서는 주장하지 않고 국소최적·실증적 수렴에 머문다(C1 상한 요인).
- 베이스라인 비교가 self-baseline이다. "baseline problem"(STC 미적용) 대 "Problem 1"(STC 적용)의 자기비교일 뿐, 외부 경쟁기법(예: MPC형 이중사원수 유도, 혼합정수 STC)과의 정량 비교가 없다.
- 검증이 단일 동역학 영역(달 하강, Apollo급)에 국한된다. 화성 대기·항력, 다엔진, 가변관성 등은 다루지 않거나 선행연구로 미룬다.
- 실시간성 주장이 데스크톱(3.2GHz i5, C++/MEX) 기준이며 임베디드/비행 하드웨어 실증은 아니다. 저자도 결론에서 C 구현을 향후 과제로 명시한다.
- 노드 간(inter-node) 제약 위반 가능성을 저자 스스로 언급하나 정량 분석은 생략된다.

**4. 시뮬/실험.** 두 사례 연구(모두 sim). (i) STC 사례: N=35 노드, 광학센서 boresight p_B=[0.91,0,−0.42], FoV 20°, ρ∈[200,450]m에서 LoS 활성화; STC 적용 시 LoS 제약을 사거리 구간 내에서 충족하고 베이스라인은 미충족함을 정량적으로 도시(Fig.5). (ii) 몬테카를로: 1000 trial, 질량 ±10% 균일분포, 속도 σ=7m/s(수평)·4m/s(수직), 위치는 3-DoF 제어가능 polytope에서 hit-and-run 샘플. 개루프 오차 목표 ||r−r_f||≤10m, ||v−v_f||≤15cm/s. 결과: 971/1000 성공(97.1%), 실패 29건을 3-DoF 초기추정으로 재해결 시 28건 추가 성공→99.9%. 3σ 신뢰타원이 속도 요구치와 약간 겹침. 계산시간: 평균 0.75s, 3σ에서 <2.21s(1-tailed); 반복당 propagation 8.3±0.9ms + solve 64.8±15.3ms(solve가 89%). 개루프 오차요구·N·δx_tol 관계도 맵으로 제시(N≈10–18, δx_tol≤0.05면 1s 미만).

**5. Contribution(delta).** 출판년(2019 arXiv/2020 JGCD) SOTA 기준으로: (i) 단위 이중 사원수를 처음으로 feedforward 6-DoF 궤적생성에 적용(선행 [35,37,48,49]는 모두 폐루프 피드백 제어). (ii) 신규 도입된 compound STC를 slant-range-triggered LoS 제약에 적용하여, HDA/안전착륙지 선정에 필요한 구간한정 포인팅 요구를 실시간 친화적 유도법에 융합한 최초 사례. (iii) defect 기반 적응적 trust-region 가중과 dual-quaternion 전용 스케일링으로 수렴을 가속하는 휴리스틱. 개별 구성요소(이중사원수 동역학, 순차볼록화, STC)는 기존 것이나 이를 실시간 6-DoF 유도로 통합·정량화한 점이 delta다.

**6. Limitation·Future work.** 저자: 최적성·강건성·수렴성의 추가 연구, 대표 우주기 하드웨어용 실시간 C 구현이 핵심 향후 과제. 내가 보는 한계: 외부 baseline 부재로 상대 우월성 입증이 약함; 볼록성·수렴 보장이 인용 의존; 검증이 달 단일 시나리오; 노드 간 제약 위반·항법오차(추정 초기상태)의 영향 미분석; 자세 초기값을 자유변수로 두는 선택이 수렴엔 유리하나 실제 임무 제약과의 정합성은 추가 검토 필요.

**평가:** 78/100 (strong) · 한줄평: 이중 사원수 6-DoF 유도에 compound STC를 실시간 친화적으로 융합하고 1000-trial MC + 0.75s 런타임으로 뒷받침한, G&C 동력하강 유도의 견실한 강(strong) 논문이나 외부 baseline 부재와 인용 의존 증명이 flagship에는 못 미친다.

---

## (2020, JGCD, Successive Convexification for Real-Time Six-Degree-of-Freedom Powered Descent Guidance with State-Triggered Constraints: Szmuk et al.)

**1. 핵심 아이디어.** 자유 종단시간(free-final-time) 6-DoF 동력 하강 유도 문제를 successive convexification(SCvx) 프레임으로 실시간 풀이한다. 세 가지 기여를 제시한다: (i) 점화시점을 최적화 변수로 두는 free-ignition-time 수정, (ii) 양력·항력을 동시에 모델링하는 타원체(ellipsoidal) 공력 모델, (iii) 핵심 기여인 state-triggered constraint(STC) -- 해(solution) 변수에 조건부로 enable/disable되는 'if-문'형 제약을 연속 최적화 안에서 표현하는 정식화이다. STC는 속도-trigger 받음각 제한, 거리-trigger 시선각(LOS) 제약 등에 활용된다.

**2. 알고리즘.** STC의 논리식: g(z)<0 ⇒ c(z)=0 (trigger 함수 g, 제약 함수 c). 이를 연속화한 cSTC는 보조변수 σ≥0로 σ⊥c(z), g(z)+σ≥0를 두고, LCP 해석해 σ̂(z)=-min(g(z),0)를 대입해 등식 h(z) := -min(g(z),0)·c(z)=0 (부등식 ≤0로 완화)으로 만든다. q-α STC 예: ||v_B||>V_α ⇒ -e1·v_B ≥ cosα_max·||v_B||. 전체는 minimize -m(t_f)의 비볼록 OCP(Problem 1)를 normalization(dilation factor s=t_b로 시간정규화) → 1차 Taylor 선형화 → FOH 이산화(상태천이행렬 Φ_A) 과정을 거쳐 SOCP 부분문제(Problem 2)로 변환. 인공 비실현성에 가상제어 ν(1-노름 페널티 w_v||ν||_1), 인공 비유계성에 soft trust region δz^T W_tr δz를 추가하여 수렴까지 반복.

**3. 비판적 검토.**
- 수렴성 보장이 없음을 저자도 명시한다. cSTC의 min(·) 비매끄러움을 g>0에서 ∂h/∂z=0으로 정의해 처리하나, 이로 인한 선형화 부분문제의 well-posedness/수렴은 경험적으로만 뒷받침된다([34]의 수렴이론은 가상제어 不要일 때만 적용).
- STC의 핵심 한계는 contrapositive 누설이다. g(z)<0⇒c(z)=0의 역(c=0이 g<0를 함의하지 않음)으로 인해, 본문 IV.B에서 보듯 속도 제약을 타려고 받음각이 제한을 넘는 구간이 자발적으로 생길 수 있다(저자도 지적). 단방향 if만 보장되며 사용자 의도와 어긋날 여지가 존재.
- 노드 간(inter-node) 제약 위반을 보장하지 않는다(K=20 노드에서만 강제). 이산화 선택이 정확도가 아닌 최적성/실현성에 영향이라 명시하나, 안전 임계 제약(받음각·LOS)에서 노드 간 위반은 실질적 위험.
- 검증이 전적으로 비차원화 합성 시나리오에 한정. 외부 baseline(MIP/complementarity 직접 비교, 또는 다중위상 정식화와의 정량 비교)이 없어 cSTC의 우월성은 정성적 논증에 머문다. 분산(dispersion)/Monte Carlo 강건성 분석 부재.

**4. 시뮬/실험.** 합성·비차원 시나리오. 세 case study: (A) 공력모델(B/SA/EA), (B) q-α STC(EA vs EA+ST), (C) free-ignition-time(EA vs EA+FI). K=20 노드, 평면/비평면 기동. 정량(Table 3): 예) (B) burn 4.42 U_T·fuel 3.17%; (B)+(EA)+(ST) 4.06·3.40%; (B)+(EA)+(FI) 2.56·3.13%로 FI가 연료 절감, ST가 더 좋은 국소최적으로 유도(반직관적). 비선형 적분 검증 e_pos ~1e-2~1e-3 U_L, e_att ≤1e-6~1.0°. 타이밍(Table 4, 2014 MacBook Pro 2.2GHz i7, ECOS/CVX): solve time median 0.164~0.654s, 전 case 최대 <0.7s, std ms급. propagation은 C++/Eigen으로 ~10ms. 외부 baseline 없음(self 비교만).

**5. Contribution(delta).** 출판년(2018 arXiv/2020 JGCD) SOTA 기준: 기존 lossless convexification[15,21]은 3-DoF 추력제약에 한정, 6-DoF SCvx[36,37,38] 선행연구가 있었으나, 본 논문의 새 기여는 (i) 조건부 제약을 연속 최적화에 직접 임베딩하는 STC/cSTC라는 '새로운 정식화'다. MIP(이산변수)·complementarity(양방향 iff)와 달리 단방향 if를 연속변수로 표현하여 실시간성을 유지하면서 더 넓은 제약군(속도-trigger 받음각, 거리-trigger LOS)을 가능케 한 것이 핵심 delta. free-ignition-time과 타원체 공력은 부차적이나 실질적 확장.

**6. Limitation·Future work.** 저자: 수렴보장 미제공, 최적성 정량화 미흡, 대표 비행 HW 검증 필요, feedback/재계산 미다룸(feedforward 궤적생성에 한정). 내가 보는 한계: contrapositive 누설로 인한 안전제약의 비보장적 동작, 노드 간 위반, 외부 baseline·MC 강건성 분석 부재, 실제 행성착륙 파라미터가 아닌 비차원 합성 검증. STC가 이후 6-DoF 착륙·재사용발사체 유도 연구의 표준 빌딩블록이 되었다는 점에서 영향력은 명확.

**평가:** 84/100 (strong) · 한줄평: 조건부 if-제약을 연속 SOCP에 임베딩하는 STC라는 새 방향을 연 핵심 core-G&C 정식화로, 수렴보장·외부 baseline 부재가 아쉬우나 분야 영향력(32.8/yr)이 강하다.

---

## (2021, Automatica, The Salted Kalman Filter: Kalman filtering on hybrid dynamical systems: Kong et al.)

**1. 핵심 아이디어.** 상태구동(state-driven) 모드 전이와 비-항등(non-identity) 리셋맵을 갖는 하이브리드 동역학계에서 1·2차 모멘트를 전이 시점에 정확히 전파하는 칼만/확장칼만 필터 확장(SKF)을 제안한다. 핵심은 비평활 해석에서 온 saltation matrix(궤적 민감도의 불연속 갱신)를 공분산 갱신에 도입하여, 리셋맵 야코비안만 쓰는 단순 접근(JRKF)이 놓치는 모드별 동역학 차이 효과를 1차로 포착하는 것이다. 일반 상태추정/하이브리드 제어에 쓸 수 있는 추정이론 기여이나, 검증은 다족로봇형 접촉계에 국한된다.

**2. 알고리즘.** Saltation matrix Xi = D_xR + ((F_J - D_xR·F_I - D_tR)·D_xg)/(D_tg + D_xg·F_I). 전이 시 모멘트 갱신: mu(t+) = R(t,mu*), Sigma(t+) = Xi·Sigma(t)·Xi^T (+ 리셋 프로세스 노이즈). 비전이 구간은 표준 KF 예측/갱신(11)-(15). a priori/a posteriori 양쪽 모두 가드 조건 만족 시 전이를 트리거(Alg.1): 적분 중단 -> 부분 타임스텝 -> saltation 갱신 -> 모드 전환. 비선형은 F_I, R, h_I를 야코비안 선형화하여 EKF형으로 동일 구조 적용.

**3. 비판적 검토.**
- 핵심 도구(saltation matrix, 공분산 전파 법칙(6)-(7))는 선행연구 [7,12,29]에서 수입한 것으로, 본 논문의 독자적 정리·증명이라기보다 칼만 필터링 맥락으로의 응용·정식화이다. Prop.1도 [7]의 결과 인용.
- 측정 갱신이 전이를 트리거할 때 이미 갱신된 추정치를 단순 변환하는 처리는 "갱신이 작다"는 가정에 의존하며, 도메인별 서브-갱신으로 쪼개는 더 정확한 처리를 미수행(저자 인정).
- EKF와 동일한 약점 상속: 선형화 붕괴(초기 모드 오판, 추정치가 실궤적에서 크게 이탈, grazing impact)에서 성능 저하, 전이 후 분포가 Gaussian으로 유지되지 않아 최적성 상실.
- 베이스라인 JRKF는 사실상 자가 약화(strawman) 베이스라인 성격; 외부 베이스라인은 입자필터뿐이며 모두 시뮬레이션, 실하드웨어/HIL 부재.

**4. 시뮬/실험.** 두 시스템(선형 constant-flow, 비선형 ASLIP). Constant-flow: 80개 노이즈·타임스텝 조합 x 1000 MC, sign test(p<0.05) — SKF가 JRKF를 80개 중 76개에서 유의하게 능가, 나머지 4개는 통계적 동등, JRKF가 이긴 경우 없음(sim). 입자필터는 1000입자 이상에서만 SKF보다 MSE 낮음(2.7% 감소, 계산 941배), 2000입자 5.2% 감소·1736배; 타임스텝 충분히 작아 확률질량이 단일 스텝에 안 모일 때만 PF 우세(Table 1). ASLIP(2점프/2.5s, 비선형 비항등 리셋): 36조합 x 100 MC에서 SKF가 JRKF를 전부 유의 능가(p<0.001), 특히 수직 위치 y_b에서 지속 개선; PF(30,000입자) 대비 SKF MSE 유의 낮음(p<0.001), PF 22000배 느림(평균 5200s/1.25s 시뮬)(sim).

**5. Contribution(delta).** 출판년(2021) 기준, 상태구동 전이와 비항등 리셋맵을 갖는 일반 하이브리드계에서 분포의 2차 모멘트를 전이 시점에 1차 정확도로 전파하는 칼만/EKF형 온라인 추정기를 최초 제시. 선행 하이브리드 추정(IMM[10], filter bank[4], switched-system 한정[21,35])은 연속상태 의존 전이가 분포에 미치는 효과를 반영 못 하고, 다족로봇 추정[9,18]은 리셋=항등(야코비안=I) 가정. SKF는 saltation matrix로 이 간극을 메워 JRKF의 부정확성을 정량 규명하고 PF 대비 수배~수만배 계산 우위를 보임.

**6. Limitation·Future work.** 저자: 이산상태/측정의 모드 소속 확률을 명시적으로 추론하지 않음, 가드 교차(동시 다중 접촉) 미지원, 비선형·비국소·비가우시안 강한 경우 PF가 유리. 내가 보는 한계: 핵심 이론은 수입 도구의 응용이라 이론적 신규성 폭이 제한적; 실하드웨어/실시간 임베디드 타이밍 부재로 온보드 주장(C7) 미검증; 베이스라인이 JRKF/PF로 다소 약하고 G&C 도메인과의 직접 연결 없음. Future: Bouligand 미분 기반 가드 교차 확장, 회귀로 saltation 근사(완전 미분가능 모델 불요).

**평가:** 75/100 (strong) · 한줄평: saltation matrix를 칼만 필터에 접목한 정식화는 견고하고 MC·통계검정도 충실하나, 핵심 도구는 수입이고 검증이 다족로봇 접촉계에 국한되어 항공우주 G&C 직접 관련성은 간접적이며 측정 인용도(0.2/yr)도 낮다.

---

## (2021, CDC, iLQR for Piecewise-Smooth Hybrid Dynamical Systems: Kong et al.)

**1. 핵심 아이디어.** 접촉 모드가 바뀌는 하이브리드 동역학계(상태 점프 포함)에 대해 iLQR을 확장한다. 핵심은 backward pass에서 reset map의 단순 Jacobian(D_xR) 대신 saltation matrix Ξ를 사용해 하이브리드 전이 주변의 1차 변분을 정확히 선형화하는 것이다. forward pass는 event detection으로 모드 시퀀스/타이밍을 능동적으로 변경하며, reference extension으로 모드 불일치 시 유효한 제어 입력을 확보한다.

**2. 알고리즘.** Saltation matrix Ξ := D_xR + ((F_J − D_xR F_I − D_tR)D_xg)/(D_tg + D_xg F_I)로 사전/사후 전이 섭동을 매핑: δx(t̄_{j+1}) ≈ Ξ δx(t̄_j). 표준 iLQR의 Q 계수 전개(식 11–14)에서 전이 시점에 f_k 항을 (식 27–28)의 두 단계 선형화 δx(t̄_j)≈f_x δx+f_u δu, δx(t̄_{j+1})≈Ξ δx(t̄_j)로 대체하여 하이브리드 Q 계수(식 29–33)를 얻는다. 최적 입력 δu*=−Q_uu^{-1}(Q_u+Q_ux δx), V_x=Q_x−Q_u Q_uu^{-1}Q_ux. 전이 비용 J_{N_j}를 추가해 모드 전환 횟수에 패널티 가능.

**3. 비판적 검토.**
- saltation matrix 자체는 비평활 역학 문헌(Leine-Nijmeijer, Aizerman-Gantmacher 등 [17]-[20])에서 가져온 도구로, 본 논문의 기여는 그것을 iLQR backward pass에 결합한 통합이지 새로운 이론적 결과의 증명은 아니다. 1차 전개에 그쳐 DDP의 2차 변분(전이 시점)은 어렵다고 명시적으로 회피한다.
- isolated transverse 전이, no-Zeno, 모든 vector field가 fully controllable이라는 강한 가정에 의존한다. 특히 controllability 가정은 실제 underactuated 시스템(쿼드콥터 perching)에서 정당화가 약하다.
- 수렴/최적성에 대한 형식적 보장이 없다. "Ξ-iLQR가 더 잘 수렴한다"는 주장은 전적으로 경험적이며, D_xR-iLQR이 발산하는 이유도 추측(speculate/postulate)에 머문다.
- 외부 baseline이 없다. 비교 대상 D_xR-iLQR은 사실상 동일 알고리즘의 열화 변형(self-baseline)이며, direct collocation은 국소최소 확인용으로만 1회 사용된다.

**4. 시뮬/실험.** 4개 하이브리드 예제 (모두 sim, 하드웨어 없음): (i) 1D 탄성 충돌 공—bounce 수 0/1/3 시드, Table I, Ξ-iLQR 전 케이스 수렴 vs D_xR-iLQR은 전이 발생 시 |δJ|>0.05로 미수렴; 1-bounce 비용 Ξ 42.5 vs D_xR 97.3. (ii) spring-damper 접촉—Ξ J=13.21 vs D_xR 13.29, δJ 한 자릿수 차이. (iii) 곡면 소성충돌—Ξ J=10.7(δJ=3.34e-5) vs D_xR J=50.5(δJ=18.5, 미수렴). (iv) 마찰·미끄러짐 평면 쿼드콥터 perching—Ξ J=4.76(δJ=0.170 수렴) vs D_xR J=2.66e3(δJ=3e5 미수렴). N=1000, Δ=0.001s, ode45 event detection.

**5. Contribution(delta).** 가장 가까운 선행 [4](Li-Wensing, impact-aware DDP, 2020)은 사전지정 모드 시퀀스 + reset map Jacobian + Augmented Lagrangian이었다. 본 논문의 delta: (a) backward pass에 saltation matrix 도입으로 전이 선형화를 정확화, (b) 사전지정 없이 forward pass에서 모드 추가/제거/이동 허용(standalone), (c) 모드 불일치용 reference/입력 extension. 출판년(2021) 기준 하이브리드 trajectory optimization에서 saltation 기반 정확 선형화를 iLQR에 적용한 것은 명확한 점진적 진전이나, 도구 자체는 기존 것이다.

**6. Limitation·Future work.** 저자: 2차 변분 미적용, 하드웨어 미검증, 국소성으로 인한 초기화 민감성, 교차 guard(Bouligand derivative) 미처리. 추가로 보는 한계: 형식적 수렴/최적성 보장 부재, 외부 SOTA baseline 부재, fully-controllable 가정의 비현실성, sim-only. G&C 관점에서 powered descent 터치다운·단 분리·접촉 기반 착륙 등 상태 점프가 있는 문제에 이론적으로 전이 가능하나 본 논문엔 항공우주 검증이 전무하다.

**평가:** 61/100 (fair) · 한줄평: saltation matrix를 iLQR에 결합한 견실한 하이브리드 궤적최적화 통합이나 형식적 보장·외부 baseline·하드웨어가 없는 sim-only 로보틱스 연구로, G&C 직접 관련성은 상태점프 문제로의 간접 전이에 그친다.

---

## (2022, IROS, The Uncertainty Aware Salted Kalman Filter: State Estimation for Hybrid Systems with Uncertain Guards: Payne et al.)

**1. 핵심 아이디어.** 접촉을 동반하는 하이브리드 동역학계에서, 가드(guard, 모드 전환 경계) 위치와 리셋 맵 파라미터가 불확실할 때 상태 공분산이 어떻게 전파되는지를 1차로 유도한다. 기존 Salted Kalman Filter(SKF, [8])는 가드를 정확히 안다고 가정했으나, 본 논문은 가드 노멀 방향 불확실성을 포착하는 새로운 "guard saltation matrix"와, 복원계수 등 리셋 파라미터 불확실성을 잡는 리셋 맵 야코비안을 도입한다. 이를 KF에 통합한 것이 uaSKF이며, 충격(impact) 이벤트 이후 추정 오차를 24-60% 줄인다고 보고한다.

**2. 알고리즘.** 표준 saltation matrix Ξ_x = D_x R_{I,J} + (f_J − D_x R_{I,J} f_I − D_t R_{I,J})·D_x g_I / (D_x g_I f_I + D_t g_I)에 더해, 가드 불확실성 δ_g에 대한 guard saltation matrix Ξ_g := (D_x R_{I,J} f_I + D_t R_{I,J} − f_J)/(D_x g_I f_I + D_t g_I)을 유도하여 δx⁺ = Ξ_x δx⁻ + Ξ_g δ_g 형태로 확장한다. 공분산은 Σ_x⁺ = Ξ_x Σ_x⁻ Ξ_x^T + Ξ_g Σ_g Ξ_g^T (직접 업데이트, eq.18) 또는 확장 saltation 행렬 Ξ̂(eq.17)로 갱신한다. 리셋 파라미터 θ에 대해서는 R(x,θ)를 테일러 전개하여 (D_θ R_{I,J})Σ_θ(D_θ R_{I,J})^T 항을 추가(eq.26). 전환 시 a priori/a posteriori KF 업데이트(eq.27-35)에 이 순간 업데이트(eq.32-33)를 결합한다.

**3. 비판적 검토.**
- 전 과정이 1차 선형화 기반이라 가드 곡률·노멀 방향 타이밍 등 고차항을 버린다. 저자도 명시하며 KL divergence가 0으로 수렴하지 않는 잔차(예: 402→0.03, 357.6→19.8)가 이 선형화 오차에서 온다고 인정한다. 큰 불확실성/강한 비선형 충격에서는 정확도 저하가 예상된다.
- 가드 불확실성을 "정상(stationary)" 분포로 가정하는 점이 [20]의 시변 noisy saltation과의 차별점이자 한계다. 노이즈 속도가 time-to-impact에 미치는 영향을 무시한다.
- baseline이 사실상 SKF [8] 단일 방법뿐이다. IMM/입자필터 등 다른 하이브리드 추정기와의 직접 정량 비교가 없어 우월성 주장의 폭이 제한된다.
- 모드 전환이 "발생했다고 믿은 후"의 공분산 갱신만 다루며, 전환 여부·시점 판단 문제는 future work로 남겨 닫힌 루프 추정의 핵심 난점이 미해결이다.
- transversality·non-Zeno·벡터장 확장성(extensible beyond nominal guard) 가정이 필요하며, 후자는 비물리적 영역으로의 외삽을 요구해 적용 한계를 만든다.

**4. 시뮬/실험.** 세 시스템 각 1000회 몬테카를로(sim). (1) 탄성 바운싱 볼: 높이 3m, 초기속도 −5m/s, 지면 높이 σ=0.25m·각도 σ=0.05rad, 복원계수 0.8 → MSE 중앙값 0.6% 개선, 단일 타임스텝 최대 24% 개선; KL divergence 402→0.03. (2) 미지 반경(평균 2m, σ=0.25m) 원에 소성 충돌하는 점질량 → MSE 3.6% 개선, 최대 35% 개선(p<0.005). (3) ASLIP 호퍼(비대칭 SLIP, 지면 높이 σ=0.01m, 5초·약 4홉) → MSE 54.7% 개선, 단일 타임스텝 최대 60.1% 개선(p<0.005, 부호검정). baseline은 모두 표준 SKF [8]. 실제 하드웨어 검증은 없음.

**5. Contribution(delta).** 출판년(2022) 기준, SKF [8]가 가드를 정확히 안다고 가정한 점을 넘어 가드 위치 불확실성을 1차 공분산 전파에 통합한 "guard saltation matrix"가 핵심 신규성이다. [20]의 시변 noisy saltation과 달리 가드를 추정 분포에서 추출된 정상 확률변수로 모델링하여 다른 매핑을 얻고, 추가로 리셋 파라미터(복원계수 등) 불확실성을 야코비안으로 처리한다. 즉 하이브리드 KF에서 "환경 구조 자체의 불확실성"을 다루는 새 공식화로, 기존 대비 명확한 진전이다.

**6. Limitation·Future work.** 저자: 1차 선형화 잔차, 모드 전환 발생 여부 판단 미해결(IMM 확장 제안), 단일/순차 전환만 처리(동시 접촉은 Bouligand 미분으로 확장 제안). 추가로 내가 보는 한계: 실하드웨어·실시간 타이밍 분석 부재, baseline 단일성, 정상 가드 분포 가정, 외삽 가능성 가정. 전반적으로 이론·시뮬은 견고하나 도메인이 다리형 로봇 보행/접촉 추정이며 항공우주 G&C로의 직접 연관성은 간접적이다.

**평가:** 76/100 (strong) · 한줄평: 하이브리드 시스템의 가드·리셋 불확실성 공분산 전파를 깔끔히 유도한 견고한 추정이론 기여이나, 검증이 다리형 로봇 위주이고 G&C 활용은 간접적이다.

---

## (2022, NAHS, Trajectory sensitivity analysis of hybrid systems with sliding motion: Pytlak·Suski)

**1. 핵심 아이디어.** 슬라이딩 운동(sliding motion)을 갖는 하이브리드 제어 시스템의 궤적 민감도(trajectory sensitivity)를 해석한다. 슬라이딩 표면 위의 운동을 index-2 미분-대수방정식(DAE)으로 모델링하여 스위칭 다양체를 정확히 추적하고, 이에 대응하는 선형화 방정식(linearized equations)의 존재·유일성과 성질을 규명한다. 핵심 기여는 슬라이딩 모드를 갖는 하이브리드 시스템에 대해 (이전에 다루어지지 않았던) 민감도 방정식을 유도하고, 스위칭 시각에서 민감도 해가 불연속임을 보이며, 모든 허용 제어/섭동에 대해 성립하는 전역(global) 섭동 추정을 확립한 것이다.

**2. 알고리즘.** 슬라이딩 운동은 Filippov 해 x' ∈ co(f1,f2)로 표현되고, h_x(x)f_F(x,u)=0 조건에서 볼록계수 α(x,u)=h_x f1 / (h_x(f1−f2))를 얻는다. 스위칭 표면 추적은 ODE 대신 DAE x'=f_F(x,u)+h_x(x)^T z, 0=h(x) (보조 대수변수 z, 슬라이딩 중 z≡0)로 적분한다. 슬라이딩 모드를 세 번째 이산상태 q=3으로 처리한다. 선형화 방정식 (27)-(28)은 index-2 DAE이며, 가정 (H1)(H2)(H3) 하에서 (29)-(32)의 근사 성질이 성립함을 Theorem 4.1(스위칭시 dt_t^d 변분), Theorem 5.2(전역 추정), Theorem 5.3으로 증명한다. 스위칭 시각에서 y^{x,d}의 점프 공식 (127),(129)을 유도.

**3. 비판적 검토.**
- 순수 해석 논문으로 수치 결과가 전혀 없다. 저자 스스로 "구현 세부가 결과 품질에 큰 영향을 주므로 동반 논문 [14][16]에서 다룬다"고 명시 — 본 논문만으로는 이론의 실효성(추정상수 c1,c2,c3 등이 실제로 유용한 크기인지)을 판단할 수 없다.
- 핵심 기계장치(DAE 민감도, 음함수 정리 적용, ODE 등가성)를 [27][29][30][32]에서 상당 부분 수입한다. 본 논문의 신규 증명은 슬라이딩 모드로의 확장 부분에 집중되어 있어, 완전한 자기완결 증명은 아니다.
- 슬라이딩 표면을 co-dimension 1, 교차 없음, 상태 점프 없음, 이산상태 3개로 제한 — 가정이 강하다. 다중 슬라이딩 표면 교차([23]-[26])는 명시적으로 배제.
- 가정 (H2)(iii),(iv), (H3)의 스위칭점 분리 조건 등이 다소 기술적이고 검증 부담이 큼. 실제 시스템에서 이 가정들이 성립하는지에 대한 예시가 없다.

**4. 시뮬/실험.** 없음. 수치 시뮬레이션·실험·HIL·baseline 비교가 전무하다. 논문은 정성적 검증 다이어그램(Fig. 1, 민감도 관계의 유효 구간 요약)만 포함. 정량 결과 없음.

**5. Contribution(delta).** 출판년(2022) 기준으로, 하이브리드 시스템 최적제어/민감도 문헌([3]-[12])에서 슬라이딩 모드를 갖는 경우의 궤적 민감도 해석은 다루어진 적이 없었다는 공백을 메운다. 선행 [13][27]의 고차 index DAE 최적제어/민감도를 슬라이딩 표면(transition guard 위 운동)으로 확장하고, 스위칭 시각에서 선형화 해의 불연속성(점프 공식)과 전역 섭동 추정을 새로 확립한 점이 delta. 이는 동반 논문 [14]의 약형 최대원리 및 알고리즘 전역 수렴성 증명의 이론적 토대를 제공한다.

**6. Limitation·Future work.** (저자) 수치 검증은 동반 논문으로 미룸; co-dim 1·교차 없는 단일 슬라이딩 표면·상태 점프 없음으로 제한. (필자) 가정의 검증 가능성·추정상수의 실용적 타이트함이 미확인이고, 응용 예제(특히 항공우주 G&C 시나리오)가 없어 실효성·영향력이 간접적이다. 스위칭 시각 자체가 결정변수인 경우([17]-[19])는 미고려.

**평가:** 59/100 (fair) · 한줄평: 슬라이딩 모드 하이브리드 시스템 민감도라는 공백을 엄밀히 메운 견실한 제어이론 논문이나, 수치 검증 부재와 강한 가정·낮은 인용속도(0.5/yr)로 항공우주 G&C에 대한 관련성은 간접적(near-domain).

---

## (2024, Proc. IEEE, Saltation Matrices: The Essential Tool for Linearizing Hybrid Dynamical Systems: Kong et al.)

**1. 핵심 아이디어.** 연속·이산 상태가 공존하는 하이브리드 동역학계에서 모드 전환(impact/reset) 순간에 perturbation이 어떻게 1차로 사상되는지를 기술하는 saltation matrix(점프 행렬, Xi)를 통합적으로 다루는 tutorial/survey 논문이다. 저자들은 reset map의 Jacobian D_xR만으로는 전환 타이밍 차이가 만드는 변분(variation)을 포착하지 못하며, saltation matrix가 reset 적용 시점 차이와 양 모드 동역학의 차이를 합쳐 정확한 1차 민감도를 준다는 점을 강조한다. 강체-접촉계의 단일 제약 전환에 대해 saltation matrix를 일관되게 유도·분류하고, EKF·iLQR·LQR·공분산 전파 등 기존 평활(smooth) 알고리즘으로의 자연스러운 확장 경로를 제시한다.

**2. 알고리즘.** 핵심 정의(식 9): Xi_(I,J) = D_xR^- + [(F_J^+ - D_xR^- F_I^- - D_tR^-) D_xg^-] / (D_tg^- + D_xg^- F_I^-). 변분 갱신은 dx(t^+) = Xi_(I,J) dx(t^-) + h.o.t.(식 10). 분모는 횡단성(transversality) 가정 D_tg^- + D_xg^- F_I^- != 0 을 요구한다(식 11). 주기계의 안정성은 monodromy matrix Phi = Xi_(Z,I)A_(Z,T_Z)...Xi_(I,J)A_(I,T_I)(식 34)와 Floquet 승수로 판정한다. 공분산은 Sigma(t^+) = Xi Sigma(t^-) Xi^T(식 37), co-vector(LQR value)는 P(t^-) = Xi^T P(t^+) Xi(식 40)로 전파된다. 강체-접촉계는 unconstrained/constrained/sliding/sticking 모드와 8개 전환에 대해 Xi를 닫힌형으로 유도(식 70~94, Table II)한다.

**3. 비판적 검토.**
- 본질적으로 tutorial/survey이며 saltation matrix 자체는 Aizerman-Gantmakher(1958)까지 거슬러 올라가는 고전적 도구다. 새로움은 방법이 아니라 통합 정리와 부록의 신규 유도(공분산·Riccati 갱신, early-impact 케이스, 강체 Xi의 고유구조 분석)에 한정된다.
- 검증이 예시(ball-drop) 수준에 머문다. MC·정량 외부 baseline·통계 처리가 없으며, "D_xR 예측은 틀리고 Xi는 맞다"는 개념적 대조가 사실상 전부다(Fig 1,5,6).
- 가정의 한계(grazing impact, 동시 접촉 시 B-derivative 필요, Zeno 배제)를 명확히 적시한 점은 긍정적이나, 이 경우들에 대한 해는 제시하지 않고 타 도구로 위임한다.
- 실시간성 주장(Salted Kalman Filter가 hybrid particle filter 대비 적은 연산으로 동등 정확도, 온라인 hybrid iLQR-MPC)은 인용에 의존하며 본 논문 자체의 타이밍·복잡도 분석은 없다.
- 항공우주 G&C로의 직접 적용 사례는 없다. 다리형 로봇·전력회로·신경모델이 home field이며, 착륙/접촉·단분리 등 G&C의 하이브리드 문제로의 전이는 간접적이다.

**4. 시뮬/실험.** 실측·대규모 시뮬은 없다. 경사면에 떨어지는 2D 점질량(ball-drop)의 마찰 없는 sliding과 sticking 두 케이스에 대해 Xi를 닫힌형으로 계산하고, 초기 분포의 공분산이 제약 방향으로 정확히 붕괴됨을 시각적으로 제시(Fig 1,6 (sim)). 고유값 분석으로 sliding은 lambda={0,1} with v0=[sin th, cos th]^T, sticking은 v0=[q1_dot,q2_dot]^T로 변분 소거 방향을 해석(식 57,58). 정량적 baseline은 "reset Jacobian D_xR 예측"과의 개념적 대조뿐이며 MC·통계·외부 알고리즘 비교는 없다.

**5. Contribution(delta).** 출판년(2024) 기준, 여러 분야(로보틱스·전력회로·신경과학)에 흩어져 있던 saltation matrix 사용·유도를 Proc. IEEE 수준의 단일 참고문헌으로 통합한 것이 1차 기여다. 신규 유도로는 (i) chain-rule 기반 대체 유도(App A), (ii) early-impact 케이스(App B), (iii) 하이브리드 전환에서의 공분산 전파(App C)와 Riccati/co-vector 갱신(App D)의 명시적 증명, (iv) 단일 제약 강체-접촉계의 8개 전환에 대한 Xi의 통일·확장 유도와 공통 구조(우상단 블록=0, identity reset 조건, 블록 고유구조) 규명(Sec V, Table II)을 제시한다. saltation matrix 개념 자체의 originality는 선행에 귀속되므로 novelty는 통합+모듈식 유도의 delta에 한정된다.

**6. Limitation·Future work.** 저자가 명시한 한계: grazing impact(접선 충돌), 동시 다중 접촉(guard 교차)에서는 Xi가 부적용이며 B-derivative가 필요; 횡단성·Zeno 가정 의존; 모든 인접 궤적이 동일 전환 순서를 거친다는 가정. 내가 보는 한계: 자체 정량 검증·실시간 벤치마크 부재로 deployment claim이 인용 의존적; 항공우주 G&C 직접 사례 부재; 비평활 마찰(stick-slip) Xi는 마찰 모델 정확도에 민감하다고 저자도 인정. 향후 비전문가용 hybrid EKF/iLQR 통합 라이브러리화와 다중 접촉으로의 확장이 자연스러운 방향이다.

**평가:** 68/100 (good) · 한줄평: 하이브리드계 선형화의 핵심 도구를 일관되게 통합한 견고한 tutorial이나 자체 정량 검증이 예시 수준이며, G&C(착륙·접촉·단분리)로의 전이 가치는 높되 직접 적용은 미입증인 near-domain 제어이론 문헌.

---

## (2024, IROS, Feasibility-Guided Safety-Aware Model Predictive Control for Jump Markov Linear Systems: Laouar et al.)

**1. 핵심 아이디어.** Jump Markov Linear System(JMLS)에서 모드 전이가 확률적이고 모드 추정이 부정확(검출 지연 포함)한 상황에 대해, CBF 제약을 가진 MPC를 설계한다. 핵심은 모든 모드에 대해 제어입력을 동일하게 강제하는 길이 h의 "consensus horizon" 개념이며, h가 클수록 모드 불확실성에 강건하지만 OCP가 비실현(infeasible)될 수 있다는 강건성-실현가능성 트레이드오프를 다룬다. 매 스텝마다 실현가능성을 유지하는 최대 consensus horizon을 이진탐색으로 적응적으로 선택한다.

**2. 알고리즘.** 이산시간 JMLS x_{k+1}=A_i x_k + B_i u_k, 모드 i(t)~µ_t, µ_{t+1}=Ω µ_t. 안전집합은 CBF β로 정의되며 Δβ(x,u)=β(x_{k+1})-β(x_k) ≥ -γ β(x_k), 즉 β(x_{k+1}) ≥ (1-γ)β(x_k). 모드별 안전집합 S^i_{t+k}를 각 모드 i에 대해 강제하고, consensus 제약 u^i_k = u^j_k (∀ i,j, ∀ k∈{0,...,h})를 추가한 QP를 푼다. 정리1: h1<h2이면 F_{h1} ⊇ F_{h2} (consensus horizon이 클수록 실현가능 집합이 줄어듦, 단조). 이 단조성을 이용해 Alg.1에서 목적함수를 상수로 치환한 LP feasibility check를 이진탐색(⌈log2 H⌉회)으로 수행하여 최대 실현가능 consensus horizon h*를 구한 뒤 h=h*로 (9)를 푼다. Stage cost는 ||x-x_r||_Q^2 + ||u||_R^2를 모드확률 µ로 가중.

**3. 비판적 검토.**
- Greedy(매 스텝 최대 h*) 설계는 저자도 인정하듯 receding-horizon 폐루프 재귀실현가능성(recursive feasibility)을 보장하지 못한다. 정리2는 낮은 h가 안전하지 않을 수 있음을 보일 뿐, 제안 방식의 폐루프 안전 보장은 없고 "실현가능성 최대화"라는 휴리스틱에 머문다.
- CBF/이산 CBF 기계는 모두 인용(refs [23][24])이며 본 논문 고유 증명은 단조성(정리1)과 정리2 sketch 수준으로 비교적 가볍다. 정리2는 proof sketch에 그친다.
- 안전 보장이 "모드별 CBF"에 의존하나 추정 µ̂가 틀릴 때의 형식적 안전 인증서(예: chance constraint, worst-case 집합 불변성)는 제시되지 않는다. consensus는 강건성을 높이지만 정량적 안전확률을 보증하지 않는다.
- baseline(First-Step, Full-Step, Non-Robust)이 모두 자체 ablation이며 외부 SOTA(예: scenario MPC, robust tube-MPC, POMDP 근사)와의 비교가 없다. 성공률 100%는 인상적이나 비교군이 자기 변형이라 우위 해석에 한계.

**4. 시뮬/실험.** 두 케이스 (sim). (1) 6-DOF 우주선 궤도 랑데부(Clohessy-Wiltshire-Hill, 2모드, 평균운동 n=0.061, n_p 0.041~0.101), 336 trial: 성공률 Ours 100% vs First-Step 56.0% / Full-Step 68.4% / Non-Robust 19.3%, 평균비용 Ours 1.95(최저). (2) 12차원 헥사콥터 광산갱도 점검(3모드: 정상+로터고장 2종), 108 trial: 성공률 Ours 100% vs 37.0/19.4/53.7%, 평균비용 3.92(최저). 모드전환 시각·검출지연을 범위로 변화. 계산률(Hz): Ours 9.89Hz(랑데부)/11.21Hz(갱도) — baseline보다 느리지만 운용 가능 수준. Julia + Clarabel.jl, 3.7GHz 단일스레드.

**5. Contribution(delta).** 출판년(2024) 기준 기여: (i) CBF 제약을 가진 JMLS MPC에 대한 consensus horizon 개념 정식화, (ii) consensus horizon이 실현가능성·안전에 미치는 영향 분석(정리1·2), (iii) 매 스텝 실현가능성을 유지하며 강건성을 최대화하는 최대 실현가능 consensus horizon을 LP 이진탐색으로 구하는 실용적 방법, (iv) 두 로보틱스/우주선 사례 벤치마크. 기존 CBF-MPC 실현가능성 향상 연구([17]-[20]) 및 고정 consensus horizon scenario-MPC([25]) 대비, JMLS+모드불확실성 하에서 horizon을 적응적으로 선택한다는 점이 명확한 델타다. 다만 근본적으로 기존 도구(CBF, IMM, scenario MPC)의 조합·확장 성격이 강하다.

**6. Limitation·Future work.** 저자: 비선형 안전집합/동역학 노이즈로의 CBF 확장, h* 계산 부하(온보드 고주파 제약), 하드웨어 검증 미수행. 추가로 본인 관점: 폐루프 재귀실현가능성·정량적 안전확률의 형식 보장 부재, 외부 SOTA baseline 비교 부재, 모드 전이행렬 Ω 부정확성에 대한 강건성이 정량화되지 않음(범위 sweep으로만 제시), G&C 적용은 랑데부 1개 사례로 항공우주 도메인 실증은 제한적.

**평가:** 70/100 (good) · 한줄평: JMLS의 강건성-실현가능성 트레이드오프를 consensus horizon으로 잘 정식화한 견고한 control-theory 논문이나, 형식적 안전 보장과 외부 baseline이 약하고 G&C 관련성은 랑데부 사례를 통한 간접적 수준이다.

---

## (2024, IROS, Robustifying Model-Based Locomotion by Zero-order Stochastic Nonlinear Model Predictive Control with Guard Saltation Matrix: Katayama et al.)

**1. 핵심 아이디어.** 레그드 로봇 보행에서 접촉(contact) 불확실성, 특히 지형 높이 추정 오차로 인한 접촉 시점/위치의 불확실성을 stochastic/robust NMPC에 통합하는 방법을 제안한다. 핵심은 하이브리드 시스템의 guard saltation matrix를 이용해 접촉 이벤트(모드 전환)를 지나는 상태 공분산을 전파하고, EKF 유사의 a priori/a posteriori 공분산 업데이트를 결합한 점이다. 이를 zero-order stochastic NMPC와 결합하여 실시간성을 확보하고, 6족 wheeled-legged 로봇(Tachyon 3)에서 온보드 하드웨어 실험까지 수행했다.

**2. 알고리즘.** (i) 노드 간 닫힌루프 공분산 전파 P_{i+1}=Φ(P_i,K_i)=(A_i+B_iK_i)P_i(...)^T+Γ_iW_iΓ_i^T. (ii) 접촉 이벤트(점프 노드)에서 saltation matrix Ξ_x, Ξ_g (식 4,5)를 이용한 a priori 업데이트: P̂_{j+1}=Ξ_x P_j Ξ_x^T+Ξ_g C_g Ξ_g^T (식 10). 다차원 guard(동시 다수 접촉)에는 1차원 업데이트의 합으로 근사 (식 11, transversality 조건과 합산 인덱스 생략). (iii) guard condition을 출력방정식으로 보고 EKF식 a posteriori 업데이트 L_{j+1}, P_{j+1}=(I-L_{j+1}∇g)P̂ (식 12,13)로 공분산 과대성장 억제. (iv) chance constraint를 backoff term β_i=γ_i sqrt(∇h^T[I;K]P[I;K]^T∇h)로 결정론적 제약화 (식 15-17), γ_i=Ψ^{-1}(p_i). (v) zero-order 알고리즘으로 공분산/backoff 민감도를 무시하고 Riccati recursion에서 피드백 게인 K를 부산물로 획득, primal-dual interior point로 실시간 풀이.

**3. 비판적 검토.**
- 다차원 guard 공분산 업데이트(식 11)는 1차원 saltation 업데이트의 단순 합으로, 저자 스스로 transversality 조건 미충족 및 이론적 근거 부재(Bouligand derivative 기반은 future work)를 인정한다. 실험적으로만 정당화된 핵심 근사다.
- 안정성/제약 만족에 대한 형식적 보장이 없다. chance constraint는 Gaussian 가정+선형화 기반 근사이며, zero-order는 공분산 민감도를 의도적으로 버려 최적성/보수성 trade-off가 정량 분석되지 않는다.
- W_i(process noise covariance)를 실제 노이즈가 아닌 수렴성/성능을 위해 경험적으로 튜닝해야 한다고 명시(식 19, σ를 두 로봇에서 10^-6/10^-8로 상이하게 설정)—물리적 의미와 분리된 자유 파라미터로, 재현·일반화에 부담.
- 비교 baseline(SMPC, HMPC, MPC)이 대부분 제안 프레임워크의 내부 ablation/변형이며, SMPC만 선행연구[8] 적응. 외부 SOTA와의 직접 비교는 제한적이다.
- 휴머노이드 rough terrain에서는 여전히 낙상을 완전히 막지 못함(저자 인정)—접촉 re-planning 부재가 한계.

**4. 시뮬/실험.** 2개 플랫폼: Tachyon 3(6족 wheeled-legged, 4단 계단, ±4cm 높이추정 오차), EVAL-03(소형 휴머노이드, rough terrain). 각 시나리오 20개 randomized 환경에서 4기법(GS-SMPC/SMPC/HMPC/MPC) 성공률 비교(sim); GS-SMPC가 양 시나리오 최고. 공분산 예측 비교(식 11-13 vs 11 vs 20 dynamics-only)로 motion-specific 예측 우수성 정성 검증. HW: Tachyon 3 온보드 PC(i7-8850H)에서 2단 계단 승하강 perceptive locomotion 성공(real), GS-SMPC 평균 29ms vs nominal NMPC 26ms(약 +3ms)로 실시간성 입증. MC급 통계는 success rate 외 부재.

**5. Contribution(delta).** 출판년(2024) 기준, guard saltation matrix를 오프라인 궤적최적화/상태추정(Payne[2], offline traj opt)이 아니라 온라인 stochastic NMPC의 공분산 전파에 통합한 최초 사례로 볼 수 있다. Gazar[8](연속시간 dynamics만), Xu[7](linear robust)와 달리 접촉 이벤트의 불연속/state jump 불확실성을 명시적으로 다루는 것이 핵심 delta. zero-order+interior point+Riccati로 피드백 게인을 부산물로 얻어 nominal NMPC 대비 거의 무비용 온보드 구현을 보인 점도 실용적 기여.

**6. Limitation·Future work.** 저자: 다차원 guard 업데이트의 이론적 근거(Bouligand derivative) 확보, 접촉 re-planning(emergent contact)으로의 확장, 휴머노이드 낙상 방지. 추가로 (필자): 형식적 robustness 보장 부재, W_i 경험튜닝 의존, baseline의 내부변형 편중, Gaussian/선형화 근사의 비선형 접촉동역학 적합성 미검증, 코드 비공개로 재현성 제약.

**평가:** 73/100 (good) · 한줄평: saltation matrix 기반 접촉 불확실성 공분산 전파를 실시간 NMPC에 녹여 온보드 실증까지 도달한 견실한 robotics 제어 논문이나, 다차원 근사의 이론 공백과 비항공 도메인으로 G&C 직접 관련성은 간접적(하이브리드/착륙 레그 접촉 이벤트로의 전이 가능성 수준).

---

## (2024, arXiv, Constrained Trajectory Optimization for Hybrid Dynamical Systems: Crestaz et al.)

**1. 핵심 아이디어.** 최근 제안된 Hybrid iLQR(HiLQR, saltation matrix 기반 간접 최적화)을 상태/입력 제약 처리가 가능하도록 확장한다. 두 가지 제약 처리 기법 — Discrete Barrier State(DBaS, 내부점 방식)와 Augmented Lagrangian(AL, 페널티 방식) — 을 하이브리드 설정에 통합하여 장애물 회피, 상태 경계, 작동기 한계를 다룬다. 직접법(NLP) 대비 간접법의 계산 효율성과 반복 과정 전체에서의 동역학적 실현가능성 유지를 노린다.

**2. 알고리즘.** 하이브리드 천이는 saltation matrix Ξ = D_xR + (F_J − D_xR F_I − D_tR)D_xg/(D_tg + D_xgF_I)로 선형화하여 δx(t+)=Ξ δx(t−)로 매핑. HiLQR backward pass의 Q-항(Q_x,Q_u,Q_xx,Q_uu,Q_xu)에 Ξ를 삽입. DBaS: 상태를 x̃=[x; w]로 증강, barrier state w_k=B(g(x_k))−B(g(x_goal)), B(g)=−1/g (역장벽), saltation을 통한 barrier 동역학 ẇ = (1/g²)g_x ẋ로 천이까지 전파; 제약 위반 시 무한 비용으로 line-search가 자동 폐기. AL: 목적함수에 (λ_k + ½ g_k I_μ)^T g_k 페널티 추가, λ_i=max(0, λ_i+μ_i g_i(x*)), μ=φμ로 갱신. 제어법칙은 표준 δu*=−Q_uu^{-1}(Q_ux δx + Q_u)=Kδx+d.

**3. 비판적 검토.**
- 핵심 수학(saltation matrix [5], DBaS [11], AL/ALTRO [12])이 모두 import이며, 자체 기여는 barrier state를 saltation으로 전파하는 식(13)-(14) 정도. 수렴/실현가능성에 대한 자체 정리·증명은 없음.
- 외부 baseline 부재가 가장 큰 약점. 직접법(ALTRO/IPOPT/SNOPT) 또는 기존 제약 DDP와의 비교 없이 자체 두 변형(DBaS vs AL)만 대조하므로 "간접법이 더 효율적"이라는 주장이 정량적으로 뒷받침되지 않음.
- 검증이 단일 시험대(2D bouncing ball, 4-상태)에 국한. 동기로 든 사족보행/공중조작 같은 고차원 하이브리드계로의 확장성은 미검증이며 저자도 인정.
- 계산 효율을 핵심 동기로 내세우지만 timing/복잡도 수치가 전혀 없음. 실시간성은 "open issue"로 명시.

**4. 시뮬/실험.** 2D 튀는 공(질량 1kg, g=9.81, e=0.75, N=200, Δt=0.02s), 장애물 1~10개 각 100시나리오(총 1000+)로 성공률·수렴 반복수 통계. Feasible start 평균 성공률: AL 87.4% vs DBaS 84.1%; Infeasible start: AL 69.4% vs DBaS 61.9%(AL이 −20.6%, DBaS −26.5% 하락). 장애물 ≤6에서 양쪽 >80%, 장애물 증가 시 AL이 우세하나 반복수는 DBaS가 적음. DBaS는 보수적·실현가능 유지, AL은 일시적 위반 허용으로 좁은 공간 통과·천이 중 재계획 가능. 모두 (sim), 실하드웨어·HIL·외부 baseline 없음.

**5. Contribution(delta).** 선행 HiLQR(Kong et al. [1],[2])이 무제약이었던 것을 외부 상태/입력 제약 처리로 확장. DBaS·AL 두 메커니즘을 하이브리드 간접 최적화에 통합하고, infeasible start에 대한 수렴 거동을 비교한 점이 delta. 출판 시점(2024/25) 기준 SOTA 대비 각 구성요소는 기존재하며, 신규성은 "saltation 인지 barrier state 동역학 + 하이브리드계 통합 + 비교 연구"에 있어 점진적~중간 수준. 코드 오픈소스화.

**6. Limitation·Future work.** 저자: 실하드웨어 검증·실시간성·확장성 미해결; learning 기반 제약 처리 통합을 향후 과제로 제시. 내가 보는 한계: 외부 baseline 부재, 단일 저차원 시험대, DBaS의 수치 불안정(safe set 밖에서 23.4% 실패), 형식적 보장(실현가능성/수렴) 부재, 불확실성/외란 모델링 없음.

**평가:** 64/100 (fair) · 한줄평: 제약 하이브리드 간접 최적화의 깔끔한 통합·재현가능한 비교 연구지만 외부 baseline·실시간 분석·이론적 보장이 빠진 점진적 기여이며, 도메인은 로보틱스(튀는 공/사족)로 G&C(하이브리드 착륙 접촉 동역학)로의 전이 가능성은 간접적.

---

## (2025, AIAA SciTech, Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-time Compound State-Triggered Constraints: Uzun et al.)

**1. 핵심 아이디어.** 6-DoF 동력하강유도(PDG)에서 논리적 명세(state-triggered constraint, STC)의 복합(AND/OR) 형태를 연속시간으로 만족시키는 SCP 프레임워크를 제안한다. 핵심은 STL의 매끄러운 강건성 척도인 D-GMSR(generalized mean-based smooth robustness, sound·complete·C1-smooth)로 복합 STC를 매개화하고, 이를 연속시간 successive convexification(CT-SCvx) 위에서 푸는 것이다. 그 결과 노드 사이 구간에서도 제약 만족이 보장되고 수렴성도 유지된다.

**2. 알고리즘.** 상태 x=(m,r_I,v_I,q,ω_B), 입력 u=(T,δ^e,φ^e,δ^b,φ^b)로 짐벌 엔진 6-DoF 동역학을 구성. STC는 고도/속도/틸트 트리거에 따른 추력·짐벌·LOS·glideslope 제약을 함의(⇒)·논리곱(∧)·논리합(∨)으로 표현. D-GMSR 키함수 ∧h, ∨h(가중 기하/멱평균)로 명세를 매개화하면, 만족 조건이 Σmax(0,y_i)^2=0 형태의 비음(non-negative) 등식으로 단순화되어 외부 벌점함수 없이 연속시간 제약으로 편입(식 2). 이후 CT-SCvx: (i) 경로제약을 적분상태 y로 reformulation해 연속시간 만족 보장(식 4c), (ii) time-dilation으로 자유종단시간→고정종단시간, (iii) multiple-shooting 정확 이산화, (iv) 비볼록 제약의 exact L1 penalty, (v) 수렴보장 prox-linear SCP(식 10·11) + adaptive weight update(Alg.1). 솔버는 CVXPY+ECOS/MOSEK.

**3. 비판적 검토.**
- 이론적 신규성의 상당 부분(D-GMSR의 soundness·completeness, CT-SCvx의 연속시간 만족·수렴 보장, prox-linear 정상점 보장)이 선행연구 [75][55][33]에서 가져온 것이다. 본 논문 고유의 정리/증명은 없고, 기존 두 기법의 결합과 PDG로의 적용이 기여의 핵심이다.
- 검증이 단일 시나리오 1회 수치 시뮬에 그친다. Monte Carlo, 외부 baseline(예: min/max smoothing 기반 STL-MIP, 또는 노드점-only STC 부과 대비), 통계 처리가 전무하여 "robust and efficient" 주장이 정량적으로 뒷받침되지 않는다.
- "real-time implementable"을 반복 주장하나 실제 solve 시간·반복수·복잡도 수치가 보고되지 않았다(PIPG 사용 가능성만 언급). 온보드 적합성 주장은 미검증.
- 불확실성·외란 처리는 없다. 연속시간 제약 만족이 이산화 오차에 대한 강건성을 주지만, 공력/질량/추정 불확실성에 대한 강건성 보장은 범위 밖이다.
- 복합 STC의 연속시간 만족이 실제로 보장됨을 보이는 정량 근거가 본문 서술(노드 사이 즉시 만족)과 그림 참조에 의존하며, 위반 마진/오차 정량지표가 제시되지 않았다.

**4. 시뮬/실험.** 6-DoF 로켓 동력하강 단일 시나리오 (sim). K=15 노드, m_i=100t→m_dry=85t, r_i=(200,200,500)m, v_i=(0,0,-50)m/s, t_f 초기추정 21s. 결과: ~5.72s에 속도·틸트가 임계 아래로 떨어져 추력제약 즉시 만족, ~11.07s에 고도<200m로 LOS 제약 트리거, ~15s에 고도<100m로 짐벌·속도·각속도·틸트·glideslope 제약 만족. 모든 조건이 노드 사이에서 연속시간 만족됨을 보고. Monte Carlo·외부 baseline·정량 비교·timing 없음. 코드 공개(github.com/UW-ACL/CT-cSTC).

**5. Contribution(delta).** 선행 STC 연구 [37-39]는 단일 트리거 제약을, 정수제약 연구 [40,41]는 별도 처리를 다뤘고, STL 명세는 통상 min/max→MIP로 귀결되어 SCP에 부적합했다. 본 논문은 (a) 복합(논리곱·논리합 결합) STC를 D-GMSR로 sound·complete하게 매개화하여 매끄러운 SCP에 편입하고, (b) 이를 CT-SCvx의 연속시간 만족·수렴보장 위에서 PDG에 적용한 첫 사례라는 점이 출판년(2025) 기준 delta다. min/max의 locality·masking 문제를 회피하면서 명세 만족의 if-and-only-if 보장을 SCP 내에서 확보한 것이 차별점.

**6. Limitation·Future work.** 저자: eventually·until 같은 시간적 명세의 연속시간 정식화 부재, 명세 구조를 활용한 전용 최적화 알고리즘 미설계. 추가로 (내 관점): 단일 시뮬·무 baseline·무 MC로 실증 빈약, 실시간/온보드 timing 미보고로 배치 주장 미검증, 불확실성 강건성 범위 밖, 고유 이론 기여보다 기성 기법 결합 성격이 강함. SPLICE 계열 비행검증 프레임워크(CT-SCvx) 위에 서 있어 잠재적 G&C 파급은 있으나, 현 시점 영향은 아직 미성숙.

**평가:** 67/100 (good) · 한줄평: 비행관련 PDG 프레임워크에 복합 논리명세를 연속시간·수렴보장으로 편입한 깔끔한 core-G&C 방법론이나, 단일 시뮬·무 baseline·미보고 timing으로 실증과 실시간 주장이 약하고 이론 기여 다수가 import에 의존.

---

## (2026, arXiv, Path Integral Particle Filtering for Hybrid Systems via Saltation Matrices: Shaji et al.)

**1. 핵심 아이디어.** 간헐적 접촉(impact)을 겪는 하이브리드 시스템(행성 탐사 로봇, 도킹 위성 등)의 상태추정을 위해, 평활화-최적제어 쌍대성(path integral filtering)을 하이브리드 영역으로 확장한 입자필터 SPIPF를 제안한다. 모드 전이 시점의 불연속 불확실성 전파를 saltation matrix(리셋맵의 1차 근사)로 매핑하고, 비가우시안·이상치에 강건한 model-agnostic 추정을 목표로 한다. 핵심은 접촉 이벤트에서의 공분산/감도 전파를 자코비안만으로 일관되게 처리하는 데 있다.

**2. 알고리즘.** 평활화 문제를 KL 발산 최소화 변분 최적제어로 변환: min E[∫ l(t,X,u)dt + Ψ_T(X)] + KL(π0||ν0), l = g + (1/2ε)||u||². 모드별로 Kallianpur-Striebel/Girsanov를 적용해 가중치 w_j^k = (dν0/dπ0)·exp[-S_u(t_j^+, t_{j+1}^-)] 도출. 리셋맵 R의 1차 근사로 Ξ_jk ≜ ∂_x R + (F_k - ∂_x R·F_j - ∂_t R·∂_x g)/(∂_t g + ∂_x g·F_j), δX^+ ≈ Ξ_jk δX^-. 슬라이딩 윈도우 [t_j^+-H, t] 위에서 salted iLQR로 피드백/피드포워드 게인을 풀고, 입자별 롤아웃 후 가드 hit 시 R로 모드 전이. 모드 불일치는 reference extension으로, 모드 추정은 입자 가중치 voting logic으로 처리.

**3. 비판적 검토.**
- 비가우시안·이상치 강건성을 핵심 셀링 포인트로 내세우지만 실험은 전부 가우시안 노이즈(SKF가 closed-form 참 사후분포를 주는 경우)에서만 수행되어, 중심 주장(비가우시안)이 검증 없이 단언으로 남는다.
- 도출은 [21][23][35-38]의 기존 기계장치(쌍대성, saltation, 하이브리드 path-integral 제어)를 상당 부분 수입·조합한 것으로, 하이브리드 필터 자체의 일관성/수렴에 대한 형식적 정리는 제시되지 않는다. t_j^- ≈ t_{i+1} 같은 근사는 "충분히 작은 timestep"이라는 비형식적 근거에 의존한다.
- 입자마다 윈도우마다 iLQR을 푸는 구조는 연산 부담이 크다. SIR 대비 효율성을 주장하나 실제 실행시간·복잡도 수치가 전혀 없어 온보드/실시간 적합성 판단이 불가하다.
- SLIP 결과에서 MSE>50 시뮬레이션을 임계로 잘라낸 채 비교하는데(불안정 trial 제외), 이 thresholding이 baseline 대비 우위 해석을 왜곡할 여지가 있고 trial 수 보정이 불투명하다.
- 모드 mismatch 히스토그램에 대해 저자 스스로 "해석이 유용하지 않다"고 인정해, 모드추정 정확도의 정량적 근거가 약하다.

**4. 시뮬/실험.** MATLAB(Parallel Computing toolbox), 2개 하이브리드 시스템: 1-D bouncing ball(선형, e 반발계수)과 SLIP(비선형, non-identity 리셋맵). 각 50 trial MC, 무작위 초기조건·프로세스노이즈·측정. 지표: Mean MSE(식52), MSE 공분산, 정규화 ESSE. K(입자수), H(윈도우), dt 스윕. Baseline: SIR multi-mode PF(외부 기법) + SPIPF-0(0-control ablation), 참값은 SKF 사후분포. 결과: SPIPF가 전 K에서 전이 후 baseline을 크게 능가(K=10 SPIPF가 K=50 baseline보다 우수), SLIP에서 전·후 모두 더 낮은 MSE/공분산과 더 높은 ESSE. 실HW·항공우주 시나리오·비가우시안 케이스는 없음(전부 sim, Gaussian).

**5. Contribution(delta).** 출판년(2026) 기준, 연속시간 path-integral 입자필터(PIPF, [23])를 saltation matrix 기반 하이브리드/접촉 추정으로 확장한 최초 사례. SKF[36]는 가우시안 전용, PIPF[23]는 연속시간 전용이라는 한계를 동시에 메우며, reference extension으로 모드 간 상태차원 불일치를 처리하고 voting logic으로 모드추정을 통합. 다만 본질적으로 최근 기존 도구들의 조합으로, 새 방향을 여는 근본적 신규성보다는 closest prior art 대비 명확한 확장에 해당.

**6. Limitation·Future work.** (저자) 멀티스레딩·adaptive resampling, 학습기반 system ID와의 결합, 하드웨어 적용. (추가) (i) 핵심 비가우시안 강건성 주장이 미검증, (ii) 실행시간·실시간성 분석 부재로 온보드 적용성 불명, (iii) 형식적 추정 일관성/수렴 보장 부재, (iv) 슬라이딩 윈도우 크기 증가가 SLIP에서 오히려 성능 저하(가드검출 민감도)되는 등 안정성 의존성, (v) 항공우주 실증(도킹·착륙 leg 접촉) 부재.

**평가:** 72/100 (good) · 한줄평: 접촉 하이브리드 시스템의 비가우시안 입자추정을 saltation matrix로 깔끔히 일반화한 견실한 추정이론 기여이나 비가우시안·실시간 주장이 미검증이며, 검증이 로코모션 toy에 국한되어 RPOD/착륙 G&C로의 직접 관련성은 간접적이다.

---

## (2004, IEEE T-AC, Optimal Control of Switched Systems Based on Parameterization of the Switching Instants: Xu·Antsaklis)

**1. 핵심 아이디어.** 활성 서브시스템의 순서(시퀀스)가 미리 정해진 스위치드 시스템의 최적제어를, 스위칭 순간(t_1,...,t_K)에 대한 비선형 최적화 문제로 환원하는 2단계 분해를 채택한다. 핵심 기여는 스위칭 순간에 대한 최적비용의 도함수 dJ_1/dt_k를 근사가 아닌 정확한 값으로 얻는 방법이다. 이를 위해 원문제를 새 시간변수 τ로 정규화하여 스위칭 순간이 고정되는 등가 문제로 변환하고, 상태·공상태·정상성 방정식과 그 미분으로 구성된 2점 경계값 미분대수방정식(DAE)을 풀어 도함수를 산출한다.

**2. 알고리즘.** 스위칭 순간 t_1을 상태변수 x_{n+1}로 승격(dx_{n+1}/dτ=0, x_{n+1}(0)=t_1)하고, 구간별 선형 시간변환 t=t_0+(x_{n+1}-t_0)τ (0≤τ≤1) 등으로 스위칭 순간을 τ-공간에서 고정. 파라미터화 해밀토니안 H=L~_k+p^T f~_k 에 대해 상태식 ∂x/∂τ=f~_k, 공상태식 ∂p/∂τ=-(∂f~_k/∂x)^T p-(∂L~_k/∂x)^T, 정상성 0=(∂f~_k/∂u)^T p+(∂L~_k/∂u)^T 와, 이들을 x_{n+1}로 추가 미분한 감도방정식(34)-(42)를 함께 2점 경계값 DAE로 구성. 이를 풀어 식 (33)으로 dJ_1/dx_{n+1}=dJ_1/dt_1 을 얻고, 경사투영법(gradient projection)으로 (t_1,...,t_K) 갱신. GSLQ의 경우 파라미터화 Riccati 방정식(60)-(65)과 그 x_{n+1}-미분을 초기값 ODE로 풀어 DAE 없이 효율화. IFS로의 확장(Thm 2, 조건 (85))도 제시.

**3. 비판적 검토.**
- 도함수 정확성이 강점이나, J_1이 비볼록(Fig.2)이므로 경사법은 국소최적만 보장한다. 저자도 인정하나(Example 1은 운 좋게 전역해), 글로벌리티에 대한 보장이나 다중 초기화 전략은 다루지 않는다.
- 미리 정해진 활성 서브시스템 시퀀스를 전제한다. 시퀀스 자체의 조합적 탐색(이산 모드 스케줄)은 범위 밖이며, 실제 G&C에서 핵심인 모드 선택 문제는 미해결로 남긴다.
- IFS의 경우 해의 유효성(상태가 스위칭 집합에 실제 도달·교차하는지)을 사후 검증(Approach 1, step 2)해야 하며 검증 성공이 보장되지 않는다 — 저자가 future work로 명시.
- 계산비용이 크다(예제별 258s/30s/34분, 2004년 하드웨어 기준). 자작 DAE-with-jumps 솔버 의존으로 IFS 예제는 특히 느림. 실시간/온보드 적용과는 거리가 멀다.
- 검증이 자기 선행연구[30](근사법) 대비 정확도·속도 우위만 보일 뿐, 외부 방법(예: Cassandras류, 다중파라미터 프로그래밍)과의 직접 정량 비교는 없다.

**4. 시뮬/실험.** 수치예제 3건(외부 baseline·MC 없음, self-baseline). Example 1: 3개 비선형 서브시스템, t_f=3, 2회 스위칭 → 15회 반복으로 t_1=0.2262, t_2=1.0176, J=5.4399, 258.31s (vs [30] 근사법 1시간 이상, 저정확도) (sim). Example 2: GSLQ 2D, 1회 스위칭 → 17회 반복, t_1=0.1897, J=9.7667, 30.75s (vs [30] 323.18s) (sim). Example 3: IFS, t_1=1.1624, J=0.1130, ν_1=-0.0605, 약 34분 (sim). 모든 비교는 저자 자신의 선행 근사법 대비.

**5. Contribution(delta).** 선행(Cassandras 계층분해[11], 저자 자신의 [26][30] 근사 도함수) 대비, 스위칭 순간 도함수의 '정확한' 수치값을 2점 경계값 DAE/파라미터화 Riccati로 산출하는 방법이 delta. 출판년(2004) 기준으로 연속시간 스위치드 시스템에서 도함수 기반 최적화를 정확·효율적으로 수행하는 새로운 방향을 연 방법론적 기여이며, 특히 GSLQ에 대한 초기값 ODE 기반 효율화가 실질적이다. 이산화/탐색트리 가지치기에 의존하던 당시 접근과 대비되는 연속시간 정밀 해법.

**6. Limitation·Future work.** 저자: IFS 해 유효성 검증의 보장 결여, 효율적 jump-DAE 솔버 부재가 future work. 내가 보는 한계: (i) 비볼록성으로 전역해 미보장, (ii) 서브시스템 시퀀스 고정 전제로 모드 스케줄링 미포함, (iii) 불확실성/강건성 처리 전무(결정론적 설계), (iv) 오프라인 계산비용 과다로 온보드 부적합, (v) toy 규모·외부 baseline 부재. 항공우주 G&C 직접 적용은 모드 스케줄 가이던스 등으로 간접적.

**평가:** 72/100 (good) · 한줄평: 스위치드 시스템 최적제어의 정확한 스위칭-순간 도함수를 DAE로 산출하는 이론적으로 탄탄하고 영향력 큰 하이브리드 제어 기여물이나, 비볼록·오프라인·toy 검증으로 G&C에는 모드 스케줄링 측면의 near-domain 관련성을 가진다.

---

## (2024, JGCD, Autonomous Trajectory Planning with Event-Trigger Constraints Using Nonsingular Convex Optimization: Zhao·Shang·Liu·Zhang)

**1. 핵심 아이디어.** 궤적계획 문제에서 자주 등장하는 event-trigger 제약(if-then 판단문, 예: 랑데부 카메라 시야각 전환, 소행성 호핑의 접촉 판정)은 불연속·비볼록 구조를 만들어 SCP의 수렴을 방해한다. 본 논문은 if문을 연속·미분가능한 saturation 함수로 근사하고, 그 근사 정도를 homotopy 파라미터 하나로 매끄럽게 제어하여 SCP 프레임 안에서 다루는 SHSCP(Saturation Homotopy SCP)를 제안한다. 정수변수(MILP)나 maximum-함수 방식이 갖는 NP-hard성·비볼록 재유입·해 chattering을 피하고, 무엇보다 트리거되는 이벤트 개수를 사전에 지정할 필요가 없다는 점이 특징이다.

**2. 알고리즘.** if g(z)<=0 then c(z)<=0 형태의 제약을 saturation 함수 h(g,Δ)=1−1/(1+exp{Δ·(g+ε)})로 감싸 (1−1/(1+exp{(g+ε)/(Δ−1)}))·c(z)<=0 의 연속형 event 제약 e^k로 변환한다. Δ∈[Δ_m,1]는 homotopy 파라미터로, 수렴 초기엔 느슨한 근사(Δ=Δ_m)에서 출발해 SCP가 수렴함에 따라 Δ→1로 보내 원래 불연속 if문을 회복한다. 추가로 성능지표에 exp형 saturation penalization 항 η(Problem P3^k)을 넣어 SCP 탐색방향을 실현가능영역으로 유도하고(β_h 기반, h_i<=0 이면 식(35), h_i>0이면 식(36) 추가), 최소 trust region D와 Δ를 Δ=1−1/(1+exp{D·(...)})로 연동(식 37)해 둘의 수렴속도를 동기화한다. MOSEK로 볼록 부분문제를 반복적으로 푼다(Algorithm I).

**3. 비판적 검토.**
- Technical Note 성격상 SHSCP 자체의 수렴성·해 회복(Δ→1에서 원문제 복원)에 대한 형식적 증명이 없다. β_h 기반 보장과 penalization의 핵심 정리는 저자 선행연구 [27]에 위임되어 "confirmed in [27]"로 인용될 뿐, 본문에서 새로 증명되지 않는다.
- 불확실성·외란 처리가 전무하다. 본문에서 말하는 "robustness"는 초기추측(initial guess) 둔감성을 뜻하며, 항법오차·모델오차·dispersion에 대한 Monte Carlo나 보장은 없다. 명목(nominal) 시나리오에 한정된다.
- 외부 baseline이 약하다. 비교 대상은 maximum-function(MF) 방식 한 가지로, "MF가 traversal 중 종종 실패한다"는 정성 비교에 그친다. MILP나 공개 결과와의 정량 비교, 통계처리가 없다.
- 랑데부 예제에서 카메라 시야축 l_c를 속도벡터로 두는 모델링은 저자도 "본래는 자세-궤적 6DOF 결합문제이며 SCP 틀에서 풀기 어려워 타협했다"고 인정한다. 핵심 응용의 물리적 정합성에 단서가 붙는다.
- Δ와 D를 잇는 homotopy 규칙이 3종(쌍곡탄젠트/선형/지수) 제시되나, 어느 규칙이 어떤 문제류에 최적인지 선택 기준은 경험적이고, 식(37)은 "여러 백업 모델 중 하나"라고만 명시된다.

**4. 시뮬/실험.** Intel i7-6700 + MOSEK(MATLAB 2021b). 예제 A(랑데부 근접운용): 초기 [−30,−30,−30] km, t_f 500~800 s, 5단계 시야각(60→10 deg) event 제약. 총비용 0.0407~0.0481 km/s, 계산시간 1.35~3.85 s, 반복 4~9회, 수렴 Δ 0.92~0.95 (Table 3). 한계시험(Fig.6)에서 상계를 점진 축소해 event 제약을 정확히 만족시키는 궤적 생성을 보임. 예제 B(소행성 호핑, 2D): 접촉판정 if문을 갖는 hybrid 시스템, 종착 r_f 500~1000 m, 최대 13회 반복으로 Δ→0.9999 수렴, 최대 계산시간 4.34 s. MF 방식과 비교(Fig.11)에서 SHSCP 출력으로 MF를 초기화하면 거의 동일 해를 얻음을 보여 초기추측 둔감성을 시사. 모두 (sim)이며 (real)/(HIL) 없음, 외부 baseline·MC 없음.

**5. Contribution(delta).** 출판년(2024) 기준 SOTA 대비: (i) MILP(Richards 2002)의 정수변수·NP-hard성 회피, (ii) maximum-함수/state-triggered 계열(Szmuk·Reynolds·Malyuta)이 갖는 max함수의 비볼록 재유입·chattering·이벤트 개수 사전지정 요구를 saturation homotopy로 해소, (iii) 저자 자신의 collision-avoidance saturation 연구 [27]을 일반 event-trigger 문제로 확장. 핵심 delta는 "이벤트 개수를 모르는 hybrid 궤적계획"을 단일 homotopy 파라미터만으로 SCP 안에서 매끄럽게 푸는 통합 절차이며, 닫힌 if문 처리의 새 각도를 제시한 점에서 최근접 선행 대비 명확한 진전이다(다만 점진적 결합·확장 성격).

**6. Limitation·Future work.** 저자: 랑데부의 6DOF 자세-궤적 결합문제는 SCP 틀에서 풀기 어려워 미해결로 남겼고(향후과제), 카메라 시야축의 속도벡터 근사는 잠정적이다. 내가 보는 추가 한계: 수렴/회복에 대한 자체 증명 부재(선행연구 의존), 불확실성·MC·외부 baseline 부재로 강건성 주장이 정량적으로 뒷받침되지 않음, homotopy 규칙 선택의 경험성, 계산시간이 데스크톱 MOSEK 기준 초 단위라 임베디드·실시간 온보드 실증은 미흡. Δ_m·D 초기화의 적응성, 외부 솔버/MILP 대비 정량 벤치마크, dispersion 하 검증이 후속으로 필요.

**평가:** 62/100 (fair) · 한줄평: if문/이벤트 제약을 saturation homotopy로 SCP에 녹인 실용적 core-G&C 기법으로 아이디어는 깔끔하나, 자체 수렴 증명·불확실성 검증·강한 외부 baseline 부재로 견고한 in-domain 저널 수준에 머문다.
