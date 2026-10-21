# 스레드 C 비판 리뷰 — Covariance Steering · Chance Constraint · Risk Allocation (18편)

> 재교정본: 엄격 앵커(5=분야선도/드묾, 4=진짜 강함, 3=견실/보통, 2=약함, 1=없음)로 18편 일관 재채점. '전 기준 4'≠80; 평균 견실 ~60-67, 표준작만 75+, 정전급만 85+. 증분/swap·niche·증명위임·실험부재를 기준대로 반영.
> 공정성: 품질≠venue 명성. 모든 Total은 가중공식 결과 그대로(post-hoc 상·하향 금지). 수식 Unicode/plain-text, corpus 상대평가.

---

## (2021, IEEE TAES, Covariance Steering with Optimal Risk Allocation: Pilipovsky·Tsiotras)

**1. 핵심 아이디어.** 선형 시변(LTV) 확률 시스템을 초기 가우시안 분포에서 종단 가우시안 분포로 조향하는 공분산 조향(covariance steering, CS) 문제에 결합 chance constraint를 부과하되, 각 제약·시점별 위험(risk) δ를 균일 배분하지 않고 *최적으로* 배분한다. 기존 CS 연구가 위험을 상수로 고정해 SDP로 푼 것과 달리, Iterative Risk Allocation(IRA) 2단계 프레임워크(상단: 위험 배분 convex 최적화, 하단: 주어진 배분 하의 제어기 SDP)를 도입해 보수성을 낮춘다. 다면체(polyhedral) 제약뿐 아니라 LOS 같은 원뿔(cone) 제약까지 다루며, 우주선 랑데부 문제로 검증한다.

**2. 알고리즘.** 제어 정책 u_k = v_k + K_k y_k (y는 필터된 노이즈 상태). 평균/공분산 분리: X̄=Aμ0+BV, X̃=(I+BK)Y. 비용 J=‖·‖_Q̄ + tr((I+BK)ᵀQ̄(I+BK)Σ_Y) 형태. 개별 chance constraint는 Boole 부등식으로 분해 후 Φ⁻¹로 결정론적 부등식 (24): αⱼᵀE_k(Aμ0+BV) + ‖Σ_Y^{1/2}(I+BK)ᵀE_kᵀαⱼ‖Φ⁻¹(1−δ_kʲ) ≤ βⱼ. 위험 δ_kʲ를 결정변수로 두면 bilinear → Theorem 1(J*(δ)의 δ에 대한 단조감소, [26] 인용)을 이용한 IRA로 active 제약은 위험 완화·inactive 제약은 위험 강화하며 반복(Algorithm 1). Cone 제약 ‖Ax+b‖≤cᵀx+d는 (i) three-cut SOC 외부근사([27]), (ii) reverse union bound, (iii) 3차원 기하학적 근사(Theorem 2: ‖ξ̄‖+σ_ξ√(2log(1/δ)) ≤ r ⇒ P(‖ξ‖≤r)≥1−δ) 세 가지로 convex 근사.

**3. 비판적 검토.**
- 핵심 추진력인 IRA 자체는 Ono·Williams(2008, [26])에서 가져온 것이며, 단조성 Theorem 1과 SOC 외부근사 Lemma 1([27])도 인용 수입이다. 본 논문의 고유 기여는 IRA를 CS에 *결합*한 점과 cone chance constraint 근사 3종이다.
- 입력 hard constraint(‖u‖∞≤30N)로 인한 포화가 상태를 비가우시안화하여 chance constraint (23)이 더 이상 성립하지 않을 수 있음을 저자 스스로 인정하나, 이로 인한 실제 위반 확률 변화를 정량화하지 않고 "무시할 만하다"로 처리한다.
- 검증이 단일 응용(우주선 랑데부) 한 계열에 국한되고, Monte Carlo 통계나 다양한 초기조건/시드에 대한 분포가 제시되지 않는다. Table I의 true risk(Δ̄)는 단일 실행값으로 보인다.
- 실시간성·온보드 연산 분석이 전무하다. SDP(YALMIP+MOSEK, MATLAB)를 오프라인으로 풀며 solve time이나 복잡도 스케일링 보고가 없다.

**4. 시뮬/실험.** Clohessy-Wiltshire-Hill 동역학 기반 평면 랑데부(N=15, Δt=4s, 6상태, m_d=300kg), 결합 실패확률 목표 Δ=0.03. 다면체 제약: IRA의 true risk Δ̄=0.0300이 목표에 근접한 반면 uniform은 0.0123으로 과보수적(종단 공분산 부피 V_N: uniform 0.5546 vs IRA 0.6279) (sim). Cone 제약: 세 근사 비교에서 기하학적 근사가 최소 보수적(V_N=4.0909), three-cut < RUB 순으로 보수성. 외부 baseline으로 CS-SMPC[17](Δ̄=0.01128)·Tube SMPC[33](Δ̄=0.00105, u_max=30N에서는 해 없음)와 비교 — 본 방법이 덜 보수적이고 종단 공분산 자유도가 큼. Monte Carlo·실기 검증 없음.

**5. Contribution(delta).** 출판년(2021) 기준으로, 위험을 상수로 고정하던 기존 CS 문헌([8,10,14,15,17])에 IRA 기반 *최적 위험 배분*을 처음 적용한 점이 1차 delta다. 2차로 기존 IRA/CS 연구가 다면체 제약에 한정된 것을 cone(LOS) 제약으로 확장하고, three-cut·RUB·기하학적 3종 convex 근사를 제시·비교했다. 컨퍼런스 예비판([23])을 cone 제약·MPC 비교까지 확장한 저널 버전. novelty는 핵심 IRA 아이디어 수입에 따른 결합형 advance로, "방향을 여는 신규 기법"보다는 "근접 선행 대비 명확한 개선" 수준.

**6. Limitation·Future work.** 저자: 입력 포화에 의한 비가우시안화로 chance constraint 보장이 약화될 수 있음, 기하학적 근사는 3차원에 한정, IRA를 SMPC에 통합하는 것이 향후 과제. 내가 보는 한계: (i) 단일 시나리오·무 MC로 통계적 엄밀성 부족, (ii) 실시간/온보드 적용성 미분석으로 G&C 비행 적용까지 거리가 있음, (iii) 위험 배분 IRA가 일반적 비convex chance constraint(large δ)에서 전역 최적/수렴 보장을 주지 못함, (iv) 인용 속도(0.2/yr, 5년 1회)가 시사하듯 현시점 G&C 파급은 제한적.

**평가:** 72/100 (good) · 한줄평: 위험 최적배분을 공분산 조향에 결합하고 cone 제약 근사를 더한 견실한 near-domain 제어이론 기여로, 우주선 랑데부 검증으로 G&C 관련성은 직접적이나 단일 시나리오·실시간성 미검증·낮은 파급이 한계다.

---

## (2021, JGCD, Minimum-Fuel Closed-Loop Powered Descent Guidance with Stochastically Derived Throttle Margins: Ridderhof·Tsiotras)

**1. 핵심 아이디어.** 최소연료 동력하강(PDG)에서 명목 추력은 항상 최대 또는 최소 스로틀에 포화되어, 외란 보정을 위한 폐루프 추력이 스로틀 한계를 벗어나기 쉽다. 이를 막기 위해 명목 스로틀에 마진을 두는데, 결정론적 설계에서는 이 마진이 과도하게 보수적이어서 연료를 낭비한다. 본 논문은 하강 궤적을 확률과정(SDE)으로 모델링하고, 폐루프 추력 명령의 공분산을 기반으로 스로틀 마진을 비보수적으로 산출하여, 반복 없이 온보드 계산이 가능한 폐루프 PDG를 제안한다.

**2. 알고리즘.** 동역학을 점질량 SDE로 두고 dx=[v; u/m+g; -α||u||]dt+[0;γ/m;0]dw로 모델링. 제어를 아핀 피드백 u=ū+K(x-x̄)로 분리하여 평균(ODE, 식13)과 편차/공분산 동역학으로 나눈다. 공분산은 미분 리카티 방정식 Ṗ_x=(A+BK/m)P_x+P_x(...)ᵀ+(γ/m)²BBᵀ를 따른다. (i) 질량 하한 m_ℓ에 대해 제약 공분산 제어(Problem 2)를 풀어 K^ℓ, P_x^ℓ를 구하고, K(t,m)=(m/m_ℓ)K^ℓ로 두면 정리 III.2에 의해 P_x(t)≤P_x^ℓ(t)로 종단 공분산 제약을 임의 질량에서 보장(리카티 비교정리 Lemma III.1[34] 사용). (ii) 제어 공분산 P_u의 최대고유값 σ_u²로 스로틀 마진 δ를 erf(δ/(σ_u√2))-2δ/(σ_u√2π)e^(-δ²/2σ_u²)=1-β의 1차원 근찾기로 산출(정리 III.5, Lemma III.4[35]의 집중부등식). (iii) 줄어든 스로틀 한계 [ρ₁+δ, ρ₂-δ]로 결정론적 최소연료 PDG(Problem 3)를 볼록최적화로 풀어 명목 ū를 구함; 최적 추력은 max-min-max bang-bang 구조(정리 III.8). 최종 제어법칙 u(t,x)=ū(t)+K(t,m(t))(x-x̄(t)).

**3. 비판적 검토.**
- 핵심 정리(III.2의 리카티 비교, III.5의 마진 부등식)는 외부 결과([34] Freiling et al., [35] Chirikjian)를 가져와 적용한 것으로, 자체 증명은 견고하나 핵심 기계장치는 인용 수입이다.
- E[||u||]≈||ū|| 근사(식4)에 질량 동역학의 결정론성이 의존한다. 저자가 MC로 최종질량 3σ 오차 약 10kg(0.6%)임을 보여 정당화하나, 큰 외란 강도나 종단 근방의 공격적 피드백에서는 이 근사가 약화될 여지가 있다.
- 마진은 P_u의 최대고유값(등방 δ-ball)으로 보수적으로 정의되어, 비등방 제어 공분산에서는 여전히 다소 보수적이다(저자도 인정하는 conservativeness의 일부 잔존).
- 피드백 이득 K를 명목 ū보다 먼저 푸는 순차 구조(반복 회피의 장점)는 결합 최적화 대비 성능 손실을 내포; 저자가 결론에서 명시.

**4. 시뮬/실험.** MSL급 화성 동력하강(습질량 1905kg, 추진제 400kg, 최대추력 16,573N, 스로틀 30~80%, glide slope 70deg, pointing 75deg). 초기 (1.9,0,1.5)km / (-80,0,40)m/s, 종단 (100,0,100)m / 0, 종단 공분산 제약 위치 5/3m·속도 1m/s. γ=최대추력의 5%. β=10/30/50/70%의 동적 마진 4케이스 + 고정 마진 4케이스, 총 8케이스(Table 2). 2000-trial Monte Carlo로 해석적 공분산(식16) 검증; 3σ 궤적이 예측 공분산을 밀착 추종. 정량 결과: 동적 마진이 고정(constant) 마진 대비 동일 타게팅 오차에서 연료 감소(예: β=50%에서 고정 마진 대비 약 12kg, 4.7% 연료 절감). β 증가 시 마진·연료 단조 감소(Corollary III.10). 모두 (sim). baseline은 동일 논문 내 constant-margin 방식(외부 경쟁 기법 아님).

**5. Contribution(delta).** 출판년(2021) 기준, 개방루프 최소연료 PDG와 공분산 조향 폐루프 제어를 연결하여 "확률적으로 유도된 스로틀 마진"을 산출한 점이 신규 기여. 공분산 제어 이론([31-33] Chen-Georgiou-Pavon, Okamoto-Tsiotras)을 비상수·유계 질량(추력 중 질량 감소)으로 확장(정리 III.2)하고, 제어 공분산의 함수로 마진을 닫힌형/근찾기로 계산해 결정론 PDG의 입력으로 사용하는 새 정식화를 제시. 또한 미래 외란 억제를 위한 폐루프 분산 감소가 연료비를 증가시킨다는 분산-연료 트레이드오프를 형식적으로(Corollary III.9) 규명. 기존 LQ식 수동 가중치 튜닝을 제거하고 반복 없는 온보드 계산을 가능케 함. 분야를 새로 여는 수준은 아니나 닫힌 명확한 advance.

**6. Limitation·Future work.** 저자: 공기역학 항력 미포함, 외란을 일정 강도 Brownian으로 가정, 피드백을 명목보다 먼저 풀어 결합 최적화 대비 성능 손실 가능(향후 K와 ū 공동 최적화로 개선 여지). 내가 보는 한계: 검증이 화성 단일 플랫폼·순수 sim에 한정되고 외부 경쟁 기법 baseline이 없음(자기 비교), 등방 마진의 잔존 보수성, 온보드 실시간성은 "반복 없음"으로 논증되나 명시적 임베디드 타이밍/복잡도 측정이 없어 C7이 demo 수준에는 미치지 못함.

**평가:** 79/100 (strong) · 한줄평: 공분산 조향으로 비보수적 스로틀 마진을 도출한 이론적으로 견고한 core-G&C 동력하강 기여로, 외부 baseline·실시간 실측이 보강되면 더 강했을 작업.

---

## (2022, JGCD, Chance-Constrained Covariance Steering in a Gaussian Random Field via Successive Convex Programming: Ridderhof·Tsiotras)

**1. 핵심 아이디어.** 공간 의존(spatially-dependent) 불확실성을 가우시안 랜덤 필드(GRF)로 모델링하고, 명목 궤적을 따라 GRF를 시간 의존 가우시안 외란 수열로 환원하여, 평균·공분산을 동시에 조향하는 chance-constrained covariance steering 문제를 풀어낸다. 비선형 동역학은 명목 궤적 주변에서 선형화되고, 그 결과 부분 문제는 볼록 프로그램이 되며 successive convex programming(SCP)으로 원래 비선형 확률 최적제어를 반복적으로 푼다. 대표 응용은 화성 대기 밀도 변동(MarsGRAM 기반 GRF)을 명시적으로 다루는 폐루프 aerocapture 유도이다.

**2. 알고리즘.** 상태이력 피드백 u_k = Σ_{ℓ=0..k} K_{k,ℓ} x̃_ℓ + v_k (GRF로 인해 과정이 비-Markov이므로 전이력 피드백 필요). 명목 주변 선형화 ẋ ≈ f(x̂,û,μ̂)+A(x−x̂)+B(u−û)+G(Ψ̂−μ̂)로 이산화하여 x_{k+1}=A_k x_k+B_k u_k+c_k+w_k. 외란 w_k=∫Φ G Ψ̂ dt의 평균/공분산은 GRF 통계 μ̂, Σ̂의 적분으로 도출(Prop. IV.2). 블록행렬 X=A x0+B U+C+G W, 결정변수 L=K(I−BK)^{-1}로 치환하여 폐루프 공분산 P_X=(I+BL)S(I+BL)^T를 L에 대해 볼록화. chance constraint는 cdfn^{-1}(1−p)‖S^{1/2}(I+BL)^T E_k^T a‖ + a^T E_k(A x0+BV+C+G W̄) ≤ α 형태의 SOCP로. 비용은 2차비용 J1과 Δv의 99% percentile 비용 J2의 가중합 J=J1+ηJ2. trust region (54)(55) 하에서 (56) 볼록문제를 반복(Algorithm 1).

**3. 비판적 검토.**
- 핵심 볼록화 기계(L=K(I−BK)^{-1} affine controller 치환, chance-constraint SOCP, terminal 공분산 부등식)는 Skaf-Boyd[23], Okamoto-Tsiotras[5], Bakolas[19]에서 그대로 가져온 것으로, 본 논문 고유 기여는 GRF→이산 가우시안 외란 통계 환원(Prop. IV.1/IV.2)에 집중된다.
- GRF 통계를 perturbed 궤적이 아닌 명목 궤적을 따라 평가하는 것이 근본 근사인데, 그 타당성을 형식적 오차 한계가 아니라 사후 MC 일치(Fig.17)로만 정당화한다. 명목으로부터 큰 편차가 있을 때의 보증은 없다.
- chance constraint는 확률의 subadditivity로 분해되어 보수적이며, 그 보수성의 정량적 대가는 분석되지 않는다.
- aerocapture에서 초기 공분산을 0으로 설정해 대기 외란 효과만 부각시켰는데, 실제 항법 불확실성이 포함된 초기 분산 하의 성능은 검증되지 않았다.
- SCP의 수렴은 보장되지 않고(고정 3회 반복, 종료기준은 사용자 판단), Δv 절대값 항으로 인한 선형 공분산-MC 불일치(Fig.12c)가 관찰된다.

**4. 시뮬/실험.** 두 시나리오 (sim). (1) 1D double integrator + locally-periodic GRF 외력: N=5, 5,000-trial MC로 chance constraint 만족 확인(폐루프 신뢰타원이 제약영역을 약간 벗어나도 MC상 만족). (2) 화성 aerocapture: B_c=150 kg/m², L/D=0.2, r_p=3397 km, MarsGRAM 기반 밀도변동 GRF(σ_ρ,max²=1480, H_scale=11.1 km), t_f=400 s, N=10, 뱅크각 코사인 제어. 정량 결과: 최종 제어법으로 Δv의 99 percentile = 314 m/s (폐루프) vs 717 m/s (개루프, 초기 추정), 5,000-trial MC. baseline은 외부 방법이 아니라 자기 개루프/초기추정 대비. 선형 공분산(LC) vs MC 일치도(Fig.16/17)로 근사 타당성 입증.

**5. Contribution(delta).** 출판년(2022) 기준, (a) 공간 정의 GRF를 통과하는 시스템 운동에서 이산시간 가우시안 외란의 통계를 해석적으로 도출 — 기존 chance-constrained 확률최적제어 문헌이 이산 가우시안 외란을 고정·명시된 통계로 가정한 것과 대조; (b) 이를 covariance steering + SCP에 결합; (c) 불확실성(밀도변동)을 명시적으로 다루는 폐루프 aerocapture 유도를 최초로 제시(저자들이 명시한 open problem). delta의 핵심은 GRF→시간 외란 환원이며, covariance steering·SCP·chance constraint 자체는 선행(자기 그룹 포함)에서 확립된 것을 차용. 방향을 새로 여는 정도는 아니나 가장 가까운 선행(자기 그룹의 결정론적/파라메트릭 aerocapture, Okamoto의 covariance steering) 대비 명확한 진전.

**6. Limitation·Future work.** 저자: 종방향만 다룸(측방향 채널·3D 대기는 향후), 명목 궤적 따른 GRF 평가의 근사성, 더 정교한 외란 모델(hypersonic 유도)로의 확장. 내가 보는 한계: 형식적 근사 오차 한계 부재, chance-constraint subadditivity 보수성 미정량화, SCP 수렴 보증 없음, 온보드 실시간성(table-lookup 유도는 언급되나 계산시간/복잡도 분석 없음), 외부 method 대비 baseline 부재(자기 개루프 대비만), 영(0) 초기 공분산 가정으로 항법 오차 미반영.

**평가:** 78/100 (strong) · 한줄평: GRF 외란을 covariance steering에 명시적으로 연결한 이론적으로 견고한 core-G&C 기여로, 화성 aerocapture에 직접 응용되나 외부 baseline과 온보드 실시간 분석의 부재가 아쉽다.

---

## (2023, ACC, Distributionally Robust Covariance Steering with Optimal Risk Allocation: Renganathan et al.)

**1. 핵심 아이디어.** 선형 이산 확률 시스템의 공분산 스티어링(CS) 문제를 분포 강건(distributionally robust, DR) 관점으로 확장한다. 상태/잡음 분포가 가우시안이라는 가정을 버리고 모먼트 기반 모호집합(moment-based ambiguity set, 1·2차 모먼트만 고정)에 속하는 임의 분포로 모델링하고, 최악분포 위반확률을 제한하는 DR 위험제약을 부과한다. 나아가 총 위험예산을 시간·제약별로 최적 배분하는 분포강건 반복 위험배분(DR-IRA) 2단계 최적화로 보수성을 줄인다.

**2. 알고리즘.** 제어법칙은 u=V+KY (Y=A(x0-mu0)+DW)로 평균(V)과 공분산(K)을 분리 조향한다. 폴리토프 제약은 Boole 부등식으로 개별 시점 위험으로 분해 후 Cantelli 부등식으로 DR 분위함수 Q(δ)=sqrt(δ/(1-δ))를 이용해 평균에 대한 결정론적 SOC 제약으로 타이트닝: a^T E_k Xbar ≤ b - Q(δ_{i,k})·||Σ_Y^{1/2}(I+BK)^T E_k^T a||_2 (식 28). 콘 제약은 reverse union bound로 양측 SOC 근사(식 43a/b). 상단 단계는 J*(δ) 최소화로 Σδ≤Δ 하에서 δ를 배분, 하단 단계는 주어진 δ에 대해 LMI/SOCP(식 20,22)로 V,K를 구하며, 비활성 제약은 ρ로 조이고 잉여 위험예산을 활성 제약에 균등 재분배(Algorithm 1).

**3. 비판적 검토.**
- 핵심 기계장치(가우시안 CS의 평균/공분산 분리 [13], IRA 단조감소 정리 [13], DR 선형 chance 제약의 Cantelli 타이트닝 [14])가 대부분 인용·수입이며, Lemma 1/Prop 1/Prop 2의 증명은 부록 또는 "[13]과 동일하여 생략"으로 처리되어 자체 이론 기여의 깊이는 제한적이다.
- 비교 baseline이 자체 변형(가우시안 chance 제약, 균등 위험배분)에 국한된다. 외부 경쟁기법(예: scenario-based, Wasserstein-DR CS)과의 정량 비교가 없어 우월성 주장이 약하다.
- 모먼트 모호집합은 보수성이 큰 반면(Q(δ)가 Φ^{-1}보다 항상 큼, Fig.1), 그 대가로 얻는 강건성 이득을 정량적 trade-off(비용 증가 vs 위반율 감소)로 충분히 제시하지 않았다.
- reverse union bound 근사에서 ε_{i,k}를 사전 고정해 bilinearity를 회피하는데, 이 선택이 결과(보수성)에 미치는 민감도 분석이 없다.

**4. 시뮬/실험.** (sim) 두 예제. (i) 확률 이중적분기 경로계획: N=15, 500회 MC, Laplacian 잡음 샘플링; 가우시안 chance 제약은 일부 궤적이 제약 위반(Fig.2a)하나 DR은 전 구간 만족(Fig.2b), Fig.3/4로 true risk가 항상 배분 risk 이하임 확인. (ii) 우주선 근접운용: 6상태, Δ=0.15, 폴리토프/콘 제약 모두, 500회 MC. 균등 배분(Fig.5b) 대비 최적 배분(Fig.5a)이 더 낮은 비용; IRA 반복마다 비용 단조감소(Fig.6). 외부 baseline 없음. HIL/real 없음.

**5. Contribution(delta).** 출판년(2023) 기준, 가우시안 CS [13]를 임의 분포(모먼트 모호집합)로 확장한 최초 시도이며, 폴리토프뿐 아니라 convex cone 상태제약까지 DR 위험제약으로 다루고 위험배분을 DRO와 결합(DR-IRA)한 점이 delta. CS·DRO·IRA라는 기존 도구의 결합이지만, CS를 분포강건 렌즈로 본 것은 당시 신규 방향 제시에 해당한다.

**6. Limitation·Future work.** 저자: 출력 피드백(부분관측) 미지원, 선형 동역학에 한정, 비선형 확장 및 일반 모먼트 스티어링이 미해결. 추가로 내가 보는 한계: 모먼트 모호집합의 과보수성, 외부 baseline 부재, 온보드 실시간성(타이밍) 미검증, 자체 기여 증명의 깊이 제한. velocity 2.0/yr로 영향력은 중간 수준.

**평가:** 71/100 (good) · 한줄평: CS를 분포강건으로 확장한 견실한 near-domain 제어이론 기여로 우주선 근접운용 콘 제약 예제까지 다뤄 G&C 전이가능성은 명확하나, 자체 이론 신규성과 외부 baseline이 얇다.

---

## (2023, CDC, Discrete-Time Optimal Covariance Steering via Semidefinite Programming: Rapakoulias·Tsiotras)

**1. 핵심 아이디어.** 이산시간 선형 확률 시스템의 최적 공분산 스티어링(CS) 문제를 상태·제어 chance constraint와 종단 공분산 경계조건(등식/부등식) 하에서 정확하게(losslessly) 푸는 방법을 제안한다. 저자들의 선행 연구 [17]에서 제시된 비선형 프로그램의 lossless convex relaxation을 제약이 있는 경우와 부등식 종단 공분산 경계조건으로 확장하고, 이 완화가 모든 경우에 대해 lossless임을 증명한다. 그 결과 원 문제가 단일 LMI 기반 반정부호계획(SDP)으로 정확히 변환되어, 기존 SOTA([7],[9]) 대비 한 자릿수 이상의 런타임 우위와 우수한 규모 확장성을 얻는다.

**2. 알고리즘.** 상태 피드백 u_k = K_k(x_k - μ_k) + v_k를 채택, U_k = K_k Σ_k 변수 치환으로 평균/공분산 전파를 분리. 비제약 CS는 J_Σ = Σ tr(Q_k Σ_k) + tr(R_k Y_k) 최소화에 LMI [[Σ_k, U_k^T],[U_k, Y_k]] ⪰ 0와 공분산 전파 등식을 부과한 SDP로 완화(식 7). 핵심은 완화 변수 C_k ≜ U_k Σ_k^{-1} U_k^T - Y_k ⪯ 0가 최적해에서 C_k=0이 되어 무손실임을 KKT 조건과 det(R_k)=det(M_k^{(1)})det(I_p + U_k Σ_k^{-1} A_k^{-1} B_k)=0 모순 논증으로 증명(Thm 1-3). chance constraint Φ^{-1}(1-ε)√(α^T Σ α) + α^T μ - β ≤ 0는 √x ≤ x/(2√x_0) + √x_0/2 접선 과대추정으로 보수적 볼록화(식 15).

**3. 비판적 검토.**
- 무손실성 증명은 견고하나 strong duality(쌍대성 간극 0)의 핵심 단계는 선행 연구 [17]의 결과와 Slater 조건에 의존한다 — 본 논문 자체로 완결적이지 않고 외부 기계장치를 import한다.
- chance constraint의 실제 만족(위반 확률 ≤ ε)을 Monte Carlo로 검증하지 않았다. 그림은 3-sigma 신뢰타원만 표시하며, 보수적 선형화로 인한 실제 보수성/타이트니스에 대한 정량 평가가 없다.
- 비선형 쿼드로터 예제는 공칭 궤적 주위 LTV 선형화에 의존하므로, 선형화 오차가 큰 공격적 기동에서 보장이 깨질 수 있으나 이 한계에 대한 정량 분석이 부재하다.
- 핵심 novelty가 저자 자신의 선행 [17]에 대한 확장(제약 추가)이며, 같은 시기 독립적으로 [18]이 유사 완화를 부등식 종단조건에 적용했음을 본문이 인정한다 — delta가 점진적이다.

**4. 시뮬/실험.** (1) 2D 쿼드로터 경로계획: 삼중적분기 모델, N=60, ΔT=0.1s, 박스형 상태/제어 chance constraint와 2개 waypoint, Σ_i=I_6→Σ_f=0.1I_6 (sim). (2) 비선형 쿼드로터의 공격적 궤적 추종: N=500, ΔT=0.01s, 4개 waypoint, LTV 공분산 스티어링 (sim). (3) 런타임 비교(Table I,II): MOSEK/YALMIP, Intel i7-11800H. 예) N=32에서 제안법 0.17s vs Approach 2[9] 3.33s, n=32에서 151.76s vs [7]/[9] 메모리 부족 — 외부 SOTA baseline 대비 한 자릿수 이상 우위. MC·HIL·하드웨어는 없음.

**5. Contribution(delta).** 출판년(2023) 기준으로, 선행 [17]의 무손실 완화가 (i) 상태·제어 chance constraint와 (ii) 부등식 종단 공분산 경계조건 하에서도 성립함을 정리(Thm 1-3)로 증명한 점이 delta다. 제어 노력에 대한 randomized policy 해석([19]의 soft-constrained) 대신 hard-constrained 정확해를 직접 다룬다. 단일 거대 LMI 대신 N-1개의 소형 p×p LMI로 분해하여 sparsity를 활용, 이차 SDP를 푸는 [7],[9] 대비 선형 SDP로 규모 확장성을 크게 개선했다 — 이론적 무손실성 + 계산 효율의 결합이 기여.

**6. Limitation·Future work.** 저자: 비선형 시스템은 공칭 궤적 선형화에 의존; 분포가 비가우시안일 때는 Cantelli 부등식으로 보수적 대체. 내가 보는 한계: chance constraint 보수성의 정량 검증(MC) 부재; strong duality가 선행 결과에 의존; 종단 공분산 부등식 외 분포-자유(distributionally robust) 일반화 미흡; 온보드 실시간 임베디드 구현(데스크톱 오프라인 solve이며 대형 문제는 100s+ 소요)으로의 경로가 미실증.

**평가:** 80/100 (strong) · 한줄평: 무손실 SDP 완화를 제약 CS로 확장한 이론적으로 견고하고 계산적으로 우수한 control-theory 기여 — covariance steering은 RLV 유도/하강의 불확실성 제어로 직접 전이 가능한 near-domain 방법이나 항공우주 실증·MC 검증은 미흡.

---

## (2024, CDC, Computationally Efficient Chance Constrained Covariance Control with Output Feedback: Pilipovsky·Tsiotras)

**1. 핵심 아이디어.** 부분 관측(출력 피드백) 하에서 선형 확률 시스템의 상태 분포를 두 경계 가우시안 사이로 조향하는 chance-constrained covariance steering(CC-CS) 문제를 계산 효율적으로 푼다. 칼만 필터로 추정 상태를 재구성하고, 추정 상태에 대해 평균/공분산 조향을 분리한 뒤, 배치(batch) SDP 대신 시점별 순차(sequential) SDP로 풀어 규모를 줄인다. 또한 chance constraint를 difference-of-convex(DC) 제약으로 재정식화해 convex-concave procedure(CCP)로 처리, 기존 선형화 대비 보수성을 낮춘다.

**2. 알고리즘.** 제어법칙은 추정상태 affine 피드백 u_k=K_k(x̂_k−μ_k)+m_k. 비용은 평균비용 J_μ(m_k)와 공분산비용 J_Σ(K_k)로 분리됨. 변수치환 U_k=K_kΣ_x̂k 와 Y_k=U_kΣ_x̂k^{-1}U_k^T 완화로 공분산 조향을 SDP(LMI [Σ_x̂k, U_k; U_k, Y_k]⪰0)로 변환, 완화는 lossless(C_k≡0, 증명은 [26] 전상태 결과의 확장). chance constraint는 Boole 부등식으로 개별 제약 (30)으로 분해 후 양변 제곱으로 DC형 (31)/(32): f_x(Σ_x̃k;·)−g_x(μ_k;·)≤0 형태, g를 기준점 둘레로 선형화하며 CCP 반복.

**3. 비판적 검토.**
- 핵심 구성요소가 대부분 자기 그룹 선행 연구의 조합이다: 순차 SDP는 [25], 출력피드백 CC-CS는 [31], lossless 완화 증명은 [26]/[37](CCP)에서 가져온다. 본 논문 고유의 새 정리/수렴증명은 없다.
- DC(제곱) 재정식화의 등가성을 위해 부호 제약 (31b),(31d)가 필요한데 이는 β−α^Tμ≥0 즉 평균이 제약 내부에 있어야 함을 강제한다. 타이트한 경계 분포에서 이 가정의 타당성·feasibility 영향이 논의되지 않는다.
- chance constraint 만족이 3σ 타원 그림 한 장으로만 주장되고, 실제 위반율을 검증하는 몬테카를로가 없다. Boole 부등식+균일 risk allocation(δ=Δ/(NN_c))으로 보수적인데, 보수성 정도의 정량화가 없다.
- 비교 대상이 모두 동일 그룹의 방법([31] 배치, [25] 선형화)이라 외부 baseline이 부재. 또한 예제가 이중적분기 단일 시나리오로, 일반화·강건성 근거가 얇다.

**4. 시뮬/실험.** 이중적분기(n=4), N=20, Δt=0.2, 과정노이즈 G_k=0.01·I, 측정 C_k=[0_{3×1}, I_3], 측정노이즈 D_k=diag(0.1,0.003,0.003). 초기/종단 분포·다면체 상태제약(2 half-space, Δ_x=0.02) 명시. 결과: 순차 CS 대 배치 CS 해 시간(ms) — N=10:6.43 vs 25.17, N=20:10.43 vs 71.14, N=50:27.52 vs 535.44, N=100:47.83 vs 3827.02 (약 80~100배 빠르고 분산도 작음). 배치는 O(N^2) 규모(LMI (N+2)n×(N+2)n), 제안법은 N−1개의 m×m LMI로 선형 스케일. YALMIP+MOSEK. 외부 baseline·MC 없음 (sim only).

**5. Contribution(delta).** 출판년(2024) 기준 선행 대비 delta는 두 가지: (i) 출력피드백 CC-CS([31])를 순차 SDP([25] 기법)로 확장해 한 자릿수 이상 빠른 계산을 달성, (ii) chance constraint를 [25]의 기준점 선형화 대신 DC 제약으로 재정식화하여 CCP로 처리, 덜 보수적인 최적해를 얻음. 새 방법론이라기보다 확립된 도구(sequential CS, 출력피드백, CCP)의 영리한 결합에 가까운 점진적 진전이다.

**6. Limitation·Future work.** 저자: risk allocation을 고정(균일)해 sub-optimal(bi-level 공동최적화 [27]로 개선 가능), 시스템 행렬을 알아야 함(향후 data-driven 확장 예고). 내가 보는 한계: 가우시안·선형 가정에 의존, chance 만족의 MC 검증 부재, 단일 toy 예제와 자기-baseline, DC 등가성의 부호 가정이 feasible region을 제약할 수 있음, 실시간 임베디드 시연은 없고 ms 단위 타이밍 보고에 그침.

**평가:** 70/100 (good) · 한줄평: 공분산 조향 G&C 하위분야에서 출력피드백+계산효율을 결합한 견실한 점진 연구이나 novelty와 검증 폭이 제한적; covariance steering은 우주선 랑데부·진입유도 등 G&C에 직접 적용 가능한 near-domain 제어이론이다.

---

## (2024, arXiv, Optimal Covariance Steering of Linear Stochastic Systems with Hybrid Transitions: Yu et al.)

**1. 핵심 아이디어.** 선형 확률 시스템의 상태 공분산을 초기에서 목표로 최적 조향하는 covariance steering(CS)을 하이브리드(불연속 점프/리셋) 천이가 있는 시스템으로 확장한다(Hybrid Covariance Steering, H-CS). 점프 동역학을 Saltation Matrix로 1차 선형 근사하여, 비특이(invertible) 점프에 대해서는 평활 CS와 유사한 폐형 해를 유도하고, 특이성·차원 변화가 있는 일반 점프에 대해서는 Schrödinger Bridge 쌍대성을 이용해 경로 분포 위의 볼록 최적화(SDP)로 재정식화한다. 복잡도가 점프 이벤트 수에 선형으로 증가한다.

**2. 알고리즘.** 비용 J_H = E[∫(||u||^2 + X'QX)dt], 마진 제약 X(0)~N(m0,Σ0), X(T)~N(mT,ΣT), 점프 시점 t^-에서 X_2(t^+)=Ξ X_1(t^-). 평활 구간은 Riccati(2)와 Lyapunov(4)로 풀고, 점프에서 공분산은 Σ_2(t^+)=Ξ Σ_1(t^-) Ξ' (식 27), Π는 Ξ'Π_2(t^+)Ξ=Π_1(t^-) (식 25)로 연결. 비특이 Ξ: 폐형 초기조건(식 34) + 하이브리드 천이 커널 Φ^H로 적분(Thm 1). 일반(rectangular) Ξ: KL 최소화로 변환 후, 보조변수 η, Σ_η^+ = Σ^+ + ηI 로 특이성 회피, W1,W2,Σ^-,Σ^+ 위의 SDP(Thm 2, 식 50): min (1/ε)Tr(S1^{-1}Σ^-) - (2/ε)Tr(Φ'_A2 S2^{-1}W2) - ... - logdet(Y1) - logdet(Y2), s.t. Σ^+ = ΞΣ^-Ξ', Schur 보형식 제약.

**3. 비판적 검토.**
- 핵심 근사(Saltation Matrix에 의한 점프의 1차 선형화)는 충격·접촉 타이밍의 비선형성을 완전히 포착하지 못한다. 저자도 인정하나, 근사 오차가 공분산 전파에 미치는 영향을 정량적으로 분석하거나 MC로 검증하지 않았다.
- 핵심 기반(평활 CS의 Riccati/SBP 해 [4,7,8], saltation matrix [29])은 선행 연구에서 가져온 것이며, 본 논문의 기여는 하이브리드 천이로의 확장 증명에 집중된다. 증명 자체는 자체적이고 완결적이다.
- 검증이 nominal 위주다. 공분산 제어 방법임에도 노이즈 강도 ε 변화·근사 오차에 대한 견실성(robustness) 스윕이 없어 실제 불확실성 하 성능 보장이 경험적으로 약하다.
- 평균 궤적은 외부 H-iLQR로 얻고 그 위에 CS를 얹는 구조라, 평균-공분산 결합이 약하게 분리(decoupled)되어 있다. 비선형 하이브리드 시스템으로의 완전 결합은 future work로 남김.

**4. 시뮬/실험.** 2개 예제 (모두 sim). (1) 1D 탄성 충돌 bouncing ball (2모드, e2=0.6, Σ0=0.2I→ΣT=0.05I): H-iLQR와 비교해 H-CS가 단말 공분산 제약을 정확히 만족(H-iLQR는 단말 페널티 추가 필요), Thm 1 해석해와 Thm 2 SDP 수치해가 Σ^-,Σ^+에서 일치(Table 1, 예: Σ^- 대각 0.1651/0.2640 일치). (2) 선형화 SLIP (4D stance→5D flight, 차원 증가로 Σ^+ 특이): Ξ_12 제시, 최적 Σ^+가 실제로 특이임을 확인, H-iLQR의 단말 공분산은 제약 미충족인 반면 H-CS는 만족. baseline = H-iLQR [26,27] (외부). MC·통계·실시간 타이밍 없음.

**5. Contribution(delta).** 출판년(2024) 기준, covariance steering을 하이브리드/접촉 천이 시스템으로 확장한 최초급 정식화. 선행 평활 CS [4–8]는 평활 동역학에 한정, 접촉-rich 불확실성 처리법 [10,40]은 stochastic complementarity로 제약 보장이 없거나 비용 페널티에 의존([48] H-iLQR도 단말 공분산을 페널티로만 다룸). 본 논문은 (i) 비특이 점프의 폐형 해와 (ii) 차원 변화·특이 공분산을 SDP 등식 제약으로 정확히 처리하는 볼록 정식화를 제시하고, 복잡도가 점프 수에 선형이라는 점을 보장한다. 이는 명시적 단말 공분산 제약을 정확히 만족시키는 점에서 페널티 기반 선행 대비 명확한 delta.

**6. Limitation·Future work.** (저자) 비선형 하이브리드 동역학으로의 반복적(iterative) 공분산 제어가 미해결; 현재는 1차 saltation 선형 근사에 의존. (내가 보는 한계) saltation 근사 오차의 정량 평가·MC 견실성 부재, 온보드 실시간성(연산 시간/임베디드) 미측정, 검증이 robotics 토이 예제(공·SLIP)에 한정되어 항공우주 G&C(EDL/착륙) 직접 적용은 미시연. 평균-공분산 분리 구조의 최적성 손실 미분석.

**평가:** 77/100 (strong) · 한줄평: 하이브리드 천이로의 공분산 조향 확장으로 이론적으로 견고하고 특이 점프까지 볼록하게 다루는 좋은 control-theory 기여이나, 검증이 robotics 토이 예제·nominal에 머물러 항공우주 G&C 관련성은 (CS의 EDL 활용을 통해) 간접적이다.

---

## (2024, CDC, Distributionally Robust Density Control with Wasserstein Ambiguity Sets: Pilipovsky·Tsiotras)

**1. 핵심 아이디어.** 잡음 분포를 정확히 모르는 상황에서 LTI 시스템의 상태 분포를 목표 분포로 조향(steering)하는 분포적 강건(distributionally-robust) 밀도 제어(DR-DS) 문제를 다룬다. 잡음의 분포적 불확실성을 Wasserstein ambiguity set으로 모델링하고, affine 피드백 제어로 상태의 ambiguity set을 목표 terminal ambiguity set으로 전파·조향한다. 동시에 분포적으로 강건한 CVaR(DR-CVaR) 제약을 경로 전체에 부과하여, worst-case 외란 하에서도 제약 위반 확률을 억제한다.

**2. 알고리즘.** 증강 동역학 x=Ax0+Bu+Dw, error state x̃=(I+BL)Dw에 affine 피드백 u_k=K_k x̄_k+v_k 적용. CS 표준 기법으로 L≜K(I−BK)^{-1}로 재매개화(decision var = v,L). Wasserstein ball을 Gelbrich(평균-공분산) ambiguity set으로 외부 근사하여 DR-CVaR 제약(15)을 Theorem 3의 볼록 제약(19)으로, 다시 Schur complement로 SOCC/LMI(21a,b)로 변환. DR 2차 비용(22)은 Theorem 4·Corollary 2를 거쳐 SDP(24)로 환원. 최종적으로 전체 DR-DS는 단일 SDP: min β·Σ‖E_k v‖ + λ(ε²−tr[Σ_w]) + tr[Γ] s.t. (21),(24b),(24c),(28),(29),(30). 잡음 시퀀스 ambiguity set은 zero-mean Gaussian P̂_w=N(0,Σw) 중심, 반경 ε.

**3. 비판적 검토.**
- 핵심 이론 기계(Gelbrich ambiguity set과 그 outer-approximation, 선형변환 하 ambiguity set 전파 Theorem 1, DRO 비용/CVaR 재정식화)는 대부분 [20],[22]에서 import한 것이며, 본 논문의 기여는 이를 feedback gain L까지 확장·재배치한 것이다. 자체 증명(부록 A–D)은 있으나 토대는 인용 기계.
- joint DR-CVaR(polytope 전체에 대한 동시 위험)은 L에 대해 비선형이라 다루기 어렵다고 인정하고, 각 facet·각 시간스텝으로 위험을 분할(per-side, per-step)하는 보수적 단순화를 취한다. 이는 보수성을 키우며 joint 정식화는 open으로 남김.
- Gelbrich set이 Wasserstein set의 outer approximation이므로 제약 만족은 충분조건일 뿐 보수적이다. 저자도 명시하나 보수성 정량화는 없음.
- 실시간/온보드 관점 분석이 사실상 부재. "polynomial time SDP"라는 언급뿐, 솔버 시간·문제 크기 대비 계산량·임베디드 적합성 평가가 없어 quadrotor landing이라는 응용에 비해 deployment 근거가 약함.
- baseline이 동일 그룹의 CS(chance-constrained) [6] 단일 비교에 그쳐, 외부·독립 비교군이 부족하다.

**4. 시뮬/실험.** 두 시나리오 (모두 sim). (i) 2D double integrator: N=20, Δt=0.3, i.i.d. N(0,I) 명목 잡음, terminal radius δ=0.05, 경로 chance |x|≤0.2(γ=0.05). MC=1000. mis-characterized Gaussian 하에서 경험적 제약위반 risk DR-DS 0.1% vs CS 5.5%; 비-가우시안 t-분포(3 DOF) 하에서 joint risk DR-DS 0.3% vs CS 3.5%. (ii) 9-DOF quadrotor landing + Dryden wind turbulence: CasADi 기준궤적 후 선형화, N=10, T=5s, ε=1, δ=0.1. MC=1000 splash-point. 극한 풍속(V0=20 m/s)에서 terminal position 표준편차 비율(σ/σ̄)이 x≈5.1, y≈9.2까지, 최대 ~15× 감소(Table II); 단 z방향 감소는 미미(≈1.1×).

**5. Contribution(delta).** 출판년(2024) 기준, Wasserstein-DR 최적제어를 open-loop 제어만 다룬 선행 [22]를 넘어 open-loop와 feedback gain을 동시에 최적화한 첫 연구로 자평. CS 문헌의 affine feedback 재매개화를 DR 설정에 결합하여, 상태의 분포적 불확실성을 terminal ambiguity set으로 조향하고 DR-CVaR 경로 제약을 SDP로 다룰 수 있게 함. Chebyshev(공통 모멘트, [1]) 대비 Wasserstein의 표현력을 확보. 새로운 방향을 여는 수준은 아니나 CS+DRO+feedback 결합으로 최근접 선행 대비 명확한 진전.

**6. Limitation·Future work.** 저자: reference 잡음 분포가 알려진 Gaussian/elliptical 중심을 가정 — data-driven(경험 분포 기반) 정식화는 future work; joint DR-CVaR 정식화는 미해결; quadrotor 경로 제약 미부과로 transient dispersion이 넓음. 추가로 내가 보는 한계: 보수성(Gelbrich outer-approx + per-facet 분할) 정량화 부재, 온보드 실시간성 미분석, 독립 baseline 부족, quadrotor는 선형화 sim에 그쳐 실기·HIL 검증 없음.

**평가:** 75/100 (strong) · 한줄평: Wasserstein ambiguity 기반 feedback 분포 조향을 SDP로 정식화한 이론적으로 탄탄한 covariance-steering 확장이며, quadrotor landing으로 G&C 관련성을 보였으나 핵심 기계는 import·검증은 선형 sim·온보드 분석 부재로 strong 하단에 위치.

---

## (2025, AIAA SciTech, Risk-Sensitive Orbital Debris Collision Avoidance Using Distributionally Robust Chance Constraints: Ryu et al.)

**1. 핵심 아이디어.** 비선형 궤도 동역학에서 잔해 위치 불확실성은 빠르게 비가우시안으로 발산하여 충돌확률 기반 chance constraint를 직접 평가하기 어렵다. 저자들은 평균·공분산만 공유하는 모든 분포(moment-based ambiguity set)에 대해 chance constraint를 보장하는 distributionally robust(DR) 정식화를 제안하고, 이를 CVaR로 보수적·tractable하게 근사한다. 결과적으로 불확실성 전파 방법(선형 가우시안/UT/Monte Carlo)에 agnostic한 충돌 회피 MPC 제어기를 얻는다.

**2. 알고리즘.** 안전비용 l(r_d)을 충돌-free 집합의 sublevel set으로 정의하고 chance constraint Prob(l(r_d)<=0)>=1-eps 를 VaR<=0, 다시 CVaR<=0 로 보수적 치환한다. 핵심은 타원체 충돌-free 집합 R_free={r: (r-mu)^T E (r-mu)-1<=0}에 대한 폐형 결과(Theorem 1): sup_{P in P^k} CVaR_eps(l(r_d)) = -1 + (1/eps)·Tr{Sigma_d^k E^k}. 이를 제약 -1+(1/eps)Tr{Sigma E}<=0 로 MPC에 삽입하고, 비선형 동역학 때문에 Constrained Cross-Entropy Method(CEM)로 제어열을 샘플링·거부·elite 갱신하여 푼다. 가능해가 없을 때는 Trajectory_Risk(할인된 제약값 합)로 정렬해 안전한 방향으로 유도한다.

**3. 비판적 검토.**
- 핵심 폐형 정리(Theorem 1)의 증명이 "See Proof of Corollary 1.3 in [17]"로 외부 인용에 의존한다. VaR/CVaR-chance 동치, DR-CVaR 폐형 모두 기존 risk-sensitive/DR control 문헌(Van Parys, Zymler, Chow et al.)의 import이며 자체 신규 증명은 사실상 없다.
- 타원체로 R_free를 under-approximation하여 비볼록 안전영역을 볼록화하는데, 이 보수성(실제 안전영역 대비 손실)이 정량화되지 않았다. eps와 E의 스케일에 따라 과도하게 보수적일 수 있다.
- 비교 대상이 외부 baseline 제어기가 아니라 자사 제어기의 3가지 전파 변형(LG/UT/MC) 간 비교에 그친다. [6]의 Mahalanobis 기반 convex 방법 등 선행 충돌회피 제어기와의 직접 정량 비교가 없다.
- 시나리오가 단일 Starlink-THEA 근접 conjunction 하나에 국한되어, 다양한 기하/속도/접근각에 대한 일반성 검증이 부족하다. moment ambiguity set은 평균·공분산이 정확히 주어진다는 가정에 민감한데 그 추정 오차의 영향은 다루지 않았다.
- CEM은 샘플링 기반이라 비용이 크고, MPC를 1s마다 호출하나 실시간/온보드 연산시간·복잡도 분석이 전혀 없다(저자도 future work로 더 정교한 solver 교체를 언급).

**4. 시뮬/실험.** MATLAB, ECI 비선형 2체+대기항력(지수모델, RK4 dt=0.01s), MPC horizon 10s·1s마다 재계산. SOCRATES Plus/CelesTrak에서 식별한 실제 영감 conjunction: Starlink(NORAD 52579, 300kg) vs THEA 3U CubeSat(NORAD 43796, 50kg, 비추력), 예측 최근접 41m, d_thres=100m. 3개 전파법(선형가우시안/UT/MC N=50) 비교. eps∈{0.05,0.01,0.001} 및 잔해 노이즈 Q∈{0.05,0.01,0.005}I 스윕, 각 설정 10회 반복(평균±표준편차). 결과(sim): eps 감소 시 최소거리·Δv 증가(예 UT: 0.55→0.57→1.36 km, Δv 72→160→250 m/s); 선형가우시안은 공분산 형상을 잘못 추정해 과도 보수적 기동(Δv 184→291→746 m/s)으로 비효율. 외부 baseline 제어기 비교는 없음.

**5. Contribution(delta).** (i) 잔해 위치의 평균·공분산만 요구하는 moment ambiguity set 위에서 chance constraint를 보장하는 DR-CVaR 충돌회피 제어기를 정식화하고, (ii) 불확실성 전파법에 agnostic함을 LG/UT/MC 세 방법으로 실증, (iii) 타원체 안전영역에 대한 폐형 제약 -1+(1/eps)Tr{Sigma E}<=0 을 CEM-MPC에 통합. 출판년(2025) 기준 novelty는 DR-CVaR·moment ambiguity(이미 control/robust optimization에서 확립)를 궤도 잔해 충돌회피에 이식하고 전파-agnostic 프레이밍을 부여한 점이며, 핵심 수학 기계는 신규가 아니어서 incremental~moderate advance에 해당한다.

**6. Limitation·Future work.** 저자: 샘플링 기반 CEM을 더 정교한 solver로 대체, 다중 물체(multi-satellite) 충돌회피 확장. 추가로 보는 한계: 단일 시나리오·외부 baseline 부재로 실증 일반성 약함, 타원체 under-approximation 보수성 미정량, 평균·공분산 추정 오차에 대한 robustness 미검증, 온보드 실시간성·연산복잡도 분석 전무, J2 등 추가 섭동 미반영.

**평가:** 68/100 (good) · 한줄평: 확립된 DR-CVaR·moment-robust 기법을 궤도 잔해 충돌회피에 견실하게 이식한 in-domain G&C 응용 논문으로, 정식화·전파-agnostic 실증은 깔끔하나 핵심 이론이 import이고 외부 baseline·실시간성 검증이 빠져 incremental에 머문다.

---

## (2025, CDC, Operator Splitting Covariance Steering for Safe Stochastic Nonlinear Control: Ratheesh et al.)

**1. 핵심 아이디어.** 비선형 동역학과 chance constraint를 갖는 공분산 스티어링(CS) 문제를, 전체 문제를 chance-constraint 부분과 두 개의 스티어링(평균/공분산) 부분으로 일시적으로 분리(operator splitting)하여 ADMM으로 병렬 풀이하는 알고리즘을 제안한다. 핵심 통찰은 기존 SCP 방식이 매 반복마다 모든 제약을 동시에 만족하는 feasible iterate를 요구해 보수적 근사로 인해 infeasible 보고나 품질 저하를 일으키는 반면, 본 방법은 수렴 전 중간 iterate의 infeasibility를 허용하여 탐색성과 feasibility를 높인다는 점이다. 시뮬레이션과 Robotarium 하드웨어 실험으로 검증한다.

**2. 알고리즘.** 비선형 시스템 X_{t+1}=f(X_t,u_t)+D(X_t)W_t에 대해 선형 피드백 u_t=v_t+K_t(x-μ_t) 파라미터화. 명목 궤적 τ 주변 1차 Taylor 전개로 affine 동역학(A_t,B_t,D_t,d_t)을 얻고 평균(8a)·공분산(8b) 전파를 유도. 비용은 2차 근사하여 평균비용 J_μ와 공분산비용 J_Σ로 분리. chance constraint는 Boole 부등식 + 각 h_i 선형화 → 반평면 제약 g_{i,t}(μ,Σ)=a^T μ+b+Φ^{-1}(1-δ')√(a^T Σ a)≤0으로 변환(Σ에 대해 concave → 다시 1차 선형화로 보수적 근사). 결정변수 복사본 z_μ,z_Σ,z_cc,z_cn을 도입해 (19)로 재정식화하고 scaled augmented Lagrangian (20)에서 z_μ,z_Σ,z_cc는 prox 연산으로 병렬 업데이트(21), z_cn은 평균화(22), 이중변수 λ,Λ는 (23)으로 갱신. 외부 루프 ForwardPass로 명목 궤적을 process noise 전파로 재추정(Alg.2), 내부는 trust-region SCvx + proximal regularizer J_px.

**3. 비판적 검토.**
- 수렴 증명이 본문에 없다. 외부(trust-region SCvx [43])·내부(ADMM 수렴 [42]) 기존 결과를 import한다고만 하고 "formal exposition...extended version"으로 미룬다 — CDC 분량 한계는 이해되나 본 논문 자체의 정리·증명은 부재.
- chance constraint 근사가 이중 보수적(Boole 부등식 + concave g의 선형화)이다. 본 방법의 장점은 보수성 자체를 줄이는 것이 아니라 중간 infeasibility 허용으로 feasibility를 확보하는 것이며, 근사 보수성은 여전히 남는다.
- Fig.2c가 직접 인정하듯, 명목 궤적 재선형화 시 dynamically feasible 집합과 chance-feasible 집합이 disjoint가 되는 한계가 존재한다(저자도 limitation으로 언급).
- baseline 비교가 double integrator(선형)로 단순화되어 있어, 비선형에서의 우월성 주장은 정성적 그림(Fig.2a/b)과 Table II의 단일 사례에 의존한다.
- 실시간성 미흡: solve time이 unicycle 2.67s, quadrotor 25.68s, Robotarium 19.03s로 offline 계획 수준. "병렬화 가능"은 구조적 주장이며 실제 병렬 가속 측정치는 제시되지 않는다.

**4. 시뮬/실험.** Unicycle(3상태, 5장애물, 10개 환경 평균): cost 7.93/safety 0.990 (estimated 8.29/0.975), 200 샘플 MC, 평균 2.67s (sim). Quadrotor(12상태/4입력, 구형 장애물): cost 762/safety 0.990 (est 821/1.000), 25.68s, 5 샘플궤적 (sim). Robotarium GritsBot 하드웨어(unicycle, 100 trials): safety 0.900(est 0.970), wheel-velocity 제약 추가, 100회 중 3회만 장애물 위반, 19.03s (real/HIL). Baseline(single-SDP, no splitting, double integrator): δ'=0.99에서 No Solution인 반면 본 방법은 cost 5.1954/safety 0.99 달성(Table II). CVXPY+MOSEK, i9-14900K.

**5. Contribution(delta).** 출판년(2025) 기준, 비선형 + chance-constrained CS에 operator splitting/ADMM을 적용해 chance-constraint 만족을 두 스티어링 문제로부터 분리하고 중간 iterate의 infeasibility를 허용하는 첫 정식화. 선행 SCP 기반 비선형 CS(Ridderhof, Yu, Saravanos 등)가 매 반복 feasible 근사를 강제하던 것과 달리, 일시적 infeasibility를 통해 더 tight한 안전 제약(δ'=0.99)에서도 baseline이 실패하는 문제를 풀고 더 낮은 비용 해를 찾는다. ADMM 분리로 subproblem 병렬화 가능성을 제공하는 것도 구조적 기여.

**6. Limitation·Future work.** 저자: 재선형화 시 동역학-chance feasible 집합 disjoint 문제(Fig.2c), 빈약한 초기화 시 외부 루프 수렴 느림(RRT* 등 warm start 권장), 수렴 분석 본문 부재. Future: Bregman ADMM, sampling-based motion planning(LQR-Trees) 결합, learning-to-optimize 통합. 내가 보는 한계: (i) 검증이 전부 robotics(unicycle/quadrotor/GritsBot)로 항공우주 G&C 직접 검증 부재 — CS는 화성 동력하강(Ridderhof) 등 EDL 불확실성 제어에 직결되나 본 논문은 transfer; (ii) 이중 보수 근사로 실제 안전확률이 목표 대비 과보수(0.975~1.000); (iii) 실시간 onboard 미입증.

**평가:** 76/100 (strong) · 한줄평: 비선형 chance-constrained CS의 infeasibility-tolerant ADMM 분리는 명확한 방법론적 진전이고 하드웨어까지 검증했으나, 수렴증명 부재·offline 수준 속도·robotics 한정 검증이 약점 — 항공우주 EDL 불확실성 제어로의 관련성은 높은 near-domain transfer.

---

## (2025, CDC, Wasserstein Distributionally Robust Adaptive Covariance Steering: Gahlawat et al.)

**1. 핵심 아이디어.** 비선형 확률 시스템(Itô 과정)에서 aleatoric(비유계 random 외란)와 epistemic(상태 의존 모델 불확실성)이 공존할 때, 안전하고 예측 가능한 covariance steering(CS) 제어를 제공하는 방법론이다. 핵심은 저자들의 선행 L1-DRAC(distributionally robust adaptive control)를 저수준 증강 제어로 사용하여, 참(true) 시스템 상태분포가 명목(nominal) 분포 주위의 Wasserstein 모호집합(ambiguity set)에 확률 1로 포함된다는 인증서를 만든다는 점이다. 이 모호집합을 고수준 CS 플래너의 분포적 강건 CVaR 안전제약으로 주입하여, 참 분포로부터의 샘플이나 사전 모호집합 지식 없이도(data-free) 안전을 보장한다.

**2. 알고리즘.** 입력은 U_t = U*_t + U_{L1,t}로 분해(식10). U*_t는 명목 CS 최적제어, U_{L1,t}는 저역통과 필터 출력 U_{L1,t}=-ω∫e^{-ω(t-ν)}Λ̂(ν)dν(식11)와 적응법칙(식12) 및 process predictor(식13)로 구성. Theorem 1([37] 인용)로 W_{2p}(X_t, X*_t) ≤ ρ, ∀t의 균일 유계가 확보되어 모호집합 B_ρ(X*_k) 멤버십이 확률1로 성립(식14,16). 고수준은 CS 최적제어 J=(X*_T)^T Q X*_T+(U*_T)^T R U*_T를 평균/공분산 경계조건(식23b-d) 하에 풀되, 각 안전 반공간에 대해 분포적 강건 CVaR 제약 sup_{ν∈B_ρ} CVaR ≤ 0을 명목 CVaR + φ(Φ^{-1}(1-δ_s/n_l))√(c^T Σ* c)/(δ_s/n_l) + ‖c‖ρ/√(δ_s/n_l) ≤ 0 로 닫힌형으로 변환(식20,21). 장애물 선택은 Big-M 혼합정수(식23e-g)로 표현하고 conic solver(Mosek)로 해석(Algorithm 1). Theorem 2: 이렇게 얻은 U*_T로 CVaR_{1-δ_s}[X_k∈X_safe]≤0이 성립.

**3. 비판적 검토.**
- 시뮬레이션·수치예제가 전혀 없다. Fig.1은 개념 만화이며 Section IV는 결론+future work뿐. 정량적 검증, baseline 비교, 해의 solvability·보수성 정도가 일절 제시되지 않아 TAES/TASE 기준 실증 근거가 부재하다.
- 핵심 보증인 Theorem 1은 저자 선행 [37]에서 그대로 import한 것이고, Theorem 2의 증명은 "trivial"로 처리된다. 즉 본 논문의 이론적 기여는 새로운 정리가 아니라 기존 L1-DRAC 균일유계를 CVaR/Wasserstein 제약으로 연결하는 재구성에 가깝다.
- Euler-Maruyama 이산화 오차를 "trivial"이라며 ρ에 명시적으로 반영하지 않는다고 밝힌다. 연속시간 보증(식14)과 이산시간 CS(식15) 사이 간극이 안전 마진에 미치는 영향이 정량화되지 않아 보증의 엄밀성에 구멍이 있다.
- 분포적 강건 CVaR 항에 ‖c‖ρ/√(δ_s/n_l) 가산은 ρ가 클수록 제약을 크게 부풀려 잠재적으로 매우 보수적이거나 infeasible해질 수 있는데, 이 보수성-feasibility tradeoff에 대한 분석이 없다.
- Big-M 혼합정수 conic 문제의 규모·해 시간이 보고되지 않아 고수준 플래너의 실용성(특히 온보드)을 판단할 수 없다.

**4. 시뮬/실험.** 없음. 수치 시뮬레이션, 시나리오, 정량 결과, baseline 비교가 전혀 제시되지 않았다. Fig.1의 장애물 회피 내비게이션은 개념적 삽화일 뿐 실행된 실험이 아니다.

**5. Contribution(delta).** 출판년(2025) 기준 SOTA인 Wasserstein DRC 계열은 (i) 참 분포로부터의 유한 샘플 가용성 또는 (ii) 시변 모호집합의 사전 지식을 요구한다. 본 논문의 delta는 L1-DRAC 적응 제어가 확률1로 Wasserstein 인증서를 생성하게 하여 이 두 요구를 모두 제거(data-free)하고, 그 모호집합을 CS 고수준 플래너의 분포적 강건 CVaR 안전제약으로 통합한 아키텍처적 결합이다. 기존 SOC 제어기에 추가 제약만 부가하므로 plug-in 가능하다는 점도 새롭다. 출판년 기준으로 명확한 선행 대비 진전이다.

**6. Limitation·Future work.** 저자: 대안 고수준 SOC 제어기로의 확장, 환경 불확실성에 대한 강건성, learned component의 루프 내 통합을 미래 과제로 제시. 내가 보는 한계: 가장 결정적으로 실증이 전무하여 방법이 실제로 풀리는지·보수성이 수용 가능한지 미검증. 이론도 핵심 정리를 선행에서 import하고 Theorem 2는 trivial 증명. 이산화 오차 미반영, MISDP 계산비용·온보드 실시간성 미분석. Gaussian 초기/종단 분포 가정에 한정.

**평가:** 62/100 (fair) · 한줄평: 샘플 불요·확률1 Wasserstein 인증서와 CS+CVaR의 결합은 control-theory 관점에서 신선하고 안전 G&C 플래닝에 근접 관련성이 있으나, 시뮬레이션·baseline·타이밍이 전무하여 publication-grade 실증이 비어 있다.

---

## (2025, arXiv, Chance-Constrained Covariance Steering for Discrete-Time Markov Jump Linear Systems: Shrivastava·Oguri)

**1. 핵심 아이디어.** 모드가 마르코프 연쇄로 전이하는 이산시간 Markov Jump Linear System(MJLS)에 대해 초기 분포를 목표 분포로 유도하는 chance-constrained 공분산 스티어링 문제를 풀이한다. 단일 모드 시스템에서 성립하는 평균-공분산 분리(separation)가 MJLS에서는 성립하지 않음을 해석적으로 증명하고, 이를 우회하기 위해 평균과 공분산을 순차적으로 푸는 2단계 볼록 최적화 틀을 제안한다. chance constraint는 Chebyshev/Cantelli 부등식으로 결정론적 볼록 제약으로 변환하고, 핵심 볼록 완화(relaxation)가 KKT 조건상 최적점에서 lossless(원 등식 제약과 동치)임을 증명한다.

**2. 알고리즘.** 부분기댓값 변수 q_k(i)=E[x_k·1{r_k=i}], S_k(i), ρ_k(i)로 동역학을 재기술한다. 모드별 affine 피드백 u_k=ū_k(i)+K_k(i)(x_k−x̄_k(i)). 비볼록 공분산 전파식(eq.7/11)을 L_k(i)=K_k(i)S_k(i), Y_k(i)=L_k(i)S_k^{-1}(i)L_k^T(i)로 변수치환하고, C_k(i)=L_k S_k^{-1} L_k^T − Y_k ⪯ 0 를 Schur 보완으로 LMI화. Theorem 1/2: KKT(eq.20,30)로 모순법을 써서 C_k(i)=0(losslessness) 증명. chance constraint는 Lemma 2(half-space): a^T v̄+b+sqrt((1−ε)/ε · a^T V a)≤0, Lemma 3(norm bound): ||v̄||+sqrt(n_v/ε·λ_max(V))≤v_max 로 변환 후 양변 제곱하여 공분산에 대해 볼록화(eq.25,26). Algorithm 1: Problem 4(평균 초기화)→Problem 7(공분산)·Problem 6(평균)을 슬랙 1-norm 가중치를 η배 증가시키며 수렴까지 반복.

**3. 비판적 검토.**
- 평균-공분산 비분리성 증명(Remark 1)과 lossless 완화 증명은 이 논문의 핵심 기여이며 자체 증명이 완결적이다. 다만 chance-constraint 변환의 기반 보조정리(Lemma 1[11], Lemma 2[26]/[8], multivariate Chebyshev[25])는 외부 인용으로 가져온 기계장치다.
- 평균-공분산 양방향 결합을 계층적 반복으로 푸는데, 이 반복 자체의 수렴성은 선행 연구[27][28]의 성질에 위임할 뿐 본 MJLS-결합 설정에 대한 형식적 수렴 보장은 제시되지 않는다.
- 외부 baseline과의 정량 비교가 전무하다. mode-independent 제약 대비 보수성 감소, 기존 covariance steering 대비 계산시간/성능 우위가 수치로 비교되지 않아 우월성 주장이 정성적이다.
- Chebyshev/Cantelli 기반 변환은 분포 무관(distribution-free)이라 안전하지만 보수적이다. MJLS 종단 분포가 지수 가우시안 혼합(N^T개)임을 지적했으면서도, 이 혼합 구조를 직접 활용한 덜 보수적인 대안과의 보수성 정량 비교가 없다.

**4. 시뮬/실험.** 두 수치예제. (예제1) 4상태 2모드 추상 시스템, ρ0=[0.3,0.7], half-space 및 control-norm chance constraint(0.95). (예제2) 3모드 double integrator(정상/풍교란/제어권상실), 풍교란 a_w가 시스템을 위험영역으로 밀어냄, T=20. 각 2500 Monte Carlo. 정량결과: 예제1은 3.43초·7 iteration 수렴, 위반율 0.01%(<5% 한도); 예제2는 79초·5 iteration, 위반율 0.02%(<5%). 종단 공분산 그림(Fig.4,7)에서 optimizer 예측 공분산·샘플 공분산·종단 제약 Σ_f가 일치, MJLS의 다봉(multi-modal) 궤적이 가시화됨. 2023 MacBook Pro M1 Pro·CVXPY+MOSEK. External baseline 없음(self).

**5. Contribution(delta).** 출판년(2025) 기준, chance-constrained 공분산 스티어링을 MJLS 클래스로 최초 확장. (1) 단일모드 평균-공분산 분리[10]가 MJLS에서 무너짐을 해석적으로 입증, (2) Schur 보완 + 변수치환으로 MJLS 공분산 부문제의 lossless 볼록 완화를 KKT로 증명, (3) Chebyshev/Cantelli 기반 mode-conditioned chance constraint를 결합한 sequential convex 틀. covariance steering·MJLS 제어 각각은 기성이나, 결합 시 발생하는 비분리·rank-1 항으로 인한 2단계 분해 필요성을 규명한 점이 명확한 advance.

**6. Limitation·Future work.** 저자: time-homogeneous 가정(time-inhomogeneous로 trivial 확장 언급), Slater 조건 필요. 추가로 내가 보는 한계: (a) 계층 반복의 형식적 수렴 보장 부재, (b) 외부 baseline·보수성 정량 비교 부재, (c) Chebyshev 기반 변환의 보수성, (d) 실시간/온보드 검증 부재(79초는 오프라인 설계 수준), (e) 모드 전이확률 P의 불확실성·추정오차에 대한 강건성 미고려.

**평가:** 77/100 (strong) · 한줄평: MJLS 공분산 스티어링의 비분리성 규명과 lossless 볼록 완화 증명이 견고한 control-theory 기여이며, 추력실패·풍교란 등 고장허용 우주 G&C 시나리오에 직접 연결되나 외부 baseline·실시간 검증은 부족하다.

---

## (2025, arXiv, Strong Duality and Dual Ascent Approach to Continuous-Time Chance-Constrained Stochastic Optimal Control: Patil et al.)

**1. 핵심 아이디어.** 연속시간·연속공간 chance-constrained SOC 문제(상태제약 위반 확률 ≤ Δ)를 보수적 근사 없이 정확히 다룬다. 연속시간 확률미적분의 exit time 개념을 이용해 chance constraint를 안전영역 경계 지시함수(indicator)의 기댓값으로 변환하고, 이를 라그랑지안에 흡수시켜 dual 문제를 구성한다. dual 목적함수는 라그랑지 승수 η로 파라미터화된 HJB PDE의 해로 표현되며, 일정 가정 하에 primal-dual 강쌍대성(duality gap=0)을 증명한다. 마지막으로 path integral 기반 dual ascent로 open-loop 궤적 샘플을 이용해 온라인 수치해를 구한다.

**2. 알고리즘.** 비용 C = E[ψ(x(t_f))·1_{x(t_f)∈X_s} + ∫(½uᵀRu+V)dt], 제약 P_fail ≤ Δ. 핵심 변환: P_fail = E[1_{x(t_f)∈∂X_s}] (exit time 경계 지시함수). 라그랑지안 L = C + η(E[1_{x(t_f)∈∂X_s}] − Δ)를 종료비용 φ(x;η)=ψ·1_{X_s}+η·1_{∂X_s}−ηΔ로 흡수. dual 함수 g(η)=min_u L은 HJB PDE −∂_tJ = min_u[½uᵀRu+V+(f+Gu)ᵀ∂_xJ+½Tr(ΣΣᵀ∂²_xJ)], u*=−R⁻¹Gᵀ∂_xJ. Assumption 1 (ΣΣᵀ=λGR⁻¹Gᵀ)로 PDE 선형화 → Feynman-Kac로 ξ=exp(−J/λ) 표현. Path integral: u*(x,t;η)=lim_{s↓t} G(x,t)·E[exp(−S/λ)∫dw]/((s−t)E[exp(−S/λ)]). Dual ascent: η ← η + γ(P_fail(u*(·;η))−Δ).

**3. 비판적 검토.**
- 강쌍대성 결과가 Assumption 1(ΣΣᵀ=λGR⁻¹Gᵀ, 제어채널 노이즈와 비용가중치 정합)에 결정적으로 의존한다. 저자도 Remark 1에서 이 조건이 간접구동(under-actuated) 상태나 모델불확실성이 있는 동역학에 제한적임을 인정한다. 핵심 가정의 보편성이 약하다.
- Assumption 3(η↦P_fail의 연속성)은 증명 없이 conjecture로 남겨두어 Theorem 3(강쌍대성 본 정리)이 미완의 가정 위에 서 있다. 저자도 future work로 미룬다.
- exit time 변환과 강쌍대성의 핵심 기계(Dynkin 공식, Feynman-Kac, Legendre duality, nonconvex strong duality)는 모두 외부에서 import되며, 본 논문의 기여는 이를 chance-constrained SOC에 조립·적용하는 데 있다. 자체 새 정리(Theorem 1-6)는 표준 verification theorem의 확장 형태다.
- 모든 시뮬레이션이 안정한 uncontrolled 동역학을 가정한다. 불안정 시스템(발사체·재진입 등 실제 G&C 대상)으로의 확장은 명시적으로 미해결로 남았다.
- 외부 baseline이 없다. path integral 해를 자체 FDM 해와만 비교하므로(self-baseline) 정확도·우월성을 외부 기준 대비 검증하지 못한다.

**4. 시뮬/실험.** 3개 모델: 2D input velocity (2상태), unicycle(4상태), car(5상태) 모바일로봇 장애물회피 항법 (all sim). velocity 모델에서 Δ=0.1 vs 0.9로 100궤적 시각화, 10⁵ Monte Carlo, 96×96 그리드 FDM 대비. 정량: FDM 전체 (x,t) 최적입력 계산 ~10.28s, path integral은 각 시점당 0.04s(velocity)/0.48s(car), Intel i7-9750H + GTX1650. η* vs Δ, P_fail vs Δ 곡선이 path integral과 FDM에서 일치(Δ↑ → η*↓, P_fail↑). 외부 baseline 없음, 실기/HIL 없음.

**5. Contribution(delta).** 출판년(2025) 기준, 기존 continuous-time chance-constrained 연구들([11] risk contour+SOS, [12] gPC+Chebyshev)이 chance constraint를 보수적으로 근사(Boole/concentration/moment 부등식 → duality gap>0, 보수성)한 것과 달리, exit-time 지시함수 기댓값으로 **정확한**(근사·시간이산화 없는) 변환을 제시하고 nonconvex 문제임에도 **강쌍대성(zero duality gap)**을 증명한 점이 명확한 delta다. 또한 path integral dual ascent로 온라인 Monte Carlo 해법을 제공해 curse of dimensionality를 완화한다(5D까지 FDM 없이 해결). [7]의 discrete-time dual(Boole 근사로 gap≠0) 대비 연속시간 + zero-gap이 진전이다.

**6. Limitation·Future work.** (저자) Assumption 1의 제한성 제거([41] 방향), Assumption 3의 정식 증명, 알고리즘 속도 최적화, sample complexity 분석, chance-constrained stochastic game 확장. (추가) 외부 SOTA baseline 부재로 비교우위 미입증; 불안정 동역학 미지원으로 항공우주 G&C(발사체 강하·재진입) 직접 적용 불가; 코드 비공개로 재현성 제약; 실기/HIL 검증 전무; Assumption 1이 사실상 control-affine·완전구동 가정을 강제하여 적용 시스템군이 좁다.

**평가:** 73/100 (good) · 한줄평: chance constraint의 정확한 exit-time 변환과 연속시간 강쌍대성 증명이 이론적으로 탄탄한 control-theory 기여이나, 핵심 가정(Σ-R 정합)의 제약·외부 baseline 부재·모바일로봇 한정 검증으로 항공우주 G&C 관련성은 간접적이다.

---

## (2026, Automatica, Recursively Feasible Stochastic Model Predictive Control for Time-Varying Linear Systems Subject to Unbounded Disturbances: Knaup·Tsiotras)

**1. 핵심 아이디어.** 비유계(unbounded, 가우시안) 외란과 시변(LTV) 시스템 행렬을 동시에 갖는 affine 시변(ATV) 시스템에 대해, 온라인 affine-feedback 기반 확률적 MPC(SMPC)의 재귀 가능성(recursive feasibility)을 보장하는 충분조건을 최초로 제시한다. 공분산 조향(covariance steering) 기반 CS-SMPC(Knaup et al., 2023)가 미해결로 남겼던 재귀 가능성 문제를 채우는 것이 목표이며, 이로써 폐루프 chance constraint 만족까지 보장한다.

**2. 알고리즘.** 시스템은 x_{k+1}=A_k x_k+B_k u_k+D_k w_k+r_k, w_k~N(0,I) (Eq.1)이고 [A_k B_k D_k r_k]∈S=co{S^1,...,S^{Np}} (Eq.2). Theorem 3가 단말집합 X_f^μ와 단말공분산 Σ_f에 대한 재귀 가능성 충분조건을 invariance 형태로 제시(Eq.10a-d). 그 조건을 만족시키기 위해 (i) 단말공분산 Σ_f는 SDP min tr(Σ) s.t. LMI (Eq.17)로, (ii) 단말집합 X_f^μ는 robust controlled invariant set으로 계산(Theorem 5, Eq.21-23). 제어 파라미터화는 u_{t|k}=ρ_{t|k}(x_{t|k})=v_{t|k}+L_{t|k}(x_{t|k}-μ_{t|k}) (Eq.16)이며, 변수 변환을 통해 전체 문제를 볼록 프로그램(Eq.35)으로 재구성한다. 초기화 (8)/(29)/(corollary8)에 따라 보수성-성능 trade가 달라진다.

**3. 비판적 검토.**
- 핵심 기여는 새 제어기가 아니라 기존 CS-SMPC 프레임워크에 대한 재귀 가능성 "분석"과 단말제약 설계법이다. 알고리즘 자체의 새로움은 제한적이며 covariance steering·Gonzalez(2010) LMI invariance 기법의 조합·확장 성격이 강하다.
- 보수성 문제. 모든 시변 파라미터(꼭짓점 S^ℓ)에 대해 robust invariant terminal set을 요구하므로 (Fig.2에서 X_f^μ가 nominal/safe 집합의 교집합으로 크게 축소됨) 실현 가능 영역이 작아진다. 저자도 indirect feedback/state interpolation 등 보수성 완화는 future work로 미룬다.
- Assumption 1(예측 지평 전체에서 시스템 행렬을 사전에 안다)은 reference 추종형 ATV에는 합리적이나, 일반 LPV/비선형 선형화 상황에서 미래 곡률·속도 프로파일을 정확히 안다는 가정은 실제 적용 범위를 제약한다.
- 검증이 단일 예제(차량 횡방향 kinematic bicycle)에 국한되고, 외부(타 방법) baseline 없이 자체 CS-SMPC의 naive 변형 및 초기화 방식 비교에 그친다.

**4. 시뮬/실험.** 비선형 kinematic bicycle 모델을 reference 궤적 따라 선형화한 ATV 시스템(state x=[δ,e_ψ,e_y], horizon N=4, T=50), 20회 Monte Carlo (sim). p_{x,i}=0.025, p_{u,j}=0.05, Q=diag(1,1,1), R=100. 제안법(robust X_f^μ,Σ_f)은 모든 trial에서 제약을 위반하지 않고 재귀 가능성을 유지; nominal 단말제약을 쓴 CS-SMPC는 infeasibility 발생(Fig.3 녹색 십자). 초기화별 평균 총비용: (8) 343 / PRS 261 / (29) 228 (std 35.9/34.2/17.3) — 덜 보수적인 (29)가 비용 최저(Table 2). 정량 baseline은 모두 내부(self) 비교.

**5. Contribution(delta).** 출판년(2026) 기준, Table 1이 보여주듯 "비유계 외란 + 시변 시스템 + 온라인 affine-feedback"이 동시에 충족되는 재귀 가능성 결과는 사실상 부재했다(Farina 2013은 LTI+open-loop init, Gonzalez 2011·Hanema 2017·Lima 2017은 유계 외란). 본 논문은 이 빈칸을 메우는 첫 결과로서, (i) ATV-SMPC 재귀 가능성 충분조건(Theorem 3), (ii) SDP/robust invariant set으로 단말제약을 실제 계산하는 방법(Lemma 4·Theorem 5), (iii) 폐루프 chance constraint 만족 증명(Theorem 7·Corollary 8), (iv) 이를 볼록 프로그램(Eq.35)으로 통합해 CS-SMPC의 재귀 가능성까지 확립한 것이 명확한 delta다.

**6. Limitation·Future work.** 저자: 모든 시변 파라미터에 대한 robust 단말집합 요구로 인한 보수성, 단일 횡제어 예제. 완화책으로 indirect feedback(Hewing 2020)·state interpolation(Köhler&Zeilinger 2022)을 언급. 내가 보는 한계: (a) 외부 SOTA 방법과의 정량 비교 부재, (b) 실시간성을 강조하나 SDP/볼록 프로그램의 실측 solve time·온보드 복잡도 수치 미제시, (c) Assumption 1의 미래 파라미터 완전 인지 가정, (d) 가우시안 외란 가정에 특화되어 heavy-tailed/비대칭 분포로의 확장 미검토, (e) 코드 비공개.

**평가:** 82/100 (strong) · 한줄평: 비유계 외란+시변 시스템 SMPC의 재귀 가능성이라는 실제 공백을 엄밀한 자체 증명과 볼록·SDP 설계로 처음 메운 견실한 control-theory 기여로, covariance steering·자율주행 횡제어 계보를 통해 G&C 관련성도 충분하나 검증은 단일 예제·내부 baseline에 머문다.

---

## (2026, JGCD, Gaussian-Process-Based Covariance Control for Autonomous On-Orbit Servicing: Garzelli et al.)

**1. 핵심 아이디어.** 자율 궤도상 서비싱(RPO) 도킹 기동에서 차등 항력(differential drag)에 의한 불확실 외란을 가우시안 프로세스 회귀(GPR)로 데이터 기반 모델링하고, 이를 확률적(공분산) 최적제어 프레임워크에 통합한다. 비볼록 확률 제어 문제를 successive convexification으로 일련의 볼록(SDP) 문제로 변환하여 내부점법으로 효율적으로 푼다. 결과는 평균 궤적과 종단 공분산을 목표 분포로 동시에 조향하는 선형 상태 피드백 제어법이다.

**2. 알고리즘.** 상대 운동은 HCW 방정식, 외란은 차등 항력 가속도 a_p를 입력 z=[x_dot, y_dot]^T의 함수로 두고 이분산(heteroscedastic) GP d(z)~N(mu^(d),Sigma^(d))로 학습(Kersting et al. most-likely 방식). 동역학 x_dot = A x + B u + c + G_d d(z). 피드백 u_j = u_bar_j + K_j(x_j - x_bar_j). 공분산 전파는 변수치환 Y_j=K_j Σ_{x,j}, H_j=K_j Σ_{x,j} K_j^T로 선형 등식화하고, Y_j Σ^{-1} Y_j^T ≤ H_j를 Schur 보수로 LMI [[H_j,Y_j],[Y_j^T,Σ_{x,j}]]⪰0로 무손실 완화. GP 사후분포의 모멘트 전파는 Girard의 1차 Taylor 전개로 근사. 제어 노름 chance constraint는 chi-square 분위수 gamma(p)를 이용해 ||u_bar_j|| + gamma(p)√ρ(K Σ K^T) ≤ u_max로 결정론화. 접근 콘은 평균 상태에만 부과. 전체는 SDP를 SCP로 반복 수렴.

**3. 비판적 검토.**
- 핵심 수학 기구(공분산 조향 무손실 완화, trace 최소화 무손실성, GP Taylor 전개)가 대부분 인용/수입(Refs 47, 59, 60)이며 본 논문 고유의 증명·수렴 보장은 없다. SCP 수렴은 경험적 정지 기준에 의존.
- 베이스라인이 모두 저자 자작(open-loop, 상수 G_d 공분산 제어)이며 외부 SOTA 방법과의 비교가 없다. 단일 도킹 시나리오·순수 시뮬레이션.
- 접근 콘 제약을 평균 상태에만 부과하여 위치 분포 전체의 콘 내 포함을 보장하지 못함(저자도 인정). chance constraint는 보수적 충분조건이라 과보수성 존재.
- B_T=0(타깃 무항력), 상수 대기밀도, 면내/면외 분리, 속도-only GP 입력 등 단순화가 케이스 의존적. 일반성은 주장하나 실증은 좁은 범위.

**4. 시뮬/실험.** LEO 200 km, t_f=300 s, N=601 노드, 알파=15deg 콘, u_max=1e-2 m/s^2, 도킹 종단거리 0.5 m. 세 제어기(OL/상수-G_d CC/GPR-CC) 각 1000회 Monte Carlo (sim). 종단 위치 분산: OL은 최대 ~60 m로 임계 위반, GPR-CC는 σ_x≈9.06e-3 m, σ_y≈1.02e-2 m로 목표 한계 내. ΔV(J): OL 0.69, CC 1.86, GPR-CC 3.11 m/s — GPR-CC만 종단 분산 요건 충족(제어비용 최대). B에 +20% 바이어스 model-mismatch 시 위치 σ_xy 1.76e-2 m, 목표 초과 7.64 mm로 견고성 유지. 해 시간 ~3 s(3 iter, GP 오버헤드 7%/iter, MOSEK/MATLAB, Ryzen 7). 무손실/선형화 검증오차 1e-4~1e-8.

**5. Contribution(delta).** 데이터 기반 이분산 GP 외란 모델을 볼록 공분산 제어와 결합하여 RPO 도킹에 적용한 점이 delta. 출판년(2026) SOTA 기준으로 공분산 조향(Chen-Georgiou, Bakolas)·SCP 기반 공분산 제어(Benedikter/Zavoli)·GP-MPC(Hewing)는 이미 확립된 도구이며, 본 논문은 이들을 차등 항력 RPO 문제에 통합한 합리적이나 점진적 진전이다(새 방향 개시는 아님). 백색 가우시안 가정의 보수성을 데이터 기반 상태의존 외란 통계로 완화한 점이 실질적 기여.

**6. Limitation·Future work.** 저자: 접근 콘 평균-only 제약, chance constraint 보수성, 고정 하이퍼파라미터(온라인 적응 미구현), 단일 불확실 파라미터(B_C만). 내가 보는 한계: 외부 베이스라인 부재·단일 시나리오로 일반화 주장 미검증, SCP 수렴 보장 부재, GPR-CC가 ΔV를 크게 증가(0.69→3.11)시키는 비용-견고성 trade가 정량 분석되지 않음, 비선형 동역학·고밀도 변동 확장은 언급만. Future: 온라인 적응 GP(SVI/streaming sparse GP), 위치(고도) 입력 추가, successive linearization 기반 확률적 콘 제약.

**평가:** 72/100 (good) · 한줄평: 확립된 공분산 조향과 이분산 GP를 RPO 도킹에 견실히 통합한 in-domain 항공우주 G&C 논문으로, 검증은 단일 시나리오·자작 베이스라인에 그치나 핵심 G&C 문제(궤도상 서비싱)에 직접 닿는 실용적 기여다.

---

## (2026, arXiv, Free Final Time Adaptive Mesh Covariance Steering via Sequential Convex Programming: Pilipovsky)

**1. 핵심 아이디어.** 비선형 SDE(가산+곱셈 잡음)에 대한 자유 종단시간(free-final-time) 공분산 조향(covariance steering)을 SCvx* 기반 순차 볼록 계획법(SCP)으로 푼다. 핵심은 구간별 시간팽창(time-dilation) 변수 σ_k를 도입해 평균 궤적·제어 정책과 시간 격자(mesh)를 동시 최적화하되, 곱셈 잡음에서는 시간팽창이 drift뿐 아니라 diffusion(√σ)까지 스케일링하므로 확산 항을 함께 1차 선형화하는 "exact diffusion linearization"을 유도한 점이다. 이를 통해 SCP 부분문제 안에서 공분산 전파 정확도를 frozen-diffusion 방식보다 한 차수 개선한다.

**2. 알고리즘.** τ∈[0,1]로 시간정규화: dx_τ = σ_τ f dτ + √σ_τ g dw_τ, t_f = ∫σ_τ dτ. 비용 J = η t_f + J_reg(평균/공분산 정규화). diffusion까지 포함한 1차 Itô 선형화 (11)로 augmented 제어입력 ū_τ=[u;σ]을 두고, ZOH+frozen-state 가정 하에 이산 국소 확률모델 x_{k+1}≈A_k x_k + ν_k + H̄_k(x_k,ū_k)Δw_k (24)를 구성. 한걸음 평균제곱오차 한계(Thm1): freezing 오차 O(Δτ²), projection 오차 O(Δτ³)이며 projection은 H_k(τ)의 시간평균 H̄_k이 조건부 MSE 최소화 해임(Lemma1, 직교투영)을 증명. 모멘트 전파(Thm2, 27)와 affine 피드백 ū_k=v̄_k+K̃_k(x_k−x̄_k) 대입(29). 공분산-피드백의 쌍선형성은 U_k:=K̃_kΣ_{x_k} 치환 + Schur보수 LMI 완화(36)-(40)로 볼록화하고, 이 완화가 J_reg의 단조성 조건(71) 하에서 lossless임을 Thm3로 증명. 분포강건(Chebyshev) chance constraint를 Boole 부등식 + risk allocation으로 처리(33). SCvx* trust-region/augmented-Lagrangian로 수렴(Alg1).

**3. 비판적 검토.**
- 검증이 단일 사례(2D drag+속도의존 diffusion double-integrator)에 그친다. 시나리오 다양성·실제 G&C 문제(동력하강, 저추력 전이 등)에서의 검증이 없어 일반화 주장이 경험적으로 약하다.
- baseline이 외부 방법이 아니라 저자 자신/선행의 "frozen-diffusion linearization"뿐이다. 95% 공분산 정확도 개선은 method-내부 비교이며, 외부 SOTA 대비 정량 비교는 없다.
- 실시간성/온보드 주장 근거가 약하다. cvxpy+MOSEK로 SDP를 62회 반복 푸는 구조이고 per-iteration 타이밍·복잡도(LMI 크기 d개 채널×N단계)·계산시간 보고가 전혀 없다. SDP 기반 곱셈잡음 CS는 일반적으로 무겁다.
- frozen-state 가정(16)과 국소 정칙성 가정(Assumption2) 하의 오차 한계는 엄밀하나, 곱셈잡음으로 상태분포가 비가우시안이 됨에도 chance constraint를 1·2차 모멘트 기반 Chebyshev surrogate로만 처리해 보수성이 정량화되지 않았다.
- novelty의 핵심 delta(time-dilation–diffusion 결합 선형화)는 개념적으로 √σ_τ 계수의 자명한 미분에서 출발하며, 새로움은 "기존이 이를 freeze했다"는 대비에 기반한다. 기여는 명확하나 방법론적 도약 폭은 중간 수준이다.

**4. 시뮬/실험.** 규모: N=30, Δτ=0.02, 상태 4D 미만(2D pos/vel), 제어 1D. (a) 가산잡음(g0=0.2,g1=0): η를 0~10으로 sweep, t_f가 1.60→0.99로 단조감소(Table II), SCvx* 62회 수렴, MC(N_mc=1000, Milstein n_sub=10) 종단위치분산 σ_{p,N}^mc=0.135 < √0.15, worst-case 입력risk 0.078<0.1 만족 (sim). (b) 곱셈잡음(g1=1): 제안 full-diffusion vs frozen-diffusion 비교, 16/33회 수렴; 종단 공분산오차 σ_{p,N}^full,mc−σ_{p,f}=−0.002 vs frozen=0.04 → 공분산전파 정확도 95% 개선 주장 (sim). 외부 baseline 없음, HIL/real 없음.

**5. Contribution(delta).** 출판년(2026) 기준 SOTA 대비: 비선형 SDE 곱셈잡음에 대해 (i) 시간격자(mesh)와 CS 정책을 동시 최적화하는 free-final-time iCS 정식화는 저자 주장상 최초이며, 선행 adaptive-mesh SCP(결정론, [17]-[19])와 frozen-diffusion iCS([12]-[14],[10])를 결합·확장한다. (ii) 시간팽창 의존성을 포함한 exact 1차 diffusion 선형화 + 한걸음 MSE 한계(O(Δτ²) freezing이 지배)와 projection 최적성(Lemma1)이 핵심 이론 delta. (iii) 곱셈잡음 CS의 쌍선형성에 대한 lossless LMI 완화(Thm3)와 DR chance constraint 통합. 선행 frozen-diffusion이 reference에서 diffusion을 동결(O(Δτ) 오차)한 데 비해 한 차수 정확.

**6. Limitation·Future work.** 저자: 고차원 guidance 문제 확장, stopping-time/first-reach 사양으로의 확장을 future work로 명시. 내가 보는 한계: (1) 검증이 toy 2D 단일사례·자기baseline에 국한되어 TAES/TASE 수준의 도메인 검증 부족, (2) 계산시간/온보드 실현가능성 분석 전무(SDP×SCP는 무거움), (3) 비가우시안성에도 2차모멘트 chance constraint만 사용해 보수성 미정량, (4) frozen-state 가정의 유효 범위(큰 Δτ, 강한 곱셈잡음)에서의 정확도 저하 미검토. core-gnc 응용(동력하강 등)으로의 실증이 영향력 확대의 관건.

**평가:** 74/100 (good) · 한줄평: time-dilation과 diffusion 선형화를 결합해 곱셈잡음 free-final-time 공분산조향을 엄밀히 정식화한 이론적으로 탄탄한 control-theory 논문이나, 단일 toy 검증·자기baseline·온보드 분석 부재로 G&C 실증 영향력은 아직 잠재적 — 도메인은 G&C 인접(near-domain).

---

## (2026, arXiv, Statistical Contraction for Chance-Constrained Trajectory Optimization of Non-Gaussian Stochastic Systems: D'Silva·Tsukamoto)

**1. 핵심 아이디어.** 이산시간 비선형·비가우시안 확률 시스템에 대한 분포-자유(distribution-free) 강건 궤적 최적화·제어 프레임워크를 제안한다. 컨포멀 추론(conformal prediction, CP)을 수축 이론(contraction theory)과 결합하여, 학습된 수축 메트릭/추종 제어기 주변 폐루프 동역학에 대해 유한 표본 기반의 커버리지 보장 신뢰집합을 구성한다. 핵심은 (i) 증분 안정성(수축 조건) 위반과 (ii) 외란이 폐루프에 미치는 영향을 동시에 정량화하는 '결합 비적합성 점수(joint nonconformity score)'를 정의하여, 강한 분포·구조 가정 없이 chance constraint를 통계적으로 타당한 결정론적 제약으로 강화(tightening)하는 것이다.

**2. 알고리즘.** 폐루프 동역학 x_{k+1}=φ_π(x_k,k)+D(x_k)w_k. 수축 메트릭 M_k=Θ_k^T Θ_k에 대해 Riemann 에너지 E가 비율 λ로 지수 수축(Lemma 1). 결합 비적합성 점수 S_k^(j)=Σ_{i=0}^{k-1} λ^i (Δ_V(x_i,x̄_i,ū_i)+||Θ̂_{i+1}D(x_i)w_i||) where Δ_V는 수축 감쇠 조건의 잔차. Weighted CP(Lemma 2)로 분포 이동(distribution shift)에 강건한 분위수 C_k=q_{1-δ+δ̄}(S_k)를 추론. Theorem 1: B^{1-δ}(x̄_k,W_k), W_k=C_k^2 M̄_k^{-1} 가 신뢰집합. Theorem 2: 강화된 결정론적 문제(Problem 2)의 해가 원래 chance-constrained 문제(Problem 1)의 실행가능 해. 다면체 제약은 Boole 부등식으로 위험 배분 후 robust counterpart로, 장애물 회피는 signed distance d_i로 변환. 최종 정책 u_k=π̂(x_k,x̄_k^*,ū_k^*).

**3. 비판적 검토.**
- 핵심 통계 기계(Weighted CP, Lemma 2)는 [17]에서, 수축 안정성(Lemma 1)은 [32,38]에서 가져온 것으로, 본 논문의 독자적 증명은 결합 점수 설계와 Theorem 1/2의 합성 부분에 한정된다. 엄밀하나 '완전 자체 증명'은 아니다.
- 보장이 커버리지 갭 상한 δ̄에 의존하나, 저자도 인정하듯 δ̄는 a priori로 알 수 없는 a posteriori 양이며, 실험에서는 분포 이동이 '미미했다'는 경험적 근거로 vanilla CP(δ=p)를 사용한다. 즉 형식적 보장과 실제 적용 사이에 간극이 존재한다.
- 수축 메트릭/제어기의 품질이 나쁘면 보장은 성립하나 신뢰집합이 매우 보수적/큰 W_k로 귀결될 수 있으며, 보장의 '비공허성(non-vacuity)'에 대한 분석은 없다.
- MC 규모(200 realization)와 하드웨어 실험(단일 cluttered arena, 15회)은 개념 검증 수준이며, 통계적 유의성(신뢰구간, 반복 시드)에 대한 정량 보고가 약하다.

**4. 시뮬/실험.** (1) Dubins car (4-state) 수치 시뮬: 균일 노이즈와 3성분 가우시안 혼합 노이즈 두 경우, p=δ=0.1, N=200 step, K=20 외란 표본. Problem 2 평균 풀이 1.67s, 폐루프 200회 realization. 경험적 실패확률 max{Pr(x∉X)}= 0% (case i), 1.5% (case ii) (sim) — 목표 10% 이내 만족. 외부 baseline: 선형화+LQR 가우시안 근사 [7,8]은 실패확률 10%(case i)/20.5%(case ii)로 보장 위반(꼬리·비선형성에 취약). (2) Crazyflie 2.1 쿼드콥터 하드웨어 (HIL/real): p=δ=0.1, 7.5s, Qualisys 모캡, 온보드 PID 위에 π̂ 배치, 15회 반복 모두 신뢰집합 내 유지 (real).

**5. Contribution(delta).** 출판년 기준, CP의 분포-자유 개념을 수축 기반 강건 운동계획에 통합하여 '비가우시안·비선형 폐루프'에 대한 비발산(non-diverging)·유한표본 chance-constraint 보장을 제공한 점이 delta. 선행 대비: [30](극값이론, 유계 노이즈만), [31](확률 수축 MPC, 노이즈 공분산 norm 상한 필요) 등 관련 시도가 제한적 가정 하에서 작동한 데 비해, 본 방법은 노이즈 분포·메트릭 구성 방식에 무관(agnostic)하며 학습 기반 수축 메트릭/제어기의 성능을 형식적으로 인증하는 경로를 연다. 표본 수에 따라 문제 크기가 커지지 않는 점도 scenario approach 대비 장점.

**6. Limitation·Future work.** 저자: 커버리지 갭 δ̄의 a posteriori 특성(적응형 CP[44]로 온라인 갱신 필요), 폐루프 제어 제약은 미고려(Corollary 1로 부분 보장), receding-horizon 확장은 future work. 추가로 보는 한계: 자체 증명 부분이 제한적이고, 보장의 보수성/비공허성 정량화 부재, 검증 규모가 작고 통계적 처리가 약함, 항공우주(launch/entry/descent) 도메인 직접 검증은 없음(로봇/드론 플랫폼).

**평가:** 81/100 (strong) · 한줄평: CP와 수축 이론을 결합해 비가우시안 폐루프에 분포-자유 chance-constraint 보장을 주는 이론적으로 견고하고 재현 가능한 작업이나, 검증이 Dubins car·Crazyflie에 한정되어 항공우주 G&C 관련성은 방법 전이(transfer) 수준으로 간접적이다.

---

## (2026, arXiv, Stochastic Model Predictive Control with Online Risk Allocation and Feedback Gain Selection: Barbosa·Löfberg)

**1. 핵심 아이디어.** 결합형 chance-constrained SMPC에서 위험 할당(risk allocation)과 피드백 정책을 동시에 최적화하면 (i) 결정론적 등가식에서 피드백 법칙과 위험 할당의 곱, (ii) 비볼록 Gaussian quantile(probit) 함수 합성 때문에 비볼록·난해 문제가 된다. 본 논문은 피드백 게인을 사전 계산된 후보 집합에서 이진 지시 변수로 선택(disjunctive)하고, probit 합성을 power-cone/exponential-cone 표현 가능 근사로 치환하여 전체를 혼합정수 원뿔(conic) 최적화로 정식화한다. 이로써 off-the-shelf solver(MOSEK)로 효율적으로 풀 수 있고, 배타적 disjunctive·Gaussian 변수 곱을 포함하는 일반 chance constraint로 확장된다.

**2. 알고리즘.** affine disturbance feedback U=MW+V로 폐루프 정식화. 게인 선택은 L=Σ δ_k L_k (Σδ_k=1, δ_k∈{0,1}), M(δ)=Σ δ_k L(L_k)(I−BL(L_k))^{-1}G. 개별 chance constraint의 결정론적 등가는 f(V)+||c(δ)||Φ^{-1}(1−γ)≤0. 이진성을 이용해 ||c(δ)||=||g_0+Σδ_k g_k||, r_k=||g_0+g_k||로 환원하고 Σδ_k r_k Φ^{-1}(1−γ)≤−f(V)를 세 가지로 변환: Inverse probit(rotated SOC + 1/Φ^{-1} 근사 Ψ^inv), Root probit(rotated SOC + √Φ^{-1} 근사 Ψ^root), Log probit(exp-cone + log Φ^{-1} 근사 Ψ^log). 비초등 함수 1/Φ^{-1}, √Φ^{-1}, logΦ^{-1}는 분수 거듭제곱(power cone)과 Lambert-W(product log, exp-cone)로 보장된 내/외부 원뿔 근사 Ψ=βα^... + 보정항으로 최소제곱 적합. 장애물 회피는 Big-M으로 disjunction 인코딩.

**3. 비판적 검토.**
- 핵심 원뿔 근사 기법(probit의 nonsymmetric cone 근사)은 선행 [22](Barbosa·Löfberg 2025)에서 그대로 가져온 것이며, 본 논문의 신규성은 그 위에 게인 선택(disjunctive/MIO)을 얹은 확장이다. 이론적 토대의 상당 부분이 import.
- recursive feasibility를 다루지 않았다고 저자가 명시. SMPC에서 재귀 실현 가능성은 핵심 안전 보장인데 미해결로 남아, 폐루프 안정성·실현 가능성 보증이 없다.
- 게인을 "사전 계산된 유한 후보"에서만 고른다 — 연속 최적화의 보수성을 회피하지만 후보 격자 품질에 성능이 좌우된다. 수치 예에서 N_F=125개 LQR 격자를 썼으나 후보 설계 지침은 빈약.
- 근사는 보수적이며 ξ(전체 위험)이 커질수록 보수성이 급격히 악화(저자 인정). 저위험 영역(γ 작음)에서만 유효한 국소 근사다.
- 검증이 단일 2D 적분기형 경로계획 toy에 한정. 외부 baseline 없이 자기 변형(ES vs inv/root/log)끼리만 비교.

**4. 시뮬/실험.** 4상태 선형 적분기 차량의 2D 경로계획. Case 1(장애물 없음)·Case 2(장애물 있음), N=10, N_F=125(N_L=5), ξ=0.15. 상태 예측 포락선은 각 1000회 Monte Carlo(sim)로 검증, 최선/최악 정수해(U^min/U^max) 비교로 게인 선택의 효과 제시. solver time(MOSEK): Case 1 평균 inv 0.31s/root 0.73s/log 0.38s vs ES 1.40s; Case 2 평균 inv 0.35s/root 9.20s/log 4.90s vs ES 224.0s — 제안법이 exhaustive search보다 1~3자리수 빠름(특히 inv). baseline은 모두 내부(ES=자기 exhaustive, [22] 근사 기반). 실기·HIL 없음.

**5. Contribution(delta).** 선행 [22]는 open-loop SMPC(피드백 없음)에 probit 원뿔 근사를 적용. 본 논문의 delta는 (1) 피드백 게인 선택을 disjunctive 이진 변수로 모델링해 위험 할당과 동시 최적화하는 첫 정식화, (2) 두 단계(bilevel) 위험-피드백 최적화의 수렴 미보장 문제를 회피하고 단일 MIO/B&B로 전역해 탐색, (3) 배타적 disjunctive 변수×Gaussian 변수 곱을 포함하는 일반 chance constraint(예: stay-out 장애물)로 확장. 출판년(2026) 기준으로는 SMPC 내 점진적·실용적 진전이며, 분야를 새로 여는 수준은 아니다.

**6. Limitation·Future work.** 저자: recursive feasibility 미해결, 큰 ξ에서 근사 보수성 악화, Gaussian mixture 확장이 향후 과제. 내가 보는 한계: (a) 검증이 단일 선형 toy로 G&C(진입/하강/착륙 등 비선형·고차원) 실증과 거리, (b) 후보 게인 격자 설계의 체계적 방법 부재로 성능·보수성이 격자 선택에 민감, (c) Case 2 root 근사 평균 9.2s 등 초 단위 풀이는 빠른 동역학의 onboard 실시간엔 부담, (d) 외부 SOTA(예: covariance steering 기반 SMPC, 샘플링 근사)와의 정량 비교 부재.

**평가:** 70/100 (good) · 한줄평: 위험 할당과 피드백 게인 선택을 단일 혼합정수 원뿔 문제로 통합한 견실한 SMPC 정식화이나, 검증이 2D toy·내부 baseline에 머물고 recursive feasibility가 미해결이라 G&C 적용성은 잠재적 수준의 control-theory 근접 도메인 기여다.
