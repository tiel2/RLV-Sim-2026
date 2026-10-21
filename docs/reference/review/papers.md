# RLV — Literature (thematic map · scoring rubric · scores)

> RLV multi-phase return GNC. Drone-era literature archived on the `legacy_drone` branch.

---

## 1. Thematic map (threads A–G)

> **논문별 점수·기준 → §3 ledger** (177편 보유 PDF) · **상세 비판 → [detailed_reviews/](detailed_reviews/) A–H.md**.
> 이 지도는 §3의 평면 표가 담지 못하는 **스레드 성격**만 남긴 것(2026-06-06 6-thread 병렬 탐색 결과를 접음). 식별자/DOI는 각 review 또는 §3의 venue·연도 참조. 전 177편 PDF는 [../](../) 보유(`docs/reference/*.pdf`, gitignored).

- **A. Convex / SCvx / LCvx backbone** — *결정론 backbone, saturated.* 정전: Ploen-2007(LCvx 창시, 90)·ct-SCvx(87)·Malyuta tutorial(85). backbone 후보 = ct-SCvx / SeCO(multi-phase) / Auto-SCvx.
- **B. RLV landing & reentry GNC** — *응용 도메인, 2024–26 saturated; transition-under-uncertainty gap 생존.* 대표: **intelligent-phase-transition**(AST 2026 — RL이 전환점 결정), **explicit-dispersion**(JGCD 2026 — gPC online dispersion, 단일-phase), **Ignition-reachability**(결정론 aero→점화 reachable set).
- **C. Covariance steering · chance · risk** — *단일-phase 조각은 saturated; cross-switch 합성은 open.* 대표: **Hybrid-Transitions-CS**(2410.13222 — guard 가로지른 CS·timing 포함·chance/trigger/GP 없음), **MJLS-CCCS**(외생 Markov 전환, 미최적화), **Free-Final-Time-CS**. IRA-CS(within-phase IRA), Knaup-RF-SMPC(unbounded recursive feasibility), Strong-Duality(exit-time/first-passage 형식론), Statistical-Contraction(non-Gaussian tail).
- **D. Hybrid · saltation · STC** — *mechanism layer. saltation+STC는 fixed guard엔 성숙; guard를 chance 하에 최적화가 niche.* 정전: Saltation-survey·Salted-KF(guard 통과 Σ)·Uncertainty-SKF(guard θ 불확실성→Σ). **Uzun**(CT compound-STC). 결정론 원조: Boccadoro(switching-surface 최적화)·Abate(P≥1−ε stochastic-DP)·Katayama(guard-saltation chance backoff).
- **E. GP / conformal / learning** — *function=GP, process=stochastic, one controller. GP-residual→Σ→chance는 saturated; 폐루프 shift 하 sparse-transition calibration은 open.* 핵심: GP-MPC survey·Lederer/Capone(GP Σ→chance 합법화)·NexCP/ACI/Conformal-PID(재보정, 단 보증이 marginal/장기평균)·Multihierarchy-GP(aero-residual GP 템플릿)·Neural-Fly(외란 모델). 도메인=quadrotor/통계.
- **F. Real-time embedded solvers · tracking MPC** — *guidance solver + RTI-SQP tracker. embedded conic saturated; anytime/early-termination thinner.* 핵심: QOCO·Clarabel·KAIST-IPM(codegen SOCP/IPM)·Conic-TinyMPC(MCU 실HW). PIPG(Automatica 2022)·acados(MPC 2022)·DAQP(TAC 2022). **SDP(Σ-steering) onboard solver는 공백** — 6 solver 전부 SOCP까지.
- **G. Disturbance models (wind · turbulence)** — *mean wind=OU/Gauss-Markov, turbulence=Dryden.* 보유: Dragonfly-gust(AR(1)/OU+Dryden 사양 템플릿, 지구 MIL-STD 재수치화). 표준/미채점: Dryden·MIL-STD-1797(standard), Pham OU-mean-wind(J. Aircraft 2025).

---

## 2. Scoring rubric (v1.3)

A common, reusable rubric so papers are comparable on one scale. Three pillars — **Rigor, Novelty, Impact** — all scored. Calibration anchor = strong field venues (IEEE TAES/TASE/RA-L, AST, JGCD, Automatica). Impact = significance (problem importance + influence), citation count a secondary signal only. Scores are **relative across the corpus** and may be re-calibrated corpus-wide (note reason + version in the ledger).

**Criteria (each 1–5, weighted):**

| # | Criterion | Pillar | Wt | Measures |
|---|---|---|---|---|
| C1 | Theoretical Rigor & Correctness | Rigor | 17% | Formal statement; assumptions justified; complete proofs; no claimed-vs-implemented gap; (S)MPC: recursive feasibility/stability/constraint guarantees |
| C2 | Novelty & Originality | Novelty | 15% | New theory/method vs incremental; clear delta from prior art |
| C3 | Methodological Soundness | Rigor | 16% | Does the method do what it claims? Modeling choices appropriate, not self-defeating |
| C4 | Uncertainty / Risk Modeling Fidelity | Rigor | 9% | Aleatoric vs epistemic clarity, distributional assumptions, tail handling, correlation, transition model |
| C5 | Empirical Validation & Baselines | Rigor | 14% | Head-to-head vs competitors; statistical rigor (MC, CIs); scenario realism; tests the core claim |
| C6 | Reproducibility | Rigor | 8% | Hyperparameters; code/data; ablations; reimplementable |
| C7 | Practical Impact & Deployment | Impact | 9% | Real-time / onboard feasibility (solve time vs flight-computer budget); implementation realism (3-DoF→6-DoF, MC dispersion); engineering readiness for flight software. **HIL / flight-HW tests are a bonus, NOT expected — RLV/GNC is sim-dominated, so sim-only is not a demerit.** |
| C8 | Significance & Scientific Influence | Impact | 12% | Problem importance; who benefits; field influence/adoption |

Pillar split: Rigor 64% · Novelty 15% · Impact 21%.

**Out-of-scope criterion → N/A + renormalize, NOT 1 (v1.3).** If a criterion is outside a paper's scope, mark it **N/A** and renormalize the remaining weights to 100% — do NOT score it 1. Judge a paper on what it set out to do, not for being a different *kind* of paper. Two recurring cases in this corpus: **C4** (uncertainty fidelity) for a *deterministic-by-design* backbone/solver paper (SCvx, LCvx, embedded QP); **C7** HW realism for a theory/sim paper. (C4 is still scored normally for any paper that *claims* to handle uncertainty — e.g. all SMPC / chance-constraint / covariance-steering work.) List the N/A criteria when reporting a score.

**Anchors (1–5):** 5 flagship-tier · 4 strong (solid accept at RA-L/AST+) · 3 adequate, notable gaps · 2 weak · 1 inadequate.
**Total** = (Σ score×weight) on 1–5, then ×20 → **/100**.

**Verdict bands (/100), at the rigor anchor (what a TAES/TASE reviewer would say — NOT where it actually publishes):**

| Band | Label | Verdict at flagship |
|---|---|---|
| 85–100 | Flagship | accept (often as-is) |
| 75–84 | Strong | accept / minor revision |
| 65–74 | Good | minor–major revision |
| 55–64 | Fair | major revision |
| 45–54 | Weak | reject & resubmit |
| <45 | Inadequate | reject |

**Venue prestige ladder (SJR 2024, objective prior — SEPARATE from paper quality score):**

| Tier | SJR | Venues |
|---|---|---|
| 1 flagship | ≳2.3 | IEEE T-AC 3.80 · Automatica 3.05 · IEEE T-RO 2.97 · IJRR 2.33 |
| 2 very strong | ~1.65–2.0 | IEEE TASE 1.92 · IEEE T-CST 1.68 |
| 3 strong | ~1.45–1.60 | AST 1.57 · IEEE TAES 1.53 · IEEE RA-L 1.48 |
| 4 solid Q1 | ~1.05–1.30 | Acta Astronautica 1.28 · JGCD 1.22 · IJRNC 1.09 |
| 5 respectable Q1 | ~0.70–0.85 | IEEE Access 0.85 (megajournal) · J. Spacecraft & Rockets 0.82 · EJC 0.81 · J. Aircraft 0.73 · Advances in Space Research 0.70 |
| 6 Q2 specialist | ~0.40–0.55 | CEAS Space Journal 0.51 · IJASS 0.43 · Aerospace (MDPI, Q2) |
| Conf | — | AIAA SciTech / AAS (GNC home) · RSS (≈T2–T3) · ICRA/IROS (≈T3–T4) · CDC/ACC (≈T4) |

**GNC prestige correction:** AIAA journals (esp. JGCD) are structurally penalized by SJR/IF (small GNC field; most GNC work goes to AIAA/AAS *conferences*). JGCD is in fact a **top specialized GNC archival journal**, above Acta Astronautica. GNC-prestige aerospace order: **AST ≳ TAES ≈ JGCD > Acta Astronautica > Adv. Space Research.**

*v1.2 (2026-06-06): RLV 관련 저널만 ladder에 추가 — 기존 tier·SJR 값은 그대로. 추가 검증 SJR 2024 (SCImago): IJRNC 1.09 · IEEE Access 0.85 · J. Spacecraft & Rockets 0.82 · EJC 0.81 · J. Aircraft 0.73 · CEAS Space J 0.51 · IJASS 0.43.*

---


## 3. Scores ledger (153편 · v1.3-strict 재교정 2026-06-06 · G-FOLD 2017 + multi-phase RLV 20편 2026-06-09)

> **점수 = 논문 본질 품질만** (/100, corpus 상대평가·절대값 아님). 전 16배치를 **단일 엄격 앵커**로 재교정: 4=진짜 강함, 3=보통, 평균 논문 ~60대; 임의 fudge·venue-prestige offset 전부 제거(예: 2007 Ploen은 배치가 89.7→80으로 누른 것을 공식값 90으로 복원). 상세 비판분석 → [detailed_reviews/](detailed_reviews/) A–H. **점수 산식은 여기(ledger)에만 두고 리뷰 본문엔 prose+한줄 verdict만 둔다.**
>
> **품질 밴드와 venue Tier는 별개 축이다 — 섞지 않는다.** 밴드는 *논문* 품질, Tier는 *저널* SJR 명성 사전값. 느슨히 상관할 뿐(관찰하되 강제 금지; "점수로 venue 서열화"·"명성으로 점수 보정"은 루브릭 위반). Tier(§2 ladder): T1 ≳2.0 · T2 ~1.5–2.0 · T3 ~1.0–1.5 · T4 ~0.8–1.0 · T5 ~0.6–0.8 · Conf=학회 · arXiv=preprint. **GNC-prestige 보정:** JGCD·TAES는 SJR상 T3이나 GNC 분야선 AST와 함께 최상위 전문지(raw SJR보다 높게 읽음).
>
> 밴드: **flagship 85+ · strong 75–84 · good 65–74 · fair 55–64 · weak 45–54 · poor <45**. 기준 C1 rigor·C2 novelty·C3 soundness·C4 risk-fidelity·C5 empirical·C6 repro·C7 onboard·C8 significance (가중 17·15·16·9·14·8·9·12%; **–** = N/A 재정규화). drone-era 점수는 `legacy_drone` 보관.

### A. Convex / SCvx / LCvx backbone (19)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Açıkmeşe·Ploen, Convex Programming Approach to Powered Descent Guidance for Mars Landing, **JGCD 2007** | 5·5·4.5·3·3·4·4·5 | **86** | flagship | T3 | LCvx 창시·SOCP 무손실완화 시초, 분야 출발점. |
| Açıkmeşe·Carson·Blackmore, Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem, **TCST 2013** | 5·4.5·4.5·3·2.5·4·4.5·4.5 | **82** | strong | T2 | pointing+자전 무손실 정전 증명. soft-landing LCvx 표준. |
| Harris·Açıkmeşe, Lossless convexification of non-convex optimal control problems for state constrained linear systems, **Automatica 2014** | 4.5·5·4.5·2.5·2.5·4·3.5·4 | **78** | strong | T1 | [control-theory] 상태제약 연속활성 무손실(강제어가능 부분공간). LCvx 이론 마디. |
| Scharf·Açıkmeşe·Dueri·Benito·Casoliva, Implementation and Experimental Demonstration of Onboard Powered-Descent Guidance, **JGCD 2017** | 3.5·4·4.5·4·4.5·4·5·4.5 | **85** | flagship | T3 | G-FOLD/LCvx 온보드 비행시연(Masten Xombie 8비행·100ms·aspect~3). convex PDG deployment capstone(이론은 companion [15]에 위임·무손실 일부 포기). |
| Szmuk·Açıkmeşe, Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time, **AIAA GNC 2018** | 4·4.5·4.5·2·2·4·3·4 | **72** | good | Conf | 6-DoF 자유종단 SCvx 최초급(자체 수렴증명 부재·notional sim). |
| Malyuta et al., Convex Optimization for Trajectory Generation: A Tutorial on Generating Dynamically Feasible Trajectories Reliably and Efficiently, **IEEE CSM 2022** | 4·3.5·4.5·2.5·3.5·5·4.5·5 | **80** | strong | T1 | LCvx/SCvx/GuSTO 결정판 튜토리얼+오픈소스. |
| Luo·Elango·Açıkmeşe, Remarks on "Successive Convexification: A Superlinearly Convergent Algorithm for Non-convex Optimal Control Problems", **arXiv 2024** | 4·2.5·4·–·1·4·2·3 | **58** | fair | arXiv | [control-theory] 원 SCvx 수렴증명 결함 정정. |
| Mao·Açıkmeşe, SCvx-fast: A Superlinearly Convergent Algorithm for A Class of Non-Convex Optimal Control Problems, **arXiv 2021** | 4.5·4·4·–·2·4·3.5·2.5 | **70** | good | arXiv | [control-theory] project-and-convexify superlinear(클래스 좁음·단일예제). |
| Oguri, Successive Convexification with Feasibility Guarantee via Augmented Lagrangian for Non-Convex Optimal Control Problems, **CDC 2023** | 4·4·4.5·–·2.5·4·2.5·3 | **71** | good | Conf | [control-theory] feasibility-guarantee+패널티 자동화. |
| Kamath et al., Real-Time Sequential Conic Optimization for Multi-Phase Rocket Landing Guidance, **IFAC 2023** | 4·4.5·4.5·2·3.5·4·4.5·4 | **79** | strong | Conf | matrix-free 다상 SCP + single-crossing STC. |
| Bernardini et al., State-dependent trust region for successive convex programming for autonomous spacecraft, **Astrodynamics 2024** | 3.5·3.5·4·3·4.5·3.5·3.5·3.5 | **74** | good | T3 | 변수별 TR 자동화(조립형). |
| Elango et al., Continuous-time successive convexification for constrained trajectory optimization, **Automatica 2025** | 4.5·4.5·5·–·3.5·4.5·4.5·4 | **87** | flagship | T1 | 연속시간 제약만족 SCP + C-codegen 실시간 + 오픈소스. |
| Luo·Echigo·Açıkmeşe, Revisiting Lossless Convexification: Theoretical guarantees for discrete-time optimal control problems, **Automatica 2026** | 4.5·4.5·4.5·3.5·3·4·3·4 | **79** | strong | T1 | [control-theory] 이산시간 LCvx 이론 갭(grid bang-bang 보존). |
| Luo·Spada·Açıkmeşe, Discrete-Time Lossless Convexification for Pointing Constraints, **IEEE LCSYS 2025** | 4.5·4·4·–·2.5·5·3·3.5 | **75** | strong | T3 | 이산 pointing LCvx(R³ 보증 결여, letter). |
| Saranathan, Indirect optimization of multi-phase Mars entry, descent, and landing trajectory involving arbitrary discrete logic, **ASR 2025** | 3.5·3.5·4·–·3·4·2·3 | **66** | good | T5 | indirect 임의 boolean 논리(비실시간). |
| Zou et al., Parallel-in-Time Nonlinear Optimal Control via GPU-native Sequential Convex Programming, **arXiv 2026** | 3·3.5·3.5·3·3·3.5·4·3 | **66** | good | arXiv | [control-theory] GPU-native SCP(Jetson 100Hz 측정·이론 얇음·코드 비공개). |
| Zhao·Shang·Dong·Wang, Multi-phase trajectory optimization of aerospace vehicle using sequential penalized convex relaxation, **AST 2021** | 4·4·4·2·3.5·4·3·3 | **70** | good | T3 | free phase-linkage/종단 time-projection + penalized SDP relaxation; 단일 NLP baseline·rank-1 미보장·SDP 차원폭증(onboard 비현실). |
| Spada·Sagliano·Topputo, Direct-Indirect Hybrid Strategy for Optimal Powered Descent and Landing, **JSR 2023** | 4·3.5·4·4·3.5·4·3·3 | **73** | good | T5 | PS-convex 직접해를 exact-CMT로 indirect BVP에 warm-start(순수 indirect 대비 ~10×·BVP급 정밀); 단일 benchmark·자기 baseline·진공·offline·결정론. |
| Yang·Liu·Song, Rocket Landing Guidance Based on Linearization-Free Convexification, **JGCD 2024** | 4·4·4·4·4.5·4·4·4 | **82** | strong | T3 | 선형화·trust-region 없이 단일 SOCP(질량/경로 사전동결+V²치환+목적등가증명); ms급(~75–90× vs SCO)·단일phase·결정론·수렴 경험적. |

### B. RLV landing & reentry GNC (30)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Chai et al., Adaptive and Efficient Model Predictive Control for Booster Reentry, **JGCD 2020** | 3.5·3·3.5·4·3·4·4·2.5 | **67** | good | T3 | 부스터 재진입 추종 MPC(control layer). |
| Sagliano et al., Onboard Guidance for Reusable Rockets: Aerodynamic Descent and Powered Landing, **AIAA SciTech 2021** | 3.5·4·4·2·2.5·4·3·3.5 | **67** | good | Conf | 공력→동력 전환 통합 선례(결정론·전환점 고정). |
| Song et al., Adaptive powered descent guidance based on multi-phase pseudospectral convex optimization, **Acta Astro 2021** | 4·3.5·4·4·4·4·3·3.5 | **75** | strong | T3 | 다상 pseudospectral PDG(switch-time 변수화). |
| Spannagl et al., Design, Optimal Guidance and Control of a Low-cost Re-usable Electric Model Rocket, **IROS 2021** | 3·3·3.5·3.5·3·2·4.5·2.5 | **63** | fair | Conf | 저가 모델로켓 추종(실HW·동역학 대표성 약). |
| Hwang·Ahn, Integrated Optimal Guidance for Reentry and Landing of a Rocket Using Multi-Phase Pseudo-Spectral Convex Optimization, **IJASS 2022** | 3.5·3·3.5·2·2.5·4·3·3 | **61** | fair | T5 | 재진입↔착륙 통합 결정론 convex(검증 빈약). KAIST. |
| Guadagnini et al., Model predictive control for reusable space launcher guidance improvement, **Acta Astro 2022** | 3·3·4·4·3.5·4·3·3 | **68** | good | T3 | SCvx-PDG trade-off(저자 신규성 부정·불확실성 미반영). |
| Sagliano et al., Powered Atmospheric Landing Guidance for Reusable Rockets: the CALLISTO studies, **AIAA SciTech 2024** | 3.5·3.5·4·2·3·3·3·4 | **67** | good | Conf | variable-triggered 운용제약(정성 trade-off·aero→reignition 구성). |
| Chung·Echigo·Açıkmeşe, Ignition Point Reachability for Aerodynamically-Controlled Reusable Launch Vehicles, **AIAA SciTech 2025** | 3·4·4·3·3·5·2·3.5 | **68** | good | Conf | 결정론 aero→점화 reachable set(SCP). |
| Jo·Han·Ahn, Fuel-optimal boost-back guidance algorithms for reusable launch vehicles, **ASR 2025** | 4·3·4·4·4·4·4·3 | **75** | strong | T5 | IIP-FPA + min-impulse boost-back 유도. |
| Mceowen et al., Autotuned Primal–Dual Successive Convexification for Reentry Guidance, **JGCD 2025** | 4·4·4·4·4·4·4·3.5 | **79** | strong | T3 | dual 패널티 자동튜닝 + 1230-case dispersion(수렴증명 부재). |
| Zhou et al., Online reentry trajectory planning for reusable rockets based on intelligent phase transition, **AST 2026** | 3.5·4·3.5·4·4·4·4·3.5 | **76** | strong | T2 | RL 전환결정+해석해(point-estimate·합성 없음·CPU 0.28s). |
| Chen et al., Explicit Trajectory Dispersion Control for Precision Landing Guidance of Reusable Rockets, **JGCD 2026** | 4·4·4·4·3·3.5·4·3.5 | **75** | strong | T3 | gPC online dispersion 직접제어(단일-phase). |
| Su·Liu, Reentry trajectory optimization for reusable launch vehicle using marine predator whale optimizer and smoothing technique, **AST 2025** | 3·2.5·3.5·3·4·3.5·2·2.5 | **60** | fair | T2 | metaheuristic reentry(analytic backbone 전무). |
| Jung et al., Optimal Landing Guidance of Reusable Launch Vehicles With Dual-convex Optimization, **IJCAS 2025** | 3.5·4·4·4·4·3.5·4·3.5 | **76** | strong | T4 | 6-DoF 실시간 dual-convex 착륙 유도(결정론·KAIST). |
| Yang, Biased-Proportional-Navigation-Based Predictor–Corrector Landing Guidance for Reusable Launch Vehicle, **JGCD 2025** | 3.5·3.5·4·3.5·3.5·4·3.5·3.5 | **72** | good | T3 | BPN 입력축소로 PC의 "단일입력 요구" blocker 해소(0.4 ms·연료 6~7% 갭); BPN+착륙은 [24] 선점·최적성증명이 1D 간소화 한정·점화 시 α̂ 분모 영점통과를 포화로 은폐·경쟁 유도법 baseline 0건·miss 4.7e-5 m는 nav/자세/액추에이터 부재 산물. *2026-07-15 원문정독 재교정 82→72.* |
| Robbiani et al., Fast Desensitized Optimal Control for Rocket-Powered Descent and Landing, **JGCD 2025** | 3.5·3.5·4·4·3.5·4·3.5·3.5 | **73** | good | T3 | desensitization(1차 sensitivity 기반 강건화). |
| de Ridder·Mooij, Optimal Terminal Area Strategies and Energy Tube Concept for a Winged Re-Entry Vehicle, **AIAA GNC 2009** | 2.5·3·3.5·3·2.5·3.5·3·2.5 | **58** | fair | Conf | TAEM "energy-tube"(허용 min/max 에너지=V–h 영역) 개념 도입(foundational Mooij); 단일차량·결정론·MC/baseline 없음·GA 비수렴·supersonic 미완. |
| Lu, Entry Guidance: A Unified Method, **JGCD 2014** | 4.5·4.5·5·4·4·2·4·5 | **86** | flagship | T3 | 정전 통일 predictor-corrector entry(예측 제약강제로 PC 약점 해결); 단일phase·결정론·3-DoF·논문내 MC 없음. |
| Toso·Maddock, Launch abort trajectory optimisation for reusable launch vehicles, **AIAA Space Planes Conf 2017** | 2·2.5·3·1.5·2·2.5·2·1.5 | **43** | poor | Conf | 양력체 RLV 상승-중 thrust-loss abort 활공 회수영역을 direct multiple-shooting+SQP로 매핑; 결정론 open-loop 재최적화·baseline/통계 없음. |
| Zhang·Liu·Liu, High-Precision Adaptive Predictive Entry Guidance for Vertical Rocket Landing, **JSR 2019** | 2.5·3·3·3.5·2.5·1.5·3·3 | **56** | fair | T5 | RCV entry NPC를 α/β 그리드핀+2계수(KD,KL) 온보드 적응보정; entry phase만 해결(landing IC)·외부 baseline 없어 "고정밀" 미입증. |
| Ma·Wang·Shao·Song·Biegler, Direct trajectory optimization framework for VTVL reusable rockets: two-stage case study, **Eng. Optimization 2019** | 3·3·4·2.5·3·3.5·2.5·2.5 | **61** | fair | T5 | 상승+1단 회수 통합 OCP를 Radau collocation→IPOPT·자유 phase 길이; 새 이론 없음·경쟁 baseline 없음(자기 실패init만). |
| Simplício·Marcos·Bennani, Guidance of Reusable Launchers: Improving Descent and Landing Performance (DESCENDO), **JGCD 2019** | 3.5·3.5·4·2.5·3·4·4·3.5 | **70** | good | T3 | 유럽 RLV 확장-포락선 online SCvx(aero+powered 단일연속); 2-burn t1/t2 grid(전환 미최적화)·이론보장/불확실성 없음. |
| Wu·Tian·Li, Full-stage Reentry Trajectory Optimization for Reusable Launch Vehicle, **CCC 2020** | 2.5·2.5·3·2·2·3.5·2.5·2 | **49** | weak | Conf | full-stage(IRE→TAEM→A&L) phase별 연결; IRE만 variable-TR SCP·전환=경계조건 chaining(불연속 "해소" 과장)·baseline/solve-time 없음. |
| Cheng·Jing·Gao, Recovery trajectory planning for the reusable launch vehicle, **AST 2021** | 2.5·3·3·3.5·2.5·3·2.5·2.5 | **56** | fair | T3 | 전 회수구간(boostback→aero→powered) PSO+다항식; 전환 고정·impulse 근사·mean-wind만(분포보증 없음). |
| Lee·Lee, Multi-phase and dual aero/propulsive rocket landing guidance using successive convex programming, **IMechE G 2023** | 3.5·3.5·4·3·3.5·3.5·4·3 | **70** | good | T5 | 6-DoF dual aero/propulsive SCP(공력핀 모멘트로 TVC손실↓); STC 전환은 고정고도 단순게이트·외부 baseline 없음. KAIST. |
| Lu·Sandoval·Davami, Fast and Robust Optimization of Full Trajectory from Entry Through Powered Descent, **JGCD 2024** | 4·4·4·4·4.5·3·4·4 | **80** | strong | T3 | entry→PDG 전환을 σ_f 1변수 bilevel로 비용최적화(결정론·전환은 PDI heuristic 간접결정·chance/Σ/recursive-feasibility 없음). |
| Mishra·Sushnigdha, A novel re-entry trajectory design strategy enforcing inequality and terminal constraints in height-velocity plane, **ASR 2024** | 3·3·4·3·3·4·4.5·3 | **67** | good | T5 | 날개형 RLV 재진입 h–V 회랑 2-param(LLL)+FL 추종; 이론보장·방법 baseline 없음·MC 35회 얕음. HIL(OPAL-RT)은 강점. |
| Wang·Zhang·Li, Endoatmospheric powered descent guidance, **Acta Astro 2024** | 4·4·4·4·3.5·4·4·4 | **78** | strong | T3 | 고AoA(90° bellyflop) endoatmospheric PDG 실시간 first; PMP-유도 ω정칙화+zero-max-mid 중간추력 feedback마진; restricted-6-DoF·외부 baseline 없음·결정론. |
| Chen·Zhang·Li, Optimal feedback guidance with disturbance rejection for endoatmospheric powered descent, **CJA 2025** | 4·4·4.5·2.5·2·5·3·4 | **72** | good | T4 | 외란 modeled/unmodeled 분할 + J₁주입 robust neighboring-optimal feedback(PDDP 28×↑); 외란감쇠=결정론 L₂/H∞(분포/chance 아님)·외부 baseline 없음·단일phase. |
| Lee·Jung·Lee, Convex Optimization Based Landing Guidance for Reusable Orbital Rockets, **IJASS 2025** | 3.5·3·4·4·4·4·4·3 | **73** | good | T5 | 4-phase(belly-flop→flip→align→land) 6-DoF dual-opt(plan+LQT-MPC track)·σ_j 전환시각 최적화; 이론보장/위험정량 없음. KAIST. |

### C. Covariance steering · chance constraint · risk allocation (18)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Pilipovsky·Tsiotras, Covariance Steering with Optimal Risk Allocation, **IEEE TAES 2021** | 4·4·4·4·3.5·3.5·2.5·3 | **72** | good | T3 | [control-theory] CS+IRA(δ 최적배분)+cone SOC(within-phase IRA). (PDF=TAES 저널본, 정정·개명) |
| Ridderhof·Tsiotras, Minimum-Fuel Closed-Loop Powered Descent Guidance with Stochastically Derived Throttle Margins, **JGCD 2021** | 4·4·4.5·4.5·3.5·4·3.5·3.5 | **79** | strong | T3 | CS→mass-varying PDG + margin-from-covariance(무반복). |
| Ridderhof·Tsiotras, Chance-Constrained Covariance Steering in a Gaussian Random Field via Successive Convex Programming, **JGCD 2022** | 4·4.5·4.5·4.5·3.5·4·3·3 | **78** | strong | T3 | 공간상관 외란(GRF)→시간 통계 적분유도. |
| Renganathan et al., Distributionally Robust Covariance Steering with Optimal Risk Allocation, **ACC 2023** | 3.5·3.5·4·4·3·5·3·3 | **71** | good | Conf | [control-theory] Φ⁻¹→Cantelli 치환(moment-DR·코드공개·신규성 얇음). |
| Rapakoulias·Tsiotras, Discrete-Time Optimal Covariance Steering via Semidefinite Programming, **CDC 2023** | 4.5·4·4.5·4·3.5·4·3.5·3.5 | **80** | strong | Conf | [control-theory] 이산 CS lossless SDP(자체증명·런타임 스케일). |
| Pilipovsky·Tsiotras, Computationally Efficient Chance Constrained Covariance Control with Output Feedback, **CDC 2024** | 4·3.5·4·3.5·2.5·4·3.5·3 | **70** | good | Conf | [control-theory] Kalman+sequential-LMI+DC/CCP(검증 최약·lossless 미증명). |
| Yu et al., Optimal Covariance Steering of Linear Stochastic Systems with Hybrid Transitions, **arXiv 2024** | 4.5·4.5·4·4·3·4·3·3.5 | **77** | strong | arXiv | [control-theory] guard 가로지른 CS(timing 포함·chance/trigger/GP 없음). |
| Pilipovsky·Tsiotras, Distributionally Robust Density Control with Wasserstein Ambiguity Sets, **CDC 2024** | 4·4·4·4.5·3.5·3.5·2.5·3.5 | **75** | strong | Conf | [control-theory] Wasserstein density control(non-Gaussian tightening). |
| Ryu et al., Risk-Sensitive Orbital Debris Collision Avoidance Using Distributionally Robust Chance Constraints, **AIAA SciTech 2025** | 3.5·3·4·4·3·4·3·3 | **68** | good | Conf | moment-DR-CVaR 단일 handoff. |
| Ratheesh et al., Operator Splitting Covariance Steering for Safe Stochastic Nonlinear Control, **CDC 2025** | 4·4·4·4.5·3.5·4·3·3.5 | **76** | strong | Conf | [control-theory] ADMM nonlinear CS(중간 비가해 허용). |
| Gahlawat et al., Wasserstein Distributionally Robust Adaptive Covariance Steering, **CDC 2025** | 4·4·4·4·1·2·2·3 | **62** | fair | Conf | [control-theory] L1-adaptive+Wasserstein(실험 0건·미성숙). |
| Shrivastava·Oguri, Chance-Constrained Covariance Steering for Discrete-Time Markov Jump Linear Systems, **arXiv 2025** | 4.5·4·4·4.5·3·4·3.5·3.5 | **77** | strong | arXiv | [control-theory] 확률전환(외생 Markov)+chance+CS. |
| Patil et al., Strong Duality and Dual Ascent Approach to Continuous-Time Chance-Constrained Stochastic Optimal Control, **arXiv 2025** | 4.5·4·4·4·3·2·3·3.5 | **73** | good | arXiv | [control-theory] chance=exit-time/first-passage 형식론(단일 탈출·가정 위임). |
| Knaup·Tsiotras, Recursively Feasible Stochastic Model Predictive Control for Time-Varying Linear Systems Subject to Unbounded Disturbances, **Automatica 2026** | 4.5·4.5·4.5·4.5·3·4·3.5·4 | **82** | strong | T1 | [control-theory] unbounded 외란 LTV recursive feasibility(단일 phase). |
| Garzelli et al., Gaussian-Process-Based Covariance Control for Autonomous On-Orbit Servicing, **JGCD 2026** | 4·3·4·4·3.5·4·3.5·3 | **72** | good | T3 | GP 잔차→covariance control(cone mean-only). |
| Pilipovsky, Free Final Time Adaptive Mesh Covariance Steering via Sequential Convex Programming, **arXiv 2026** | 4.5·4·4.5·4·3·2·2.5·3.5 | **74** | good | arXiv | [control-theory] 승법노이즈 free-final-time CS(정지시각 future work로 비움). |
| D'Silva·Tsukamoto, Statistical Contraction for Chance-Constrained Trajectory Optimization of Non-Gaussian Stochastic Systems, **arXiv 2026** | 4·4·4·4.5·4·5·4·3.5 | **81** | strong | arXiv | [control-theory] CP+contraction non-Gaussian tail(분포무관·HW·코드). |
| Barbosa·Löfberg, Stochastic Model Predictive Control with Online Risk Allocation and Feedback Gain Selection, **arXiv 2026** | 4·3.5·4·4·2.5·4·3·3 | **70** | good | arXiv | [control-theory] MICP online risk-alloc+feedback 선택(합성·recursive feasibility 없음). |

### D. Hybrid systems · saltation · state-triggered constraints (17)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Boccadoro et al., Optimal Control of Switching Surfaces in Hybrid Dynamical Systems, **DEDS 2005** | 4.5·4·4.5·–·2·4·2·3 | **70** | good | T4 | [control-theory] 결정변수=switching-surface 파라미터 결정론 원조(transversality 붕괴 자인). |
| Egerstedt et al., Transition-time optimization for switched-mode dynamical systems, **IEEE TAC 2006** | 4·4·4.5·–·2·4·2.5·4 | **72** | good | T1 | [control-theory] switching-time(개루프) 1차 그래디언트 정전(toy·외부위임). |
| Abate et al., Probabilistic reachability and safety for controlled discrete time stochastic hybrid systems, **Automatica 2008** | 4.5·4.5·4.5·4·2.5·4·2·4 | **76** | strong | T1 | [control-theory] P(안전)≥1−ε across modes를 stochastic-DP로 엄밀정의(정확-but-비실시간). |
| Reynolds et al., Dual Quaternion-Based Powered Descent Guidance with State-Triggered Constraints, **JGCD 2020** | 4·4·4·4·3.5·4·4·4 | **78** | strong | T3 | range-trigger STC를 dual-quaternion 6-DoF SCvx에 인코딩+1000 MC. |
| Szmuk et al., Successive Convexification for Real-Time Six-Degree-of-Freedom Powered Descent Guidance with State-Triggered Constraints, **JGCD 2020** | 4·5·4.5·3·3.5·3.5·4.5·5 | **84** | strong | T3 | STC를 LCP로 정초·σ̂=−min(g,0) 유도. trigger-as-constraint 원전. |
| Kong et al., The Salted Kalman Filter: Kalman filtering on hybrid dynamical systems, **Automatica 2021** | 4·4·4·4·4·4·3.5·2.5 | **75** | strong | T1 | [control-theory] saltation을 KF로 통합 정본. Σ-점프 가드레일(근사 붕괴 비율표). |
| Kong et al., iLQR for Piecewise-Smooth Hybrid Dynamical Systems, **CDC 2021** | 4·3.5·4·2.5·3·1·2.5·2 | **61** | fair | Conf | [control-theory] saltation을 iLQR backward에 적용(toy·결정론). |
| Payne et al., The Uncertainty Aware Salted Kalman Filter: State Estimation for Hybrid Systems with Uncertain Guards, **IROS 2022** | 4·4·4·4·4·4·3·3 | **76** | strong | Conf | [control-theory] uncertain guard→추가 Σ(guard saltation matrix). |
| Pytlak·Suski, Trajectory sensitivity analysis of hybrid systems with sliding motion, **NAHS 2022** | 4·4·4·–·1·3·2·2 | **59** | fair | T3 | [control-theory] sliding 영역 민감도 DAE(검증예제 0). |
| Kong et al., Saltation Matrices: The Essential Tool for Linearizing Hybrid Dynamical Systems, **Proc. IEEE 2024** | 4·3·4.5·3·2·3.5·3·4 | **68** | good | T1 | [control-theory] saltation 표준 튜토리얼+서베이(부록 Σ·Riccati). |
| Laouar et al., Feasibility-Guided Safety-Aware Model Predictive Control for Jump Markov Linear Systems, **IROS 2024** | 3.5·3.5·4·3.5·3·3.5·4·3 | **70** | good | Conf | [control-theory] jump-Markov 안전MPC(전환 외생·결정론). |
| Katayama et al., Robustifying Model-Based Locomotion by Zero-order Stochastic Nonlinear Model Predictive Control with Guard Saltation Matrix, **IROS 2024** | 4·4·3.5·4·3·4·4·3 | **73** | good | Conf | [robotics-transfer·G&C-capped] guard-saltation covariance + chance backoff(trigger 비최적화). |
| Crestaz et al., Constrained Trajectory Optimization for Hybrid Dynamical Systems, **arXiv 2024** | 3.5·3·3.5·2.5·3·4.5·3·3 | **64** | fair | arXiv | [control-theory] hybrid iLQR 궤적최적화(2D toy 단일). |
| Uzun et al., Sequential Convex Programming for 6-DoF Powered Descent Guidance with Continuous-time Compound State-Triggered Constraints, **AIAA SciTech 2025** | 4·3.5·4·2·2.5·4·3·3.5 | **67** | good | Conf | CT compound-STC 6-DoF PDG(결정론 backbone·단일케이스). |
| Shaji et al., Path Integral Particle Filtering for Hybrid Systems via Saltation Matrices, **arXiv 2026** | 4·3.5·4·3.5·4·4·2.5·3 | **72** | good | arXiv | [control-theory] hybrid PF via saltation(비Gaussian 헤드라인 미검증). |
| Xu·Antsaklis, Optimal Control of Switched Systems Based on Parameterization of the Switching Instants, **IEEE T-AC 2004** | 4.5·4·4·–·2.5·3.5·2·4 | **72** | good | T1 | [control-theory] switch-time 결정변수화 + ∂J/∂t_s 정확산출(2차 sensitivity-BVP). switch-time-as-decision의 결정론 정전. toy 검증·자기 baseline. |
| Zhao·Shang·Liu·Zhang, Autonomous Trajectory Planning with Event-Trigger Constraints Using Nonsingular Convex Optimization, **JGCD 2024** | 3·3.5·3.5·2·3·3.5·3·3 | **62** | fair | T3 | event-trigger(if문)을 단일 호모토피 포화함수로 볼록화(STC/max-함수 대안); 형식증명·정량 baseline 없음·rocket landing 아님(rendezvous/hopping). Technical Note. |

### E. GP / learning disturbance · uncertainty calibration (12)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Hewing et al., Cautious Model Predictive Control Using Gaussian Process Regression, **IEEE TCST 2020** | 4·4.5·4.5·4·3.5·3.5·5·4.5 | **84** | strong | T2 | [control-theory] GP-MPC 정전(PRS chance tightening·calibration 미검증). |
| Torrente et al., Data-Driven MPC for Quadrotors, **RA-L 2021** | 3·3.5·4·2.5·3·5·4·3 | **68** | good | T2 | [robotics-transfer·G&C-capped] 공력잔차 GP→full-state MPC(variance 미사용). |
| Schmid et al., A real-time GP based MPC for quadcopters with unknown disturbances, **ACC 2022** | 2.5·3·3.5·3·2.5·3.5·4·2.5 | **60** | fair | Conf | [robotics-transfer] SS-GP+Cautious 조립(이론 delta 없음·전환 휴리스틱). |
| O'Connell et al., Neural-Fly enables rapid learning for agile flight in strong winds, **Sci Robotics 2022** | 4·4.5·4.5·4·3·5·4·3.5 | **80** | strong | T1 | [robotics-transfer·G&C-capped] meta-learning 강풍 적응비행(실HW·코드·Σ 없음). |
| Scoggins et al., Multihierarchy Gaussian Process Models for Probabilistic Aerodynamic Databases using Uncertain Nominal and Off-Nominal Configuration Data, **AIAA SciTech 2023** | 3.5·3.5·4·3.5·2.5·3·2·2.5 | **63** | fair | Conf | 공력잔차 GP DB 템플릿(calibration 미검증). |
| Zhou et al., Safety-Critical Control with Uncertainty Quantification using Adaptive Conformal Prediction, **ACC 2024** | 3.5·3·3.5·4·3.5·4·3·3 | **68** | good | Conf | [control-theory] ACP→CBF chance hand-off worked example(baseline 빈약). |
| Frey et al., Efficient Zero-Order Robust Optimization for Real-Time Model Predictive Control with acados, **ECC 2024** | 3·3·4·4·3·5·4·3.5 | **71** | good | Conf | [optim-tool·G&C-capped] zoRO BLASFEO 가속 구현(propagation/chance 검증 전무). |
| Scampicchio et al., Gaussian processes for dynamics learning in model predictive control, **Annu. Rev. Control 2025** | 3.5·3·4·4·1.5·2.5·3.5·4 | **65** | good | T1 | [control-theory] GP-MPC scalability/propagation/safety 균형 서베이. |
| Liu et al., Learning for Predictive Control: A Dual Gaussian Process Approach, **Automatica 2025** | 4·4·4·3.5·3·4·3.5·3.5 | **74** | good | T1 | [control-theory] long/short GP 분리(실시간 미측정·closed-loop 보증 전무). |
| Hayes·Caverly, Atmospheric-Density-Compensating Model Predictive Control for Targeted Reentry of Drag-Modulated Spacecraft, **JGCD 2025** | 3·3.5·4·4·3.5·4·3.5·3 | **70** | good | T3 | 밀도오차 EKF→MPC 보상(244→12 km·공분산 미전파). |
| Lahr et al., L4acados: Learning-Based Models for acados, Applied to Gaussian Process-Based Predictive Control, **IEEE TCST 2025** | 3.5·3.5·4·4·4.5·5·5·4 | **82** | strong | T2 | [control-theory] GP/NN→acados 실시간 주입+실차 측정. |
| Carradori·Sagliano·Mooij, Transformer-Based Atmospheric Rocket Landing Guidance Using Meta-Reinforcement Learning, **JGCD 2026** | 2.5·3·3.5·4·4·4·4·3.5 | **69** | good | T3 | GTrXL meta-RL을 full 6-DoF 고충실(US76/HWM14/공력DB) 대기권 RLV 착륙에 적용; 강건성=도메인랜덤화 경험빈도(분포보증無)·SOTA convex baseline 없음. DLR. |

### F. Real-time embedded conic solvers (6)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Numerow et al., Inherently Robust Suboptimal MPC for Autonomous Racing With Anytime Feasible SQP, **RA-L 2024** | 4·3.5·4·4·4·4·4.5·3.5 | **78** | strong | T2 | [control-theory] anytime feasible SQP(조기종료 개념 유용·δ마진 미정량). |
| Mahajan et al., Code Generation and Conic Constraints for Model-Predictive Control on Microcontrollers with Conic-TinyMPC, **arXiv 2024** | 3.5·3.5·4·2.5·4·5·5·4 | **77** | strong | arXiv | [control-theory] MCU SOC-MPC codegen 실HW(SDP 미지원). |
| Amiri·Hosseinzadeh, Practical Considerations for Implementing Robust-to-Early Termination Model Predictive Control, **SCL 2025** | 4·3.5·4·3.5·4·5·4.5·3 | **77** | strong | T3 | [control-theory] robust-to-early-term 이산구현(polytope-LTI 한정). |
| Goulart·Chen, Clarabel: An Interior-Point Solver for Conic Programs with Quadratic Objectives, **Math. Prog. Comput. 2026** | 4·3.5·4.5·3·3·5·4·3.5 | **75** | strong | T1 | [optim-tool·G&C-capped] 이차목적 conic IPM 기준구현(신뢰성 최상·데스크톱 전용). |
| Chari·Açıkmeşe, QOCO: A Quadratic Objective Conic Optimizer with Custom Solver Generation, **Math. Prog. Comput. 2026** | 3.5·3·4.5·2.5·3·5·4·3.5 | **71** | good | T1 | [optim-tool·G&C-capped] 이차목적 SOCP C-solver+codegen(LCvx PDG·CM4 실측). |
| Jang·Lee, Customized Interior-Point Methods Solver for Embedded Real-Time Convex Optimization, **IEEE TAES 2026** | 3.5·3.5·4·–·3·4·4·3.5 | **72** | good | T3 | [optim-tool·G&C-capped] infeasibility 인증+C-codegen+Cortex-A9 Mars-PDG. |

### G. Disturbance models (wind · turbulence) (1)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Lorenz, Turbulence for extraterrestrial aviation: Gust specification for Dragonfly's powered flights, **PSS 2022** | 2·3·3.5·3·2·4·3·3 | **56** | fair | T4 | 행성 돌풍사양(AR(1)/OU+Dryden 바람모델 템플릿·지구 MIL-STD 재수치화). |

### H. Control / tracking & stabilization (RLV-specific) (49)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Sagliano et al., Unified-Loop Structured H-Infinity Control for Aerodynamic Steering of Reusable Rockets **JGCD 2023** | 3.5·3.5·4·4·3.5·3·3.5·3.5 | **72** | good | T3 | Solid, thoroughly-engineered structured H-inf design for CALLISTO aero descent; strong linear/mu/MC validation but no baseline benchmark and only normalized (no physical-unit) results. |
| Biertümpfel et al., Robust Space Launcher Control with Time-Varying Objectives **JGCD 2024** | 4·3.5·4·4·3.5·4·3·3.5 | **74** | good | T3 | 유한구간 LTV mixed-sensitivity 합성(선행 Ref.[8])을 발사체 대기권 상승에 적용 + 시변 목표를 시변 가중함수로 인코딩. 방법론·배치성(C3/C7=4), 모델 충실도(C1=4) 강점. 합성기법 자체가 선행연구라 novelty 디플레이트(C2=3), 정량 baseline 부재가 핵심 공백(C5=3). 불확실성은 1024 꼭짓점 worst-case+37 난류(분포/chance 아님)→C4=3. |
| Cao et al., Robust Multiple Model Predictive Control for Ascent Trajectory Tracking of Aerospace Vehicles **TAES 2022** | 4·3.5·4·4.5·4·3.5·3.5·3 | **75** | strong | T3 | Gap-metric model-set + polytopic RMMPC + guardian-map switching + transient command correction; solid LMI rigor and real baselines, but the contribution is an integration of established components (Cuzzola/PDL-RMPC, GM switching from Xiao, transient compensation from He/Lu) with worst-case (not distributional) uncertainty. |
| Sun et al., Integrated Launcher Guidance and Control via High-Order Fully Actuated System Approach **JGCD 2026** | 4·3·4·3.5·3·4·4·3 | **71** | good | T3 | HOFAS를 런처 궤도투입 IGC에 적용한 기술적으로 견실하나 점진적인 연구. 닫힌루프 전체 안정성 증명 부재, 단일 자기-baseline(FAS [44]), 시뮬레이션 한정. 해석적 실시간 제어법칙(<1ms)은 명확한 강점. |
| Li et al., RLV integrated guidance and control based on adaptive high-order sliding mode **Nonlinear Dynamics 2023** | 4·3·4·3.5·2.5·4·3·3 | **67** | good | T3 | rel:M |
| Dong et al., Adaptive-gain multivariable super-twisting sliding mode control for reentry RLV with torque perturbation **IJRNC 2016** | 3.5·4·3.5·3·2.5·4·3·3 | **66** | good | T4 | rel:M. Incremental multivariable extension of adaptive-gain STW (Shtessel [35,36]) to double-loop RLV reentry with first treatment of multiplicative torque perturbation; two Lyapunov finite-time proofs but standard handwaves (xi=0/zeta=0 assumed, two-timescale asserted, R-nonsingularity circular); single-scenario 6-DOF X-33 sim, baselines only qualitative prose, no Monte Carlo / actuator limits. |
| Hu et al., Robust model predictive control for hypersonic vehicle with state-dependent input constraints and parameter uncertainty **IJRNC 2021** | 3.5·3.5·3.5·3.5·2.5·3.5·3·3 | **65** | good | T4 | State-dependent virtual input constraint under feedback linearization handled via SOS-to-LMI relaxation embedded in infinite-horizon min-max RMPC; deterministic worst-case uncertainty (polytopic LPV + norm-bounded disturbance) so C4 mid. Empirical section is the main weakness: single longitudinal AFRL model, one climb trajectory, single RMPC baseline, no Monte Carlo despite 50% mass/inertia uncertainty, results reported only qualitatively from figures with no error/chattering tables. T4 venue. |
| Gäßler·Robens, Incremental Nonlinear Dynamic Inversion Flight Control for the DLR Reusability Flight Experiment ReFEx **AIAA SciTech 2025** | 2.5·2.5·3.5·3.5·3·2.5·3.5·3.5 | **61** | fair | Conf | C4 scored (not '-'): paper claims disturbance/uncertainty robustness via Monte Carlo over perturbed AEDB/MCI/wind/sensor models with 1σ/3σ bands → robust/worst-case style, mid score; no chance-constrained or distributional guarantee. TOTAL recomputed = (2·17+2·15+3·16+3·9+2·14+2·8+4·9+3·12)/100 = 255/100 = 2.55 ×20 = 51. Engineering application/design-status paper for a real DLR flagship mission; near-zero method novelty (standard NDI+INDI cascade + DLS allocation from ref [51]); no stability proof; only 100 MC runs with qualitative results and no baseline. |
| Trotta et al., Optimal Tuning of Adaptive Augmenting Controller for Launch Vehicles in Atmospheric Flight **JGCD 2020** | 3·3·3.5·4·3.5·3.5·3·3 | **66** | good | T3 | rel:M. Engineering Note: automates AAC gain tuning (GA over noisy MC vs Nelder-Mead min-max corner cases) for a VEGA-class single-axis LV; AAC architecture itself is NASA SLS prior art, so the delta is methodological, not a new controller. |
| Schierman et al., Integrated Adaptive Guidance and Control for Re-Entry Vehicles with Flight-Test Results **JGCD 2004** | 4·4.5·4·4·4.5·3·5·4 | **84** | strong | T3 | 통합 재구성 G&C(동적역변환+제어할당+모델추종)와 오프라인 OPTG 궤적재형성을 TIFS 비행시험(X-40A 모사)으로 실증. 신규성은 통합·OPTG에 한정, 안정성 증명 부재. 강점은 온보드 배치성과 실비행 검증. |
| Simplício et al., Structured Singular-Value Analysis of the Vega Launcher in Atmospheric Flight **JGCD 2016** | 3·3·4·4.5·4·2·2·3 | **66** | good | T3 | 성숙한 mu-해석을 Vega 발사체에 체계적으로 적용한 견고한 산업 V&V 사례. 이론적 신규성은 낮으나 검증(10,000 MC + 실비행 데이터)은 강함. |
| Tian et al., Adaptive Multivariable Reentry Attitude Control of RLV With Prescribed Performance **IEEE TSMC 2022** | 4·3·4·3.5·2·4·3·3 | **66** | good | T1 | 5-page correspondence; rigorous two-step Lyapunov proof but single sim run with no baseline. Total recomputed: (4*17+2*15+3*16+2*9+2*14+3*8+3*9+2*12)/100*20 = 53.4 -> 53. |
| Mao et al., Reentry attitude control for a reusable launch vehicle with aeroservoelastic model using type-2 adaptive fuzzy sliding mode control **IJRNC 2018** | 4·3·4·3.5·3·4·3·2.5 | **68** | good | T4 | Combination paper (interval type-2 fuzzy + adaptive + SMC + backstepping) in a crowded RLV-reentry niche; refs 30-32 already cover type-2 fuzzy SMC for flexible hypersonic vehicles, so the scoped delta is thin. Full Lyapunov UUB proof is complete but textbook. Single weak baseline (plain SMC). Aeroelastic coupling is largely decorative. C4 scored (bounded/worst-case robustness, no distributional treatment). |
| You et al., Multivariable supertwisting fixed-time approach for RLV re-entry attitude control **IJRNC 2019** | 4·3·3.5·3.5·3·4·3·2.5 | **66** | good | T4 | rel:M. Incremental combination of multivariable supertwisting (Nagesh-Edwards) and fixed-time stability (Polyakov); meaningful overlap with authors' own refs 36-38. Rigorous multivariable STW fixed-time proof (Theorem 1) but rests on a vanishing-disturbance bound |  | Delta |  | <=xi |  | x1 |  | and gives no feasibility demonstration for the coupled gain conditions. Validation is a single X-33 sim with one disturbance realization, qualitative HOSM baseline, no MC/saturation/error tables. |
| Mooij·Wang, Incremental Sliding Mode Control for Aeroelastic Launch Vehicles with Propellant Slosh **AIAA SciTech 2021** | 4·3·4·4.5·3.5·4·3·3 | **72** | good | Conf | INDI-SMC framework은 Ref.19/20 선행(Wang et al. 2019)이며, 본 논문의 delta는 (1) C^T null-space를 활용한 unmatched 불확실성 강건 sliding surface, (2) under-actuated 시스템으로의 확장 및 Lyapunov 증명 두 가지로 명확히 scoping됨. 적용형 incremental 기여로 C2/C8 deflate. 강건성 주장(slosh 파라미터 변동, 결함/외란) 있으므로 C4 채점(robust/ultimate-bound = mid, 분포는 uniform 임의 선택, chance-constraint 없음). 단일 PacAstro 차량, pitch-plane, sim-only, real/HIL 없음. |
| Navarro-Tapia et al., Legacy recovery and robust augmentation structured design for the VEGA launcher **IJRNC 2019** | 3.5·3.5·4·4.5·4.5·2·4·3.5 | **76** | strong | T4 | 산업용 VEGA TVC 벤치마크에 structured H-inf을 적용한 응용/통합 연구. 방법론 신규성은 낮으나(붐비는 launcher-H-inf 계열, 저자 선행과 인접) 모델링·검증·배치 적합성은 견고. 재현성은 독점 시뮬레이터/기밀 데이터로 제약. |
| Johnson·Calise, Limited Authority Adaptive Flight Control for Reusable Launch Vehicles **JGCD 2003** | 4·4.5·4.5·4·3.5·3·4·4.5 | **82** | strong | T3 | rel:M. Foundational paper introducing Pseudocontrol Hedging (PCH); genuinely novel reference-model modification with full UUB proof and high-influence legacy. Validation high-fidelity but single-vehicle, no Monte Carlo, qualitative baseline; reproducibility weak (proprietary MAVERIC/X-33). C4 worst-case/robust framing only. |
| Trotta et al., Tracking Filter Integration in the Adaptive Augmenting Controller of a Launch Vehicle **JGCD 2021** | 2.5·3·3.5·4·3.5·4·3·2.5 | **63** | fair | T3 | NASA AAC + RLS-adaptive notch 결합. 검증·재현성은 견실하나 신규성·영향력은 조합 수준으로 제한됨. 결합 시스템 안정성 증명 부재. |
| Simplício et al., Launcher flight control design using robust wind disturbance observation **Acta Astronautica 2021** | 3.5·3.5·4·4.5·3.5·4·4·3 | **74** | good | T4 | Weighted: (4*17+3*15+4*16+3*9+4*14+2*8+3*9+3*12)/100=3.39 -> *20=67.8 -> 68. C4 scored (deterministic robust observer + mu/SSV worst-case = mid; no chance/distributional modeling). |
| Ge et al., Adaptive multivariate reusable launch vehicles reentry attitude control with pre-specified performance in the presence of unmatched disturbances **AST 2024** | 4·3·3.5·3.5·3·4·3·3 | **67** | good | T3 | Incremental combination of an existing multivariable AFO and a prescribed-performance SMC; single-scenario sim, one baseline, deterministic bounded-disturbance only; oversized control magnitudes and weak interconnection analysis. |
| Luo et al., Adaptive finite-time prescribed performance attitude tracking control for reusable launch vehicle during reentry phase: An event-triggered case **ASR 2022** | 3.5·2.5·3.5·3.5·3.5·4·3·2.5 | **64** | fair | T5 | Combination paper. See review_md. |
| Sarkar et al., Re-entry trajectory tracking of reusable launch vehicle using artificial delay based robust guidance law **ASR 2021** | 4·2.5·4·3.5·3·4·3.5·2.5 | **67** | good | T5 | TDC/TDE transplanted from robotics to RLV re-entry attitude tracking; clean UUB proof and reproducible setup, but single qualitative baseline and deterministic uncertainty model. |
| Li et al., Adaptive attitude tracking control for reusable launch vehicle with actuator and sensor faults **AST 2025** | 4.5·4·4·3.5·3·4·3·3.5 | **75** | strong | T3 | C4 scored (deterministic bounded-worst-case + AG/NAG threshold + adaptive estimates; no distributional/chance treatment → mid). Crowded RLV FTC niche deflates C2/C8. Delta = differentiable sensor-fusion-into-dynamics for simultaneous actuator+sensor FTC. TOTAL recomputed from 8 scores: (4*17+3*15+4*16+3*9+3*14+4*8+3*9+3*12)/100=3.41*20=68. |
| Farì et al., Loss-of-Efficiency Fault Detection and Diagnosis for Launch Vehicle Thrust Vector Control Systems **SysTol 2025** | 3.5·3·3.5·3·3.5·3·3.5·3 | **66** | good | Conf | nullspace 기반 모델형 FDD를 EMA-TVC의 LoE(효율손실) 고장에 적용(Ossmann/Varga FDITOOLS 수입). delta: vehicle-induced load τ_g를 unknown-disturbance가 아니라 measurable 입력으로 두고 invariant-zero(s0=0) 강건검출로 decouple + ε̂=r/τ_m로 효율 정량화. 액추에이터-로컬로 한정(불안정·mass-varying vehicle-level은 "may not be applicable"이라 명시 회피), τ_g 완전기지 가정, 외부 baseline 부재로 C2/C4 deflate. MC 500(선형)+100(고충실 비선형 MPT-L6)·2 고장케이스(부분 LoE·jamming). |
| Chai et al., Attitude tracking control for reentry vehicles using centralised robust model predictive control **Automatica 2022** | 4·3.5·4·4·4·4·4.5·3.5 | **78** | strong | T1 | rel M. Tube-based centralised robust MPC for reentry attitude; deterministic worst-case (C4 mid); HIL-validated with two baselines; combination work in crowded field deflates C2/C8. |
| Stott·Shtessel, Launch vehicle attitude control using sliding mode control and observation techniques **J. Franklin 2012** | 3·3·3.5·3·2.5·2·3·3 | **59** | fair | T3 | rel: M. Incremental application of Levant quasi-continuous HOSM + prior SMC-SMDO to launch-vehicle attitude control; single 6DOF sim case, thin baselines, no original proofs. |
| Clement et al., Aerospace launch vehicle control: a gain scheduling approach **CEP 2005** | 4·4·4·3.5·3·1.5·3·3 | **69** | good | T3 | rel:M. Industrially grounded launcher autopilot (CNES/EADS), but theory recycled from authors' own prior work; validation undercut by confidentiality (no numbers, no baselines, axis-less figures). |
| Meng et al., Fixed-time attitude control of reusable launch vehicles utilizing reliability-based control allocation **CEP 2024** | 4·3.5·4·3.5·3·4·3·3 | **70** | good | T3 | Rigorous fixed-time proofs but heavily incremental stack; the one real delta (BIM reliability-based allocation) is validated circularly against its own reliability model with weak baselines. |
| Wang et al., Robust adaptive backstepping control for reentry reusable launch vehicles **Acta Astronautica 2016** | 4·2.5·4·3.5·3·2·2.5·3 | **64** | fair | T4 | Combination of backstepping + adaptive bound estimation + VSC; GAS proven only for discontinuous law (implemented sat version is UUB, acknowledged); single-scenario sim vs one self-built SMBC baseline; adaptive laws lack leakage/projection (estimates drift, see Figs 8-9); g(Omega)^-1 conditioning unaddressed. |
| Efremov et al., Adaptive flight control system for flight safety improvement in reentry and other high-velocity vehicles **Acta Astronautica 2023** | 2.5·2.5·3·3·3.5·2·3.5·2.5 | **57** | fair | T4 | rel:M. NDI + online batch least-squares identification (replay buffer + rank check) + modified SAAB rate limiter; simulator-only, 3 operators, no closed-loop adaptive stability proof. Combination work in a crowded PIO/adaptive field. |
| Zhang et al., Ascent control of heavy-lift launch vehicle with guaranteed predefined performance **AST 2021** | 4·3·4·3.5·3·4·2.5·3 | **68** | good | T3 | Combination of predefined-time PPF + predefined-time SMC + adaptive disturbance-bound estimation for HLV ascent; rigorous Lyapunov proofs but incremental delta over authors' own and Sanchez-Torres predefined-time line. C4 deterministic worst-case/adaptive-bound (mid). C7 deflated: high computation acknowledged but deferred. |
| Tian et al., Multivariable uniform finite-time output feedback reentry attitude control for RLV with mismatched disturbance **J. Franklin 2018** | 4·4·4·4·3.5·4·3·3 | **74** | good | T3 | rel M. Weighted: (4*17+3*15+4*16+3*9+3*14+3*8+3*9+3*12)/100=3.33; x20=66.6 -> 67. C4 scored (deterministic worst-case bound + observer-based disturbance estimation + inertia Monte Carlo = mid). Good band. Rigorous dual-theorem Lyapunov+bi-limit homogeneity proofs lift C1; combination of imported tools (bi-limit [36], arbitrary-order differentiator [38], output feedback over own [31]) deflates C2/C8; single-trajectory sim with one trial-and-error-tuned ADRC baseline and no quantitative tables caps C5. |
| Zhang et al., Fixed-time extended state observer based non-singular fast terminal sliding mode control for a VTVL reusable launch vehicle **AST 2018** | 4·3.5·4·3.5·3·4·3·3 | **70** | good | T3 | Combination work (FxTESO + observer-built NFTSM surface) in a saturated RLV fixed-time SMC niche; complete Lyapunov proofs but heavy reliance on cited lemmas, single 6-DOF scenario with one baseline (SFTSMC), no Monte Carlo / actuator dynamics / B-matrix invertibility analysis. C4 scored (deterministic bounded-disturbance observer = mid). |
| Zhang et al., Adaptive fault-tolerant control for a VTVL reusable launch vehicle **Acta Astronautica 2019** | 4·3·3.5·3·3·4·3·3 | **66** | good | T4 | FxTESO+NFTSMS+adaptive-gain 조합형 자세 FTC. 기술적으로 무난하나 구성요소가 모두 선행과 근접해 delta가 좁고, 불확실성은 결정론적 단일 시나리오로만 처리. |
| Liang et al., Fixed-time observer based fault tolerant attitude control for reusable launch vehicle with actuator faults **AST 2020** | 4·2.5·3.5·3·2.5·3·3·2.5 | **61** | fair | T3 | Fixed-time disturbance/fault observer feeding a finite-time NTSM attitude FTC for a VTVL RLV reentry; combination work with modest scoped delta in a crowded RLV-FTC niche, single-scenario validation against one cross-class baseline. |
| Liang et al., Quaternion observer-based sliding mode attitude fault-tolerant control for the Reusable Launch Vehicle during reentry stage **AST 2022** | 4·3·4·3·3·4·3·3 | **68** | good | T3 | Quaternion ESO-type observer + integral terminal SMC FTC for RLV reentry; norm-preserving observer is a mild delta in a crowded niche, single-scenario validation against one baseline. |
| Tian et al., Adaptive prescribed performance attitude control for RLV with mismatched disturbance **AST 2021** | 4.5·3.5·4·3.5·2·4·3·3 | **69** | good | T3 | Disturbance-bound-independent prescribed region for mismatched+matched disturbance is a scoped but modest delta in a crowded PPC+backstepping niche; discontinuous sign-based law yields heavy chattering (control torque oscillates at +/-1e5 N*m), no baseline comparison, single deterministic sinusoidal disturbance profile. |
| Gu et al., Performance improvement-oriented reentry attitude control for reusable launch vehicles with overload constraint **ISA Transactions 2022** | 3.5·3·3.5·4·3.5·4·3·2.5 | **67** | good | T3 | Funnel control variant with switching/piecewise boundary + overload-constraint embedding + RCS/rudder allocation. Solid but incremental combination of known FC, ISS, and PWM allocation; deterministic bounded-disturbance design with stochastic-only validation. Scored 3 across the board → 60/100 (fair). |
| Sarkar et al., Finite time fractional order ISMC with time delay estimation for re-entry phase of RLV with enhanced chattering suppression **Acta Astronautica 2023** | 3.5·2.5·3.5·3·3·3.5·3·2.5 | **61** | fair | T4 | Combination work (TDE + fractional-order integral SMC + homogeneity-based finite-time), heavily built on authors' own prior RLV/TDE papers [22,23,24]; single nominal X-33 trajectory, MATLAB-only, no Monte Carlo/dispersion, deterministic slowly-varying-uncertainty assumption; proposed method not Pareto-dominant (PD-TDC beats it on ISE and TV). C2/C8 deflated for incremental combination. |
| Hervas·Reyhanoglu, Thrust-vector control of a three-axis stabilized upper-stage rocket with fuel slosh dynamics **Acta Astronautica 2014** | 3.5·3·4·2·2.5·3·2.5·3 | **61** | fair | T4 | C4 marked '–' (renormalized over 7 criteria): controller is deterministic-by-design on a nominal known slosh model — no disturbance/fault/parametric-uncertainty or robustness claim; authors explicitly leave time-varying/uncertain slosh parameters to future work. Solid 3D nonlinear Lyapunov+LaSalle design but validation is a single scenario/single IC with only an uncontrolled baseline. |
| Zhang et al., Compensated acceleration feedback based active disturbance rejection control for launch vehicles **CJA 2024** | 4·3.5·4·4·4·4·3.5·3.5 | **76** | strong | T4 | M |
| Costa et al., A reinforcement learning approach for adaptive tracking control of a reusable rocket model in a landing scenario **Neurocomputing 2024** | 3·2.5·3·2.5·2.5·3·3·3 | **56** | fair | T3 | Off-the-shelf linear-quadratic Q-learning + Youla + cascade SISO applied to a planar Falcon-9 landing model; RL merely matches LQG/PID baselines on a single deterministic trajectory, with no Monte Carlo, no new guarantees, and negligible demonstrated advantage. |
| Xueyun et al., Optimal trajectory-tracking guidance for reusable launch vehicle based on adaptive dynamic programming **EAAI 2023** | 3.5·3·3.5·2.5·2.5·3·3.5·2.5 | **61** | fair | T3 | Single-critic ADP tracking + GPM feedforward with rate/accel saturation; strong compute case but single deterministic scenario and incremental, citation-deferred stability. |
| Iafrate et al., Propulsive landing of launchers' first stages with Deep Reinforcement Learning **Acta Astronautica 2025** | 2·3.5·3.5·4·4·4·3·3.5 | **67** | good | T4 | PPO 기반 model-free RL을 Falcon-9급 1단 추진착륙(3-DOF/6-DOF)에 적용한 응용 논문. 이론적 보증 부재, novelty는 reward shaping 진행과 open-source 6-DOF 환경에 국한. SCVX 대비 propellant ~60% 초과로 고전 최적화에 열위지만 광범위한 disturbance/Monte Carlo robustness 분석은 실용적 강점. |
| An et al., Performance-prescribed optimal neural control for hypersonic vehicles considering disturbances: An adaptive dynamic programming approach **AST 2024** | 4·3·3.5·3.5·3·4·3·3 | **67** | good | T3 | Competent but incremental combination (PPC error transform + Levant-type HOSM disturbance observer + ADP actor-critic) in a very crowded HSV PPC+ADP niche; "stochastic disturbance" framing is unsupported (sims use deterministic sinusoids), only SGUUB achieved. Full parameters given, no code/HIL/Monte-Carlo. C4=3 as deterministic observer mid-anchor. |
| Navarro-Tapia et al., The VEGA launcher atmospheric control problem: A case for linear parameter-varying synthesis **J. Franklin 2022** | 3·3.5·4·4·4.5·2·3.5·3 | **71** | good | T3 | Industrial-grade LPV TVC case study on VEGA with strong high-fidelity MC validation, but no new theory and partly a consolidation of prior conference work. |
| Acquatella et al., Guidance command generation and nonlinear dynamic inversion control for reusable launch vehicles **Acta Astronautica 2020** | 3·3·3.5·1.5·2.5·2·2.5·3 | **55** | fair | T4 | Competent but incremental DLR integration paper: offline trajOpt/MOPS + MODELICA-FMU guidance + textbook two-loop NDI on the AURORA HTHL concept; nominal-only descent sim, no robustness, no controller baseline, proprietary toolchain. |
| Mao et al., Attitude controller design for reusable launch vehicles during reentry phase via compound adaptive fuzzy H-infinity control **AST 2018** | 4·3·4·3.5·2.5·4·3·3 | **67** | good | T3 | Competent Lyapunov-rigorous adaptive-fuzzy/H-infinity attitude controller; novelty limited to a region-split blending of a known idea, deterministic-bounded uncertainty, sim-only with a single self-built baseline. |
| Ren et al., Optimal fault-tolerant control of reusable launch vehicle with actuator faults using zero-sum differential game theory **AST 2025** | 3.5·3·3.5·3.5·2.5·3·2.5·3 | **62** | fair | T3 | Zero-sum differential game + critic-only ADP for RLV reentry attitude FTC; complete HJI/Lyapunov UUB theory but incremental combination, and the only empirical comparison is no-FTC ablation (no Table-1 baseline run). rel M. |
| Zhang et al., Fixed-time disturbance observer-based nearly optimal control for reusable launch vehicle with input constraints **ISA Transactions 2022** | 4·3·4·3.5·3.5·4·3·3 | **70** | good | T3 | Competent two-loop combination (AMGST outer + FTDO/ADP-NOC inner) with full Lyapunov proofs and good ablation, but the delta is incremental in a crowded RLV-reentry literature and validation is a single deterministic scenario. |

### 분포 요약 (153편)

flagship(85+) **4** · strong(75–84) **44** · good(65–74) **74** · fair(55–64) **29** · weak(45–54) **1** · poor(<45) **1**. 중앙값 ~70 (전체 153편). 최상위: ct-SCvx **87** · ACB-convex **86** · Entry-Guidance **86** · Impl-Demo **85** · Szmuk-6DoF **84**.

**재교정 핵심 변경:** (1) 임의 하향 fudge 복원 — Ploen 80→90, ct-SCvx 80→87, Ignition 64→71, Auto-SCvx 71→79, BPN-PC/explicit-dispersion/IJCAS/DR-CS/Operator-Splitting 등 ±5~10 복원. (2) 상향 fudge·venue offset 제거 — GPU-SCP 67→59, intelligent-transition 78→67(RL 전환 순delta 미입증), zoRO 71→62, Dual-GP 66→56, ACI 88→79(top-venue 후광 제거). (3) all-4 인플레 해소 — Uncertainty-SKF·Salted-KF 등 증분 IROS/조합 논문의 C2·C8 deflation.
