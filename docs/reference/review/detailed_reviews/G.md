# Thread G 비판 리뷰 — 바람/환경 사양 1편

> 바람 확률과정 모델 1편(Dragonfly-gust-spec).
> 공정성: 알고리즘이 아니라 사양·데이터 논문 → C1(이론엄밀)·C2(신규성)·C7(온보드) = N/A. C3(모델 타당성)·C4(난류 충실도)·C5·C6·C8만 채점한다.

---

## (2022, PSS, Turbulence for extraterrestrial aviation: Gust specification for Dragonfly's powered flights: Lorenz)

**1. 핵심 아이디어.** 지상 항공기 감항 인증에 쓰이는 경험적 돌풍/난류 규격(MIL-STD-1797, Dryden/von Karman)을 토성 위성 Titan의 저층 대기에 맞게 일차원리 물리 스케일링으로 재계수화하여, NASA Dragonfly 회전익 착륙선의 동력비행 설계 검증(V&V)용 돌풍 규격을 제시한다. 이산(discrete) 돌풍 모델과 연속(continuous) 난류 모델을 모두 제공한다. 본질적으로 G&C 알고리즘이 아니라, 비행제어계가 견뎌야 할 외란(돌풍) 환경 모델을 규정하는 환경-사양 논문이다.

**2. 알고리즘.** 대류속도 스케일 w*=(F g Z / rho Cp T)^0.33. 돌풍 강도 sigma_g = sigma_u = sigma_v = [(0.2 U)^2 + (0.6 w*)^2]^0.5 (기계적 난류 0.2U와 대류 성분의 RSS). 연속 난류는 1차 자기회귀(AR(1)) 랜덤워크: Ug[n] = exp(-k delta) Ug[n-1] + (2 k delta)^0.5 sigma_g N(0,1), 여기서 k ~ U/L (전진비행) 또는 V/L, L은 적분 길이 스케일. 이산 돌풍은 코사인 형상: Ug = (Um/2)[1 + cos(pi t / tg)] (−tg < t < tg). Titan용 대표값: U=2 m/s, w*~0.13 m/s → sigma_g≈0.41 m/s, L=40~400 m, 이산 horizontal gust Vg=0.5~2.0 m/s.

**3. 비판적 검토.**
- 정리·증명이 없는 물리 스케일링/공학 사양 논문. 핵심 수식(Dryden/von Karman 스펙트럼, Deardorff 스케일, MIL-STD 0.1U 규칙)이 모두 지상 문헌에서 import된 것이며, 저자 고유의 엄밀한 유도는 없다(C1).
- 검증이 단일 LES 1개 사례(Lavely 2020)와 Dryden/vK 스펙트럼 비교(Fig.4,5)에 그친다. 실측 데이터(Titan 미방문), 차량 응답 Monte Carlo, 외부 baseline이 부재. 사실상 self/문헌 baseline이다(C5).
- L=40~400 m, 위험 임계(99%, 90% exceedance), worst-case wind 등 다수 파라미터가 "보수적으로 임의 선택"임을 저자도 인정 — 사양으로서는 솔직하나 정량적 근거가 약하다.
- AR(1) 모델이 고주파에서 LES보다 과대 변동(보수적)임을 인정하나, 이를 정량적 보수성 마진으로 환산하지는 않았다.

**4. 시뮬/실험.** Titan 대류 PBL 깊이 Z≈300 m(아침)~900 m(오후), 표면 sensible heat flux로 w*~0.13 m/s 도출. 연속 모델을 Lavely 2020 LES 1개 run(U=0.5 m/s, L=40 m, sigma=0.11 m/s)과 시계열 비교(Fig.4): 전반적 진폭 추세는 재현하나 고주파 변동 과다(보수적)(sim). PSD 비교(Fig.5)에서 AR(1)이 Dryden(−2)~von Karman(−5/3) 기울기와 정성적 일치(sim). 정량 사양 산출: 50 Titan일·~5000분 비행 중 1회 초과 기준 → discrete gust 0.5 m/s(near-surface), 순항 traverse는 ~7배 경로로 보정. 실제 비행/HIL 데이터 없음.

**5. Contribution(delta).** 출판년(2022) 기준 delta: 외계 대기(Titan) 동력비행을 위한 최초의 명시적 돌풍·난류 사양을 일차원리 스케일링으로 정립. 선행은 EDL용 연속 돌풍(Mars MER/Phoenix의 TKE-기반 mesoscale perturbation, Kass 2003 등)에 국한되었고 동력비행·회전익용 사양은 부재했다. 새로움은 "적용 도메인의 확장(지상→외계)과 명문화"에 있으며, 방법론 자체는 확립된 모델의 재계수화로 methodological delta는 얇다.

**6. Limitation·Future work.** 저자: 사양이 Titan 저위도(<20도)·equinox 시기에 한정; 극지/저위도 equinox의 습윤 대류 폭풍은 별도 모델 필요; 지형 강제(능선·사구 압축)는 초기 동력비행에만 위협이며 이후는 quasi-deterministic; EDL(>1000 km 고도 범위)로 확장 시 추가 정교화 필요. 내가 보는 한계: (i) 차량 폐루프 제어계 응답에 대한 Monte Carlo·보장이 전무 — 환경 측 불확실성만 다룸. (ii) baseline이 단일 LES로 통계적 신뢰가 약함. (iii) G&C 알고리즘이 아니므로 G&C 커뮤니티 직접 채택 경로는 간접적(설계 검증 입력으로서만 기여).

**평가:** 56/100 (fair) · 한줄평: Dragonfly 실제 비행제어 V&V에 직접 쓰이는 견실하고 솔직한 외계 돌풍 사양이나, G&C 방법론이 아닌 환경-모델 기여로 도메인 관련성은 enabling-input 수준에 한정된다.
