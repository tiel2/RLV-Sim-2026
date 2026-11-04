# Project 2 구간별 물리 실험 결과

2026-11-03. 현재 목적은 **지정한 초기조건과 입력에서 구간별 운동·하중·추진제·구동기 응답을
확인하는 것**이다. 대표 phase는 **상승·재진입 연소·공력 하강·최종 동력 하강·착륙의 5개**다.
독립 실행하며, 앞의 네 지정 입력 실험은 접지를 요구하지 않고 착륙은 연착륙을 요구한다.
CRS-11 RMSE 감소는 합격 조건이 아니며 비교는 기본 OFF다.

이 문서는 다섯 phase의 현재 결과를 요약한 수동 작성 문서다. 수식·검증 방법은
[영문 검증 보고서](../physics_implementation_report_en.md)에 모은다. 연결된 생성 데이터·그림·실행 설정은 Git에서 제외하는
`results/verification/`에 있으며 아래 재현 명령으로 만든다. 이전 생성 결과도 로컬 보관한다.

## 실행 결과

[실험 명세](../../reference/project2_scenario_spec.md)의 다섯 `project2_*`를 실행했다.
네 지정 입력 실험은 지정 시간까지 진행됐고, 마지막 실제 주 엔진 추력은 0 N이었다.
착륙은 접지 시 종료하며, 접지 시점의 추력 정지를 요구하지 않는다.
현재 다섯 사례는 구면 회전지구를 사용한다. 기준 위도 28.6°는 각 독립 원점에 부여한 가정이며,
상승 원점은 발사점 `[0,0,0]`이다. 고도는 구면 반경 기준이고 속력은 지구 기준이다.

| 실험 | 시간 [s] | 시작 → 종료 고도 [m] | 시작 → 종료 속력 [m/s] | 종료 주 추진제 [kg] |
|---|---:|---|---|---:|
| 상승 | 150 | 0 → 101,055.779 | 0 → 2,171.271 | 20,654.720 |
| 재진입 연소 | 14 | 47,500 → 32,999.569 | 1,260.277 → 944.131 | 9,576.247 |
| 공력 하강 | 45 | 35,000 → 3,758.044 | 953.888 → 310.462 | 8,800.000 |
| 최종 동력 하강 | 22 | 4,700 → 1,527.737 | 319.680 → 80.630 | 3,557.695 |
| 착륙 | 약 17.468(접지) | 300 → 0(접지) | 25 → 0.086245 | 1,378.823 |

앞의 네 행은 native 종료 기록이며 착륙 행은 접지 보간 KPI다. 접지 직후의 native 기록과 구분한다.

### 다섯 번째 phase: 착륙

착륙은 기존 PDG를 바탕으로 만든 `scenario=project2_landing`이다. 구면 회전지구·단일 엔진·
ideal 구동기·truth 전달 항법에서 LCvx 유도와 PD 제어를 사용한다. 300 m, 하강 25 m/s,
총질량 29,500 kg에서 독립 초기화한다. 유도 명목 중력에는 표면 원심 가속도를 포함하고,
N/E/D 추종 이득은 위치 `[0.1,0.1,1]`, 속도 `[0.6,0.6,4]`로 설정했다.

이번 실행은 약 **17.468초**에 접지했고, 보간한 접지 속도 **0.086245 m/s**, 기울기 **0.055317°**,
위치 오차 **0.019430 m**, 주 추진제 **1,378.823 kg**, `landing_class=soft`, `success=true`였다.
마지막 native 기록의 속력 0.084560 m/s와 접지 보간 속도를 구분한다.
[착륙 판정](../../../results/verification/project2_segments/physics/landing/verification.json),
[실행 KPI](../../../results/project2_physics/landing/kpi.json)를 보존한다.
구동기·검증 방법은 앞의 지정 입력 실험과 다르다. realistic 구동기나 센서 항법의 착륙 성공을
검증한 결과는 아니다. 초기 수평 위치 `[1,-1] m`, 속도 `[0.1,-0.1] m/s`를 넣은 별도 실행도
접지 속도 0.140369 m/s, 위치 오차 0.098386 m로 연착륙했다.

### 앞의 네 지정 입력 실험

- 상승은 사전 점화·발사대 해제·지정 pitch 입력·throttle 변화·145초 정지 후 관성비행을 보여준다.
- 재진입 연소는 3개 엔진의 점화·감속·추진제 소모·12초 정지와 그 응답을 보여준다.
- 공력 하강은 엔진 OFF와 중립 핀 노출 상태에서 중력·공력에 따른 속도/자세 변화를 보여준다.
- 최종 동력 하강은 중앙 엔진으로 20초간 감속하고 2초간 정지 응답을 관찰한다. 종료 수직 속도는
  반경 기준 down +80.410 m/s로 여전히 하강 중이며, 접지시키는 제어기는 이 실험의 범위에 포함하지 않는다.

이전 33초 연소/35초 실행은 접지 전 상승으로 돌아서 받음각 범위를 벗어났다. 그 실행은
[과거 비교 기록](../../../results/verification/project2_segments/runtime_results.md)에 보존했다. 현재 범위는 하강 중 감속·정지 응답을
해석 가능한 body 공력표 범위 안에서 보여주도록 선택했다. 실비행에 맞춘 피팅은 수행하지 않았다.

## 물리·수치 검사

[판정 요약](../../../results/verification/project2_segments/physics/summary.json)은 다섯 사례 모두 `passed`다.
착륙은 실제 접지·연착륙·유한 상태·quaternion·추진제·질량 수지·공력표 범위에 대한 별도 회귀 검사다.
다음 독립 적분·시간 간격 검사 항목은 앞의 네 지정 입력 실험에 적용한다.

- 설정에서 독립적으로 계산한 Newton–Euler 식, 회전행렬 자세, 원통 질량 적분, realistic
  throttle/TVC 및 중립 핀 힘·모멘트의 전 구간 DOP853 적분과 비교한다. 회전지구 기준 해는
  관성좌표계에서 직접 전파한 뒤 지구고정 상태로 변환하므로 native의 자전 항을 복제하지 않는다.
- 기준 적분 tolerance를 줄인 결과와 2/1/0.5 ms native 시간 간격 비교를 확인한다.
- 질량 bookkeeping, 추진제 비음수, 질량 증가 없음, quaternion norm, CG/관성, 비활성 엔진 힘,
  RCS 예산, 실제 추력 정지와 body 공력표 범위를 확인한다.

| 실험 | 최대 받음각 [deg] | body 표의 받음각/Mach 범위 | 실험 실행 완료 |
|---|---:|---|---|
| 상승 | 0.968 | 통과 | 예 |
| 재진입 연소 | 0.808 | 통과 | 예 |
| 공력 하강 | 0.766 | 통과 | 예 |
| 최종 동력 하강 | 0.726 | 통과 | 예 |

모든 사례의 질량 bookkeeping 오차는 0 kg이고 RCS 소모도 0이다.
네 지정 입력 실험의 독립 적분 및 시간 간격별 오차·허용값·해시·설정은 `physics/<case>/verification.json`과
`convergence.json`에 기록한다. 이는 **선언한 모델의 물리·수치 구현 검사**다. 실기체 공력·질량·
추진 모델의 경험적 정확도나 항법 정확도를 검증한 것은 아니다. 특히 상승의 고고도 대기식
연장, 구면 지구의 J2·중력경사 토크 생략, 가정한 기체 계수는 모델 한계로 유지한다.

별도의 원·타원궤도, 관성계 보존량 및 비대칭 무토크 회전 검사도 통과했다.
[회전지구 검증 및 자전 ON/OFF 비교](../physics_implementation_report_en.md#11-spherical-rotating-earth-verification)를
참조한다. 동일한 구면·지구 기준 초기조건에서 자전율만 0으로 바꾸면 상승 최대 위치 차이는
약 4.03 km다. 이는 현재 지정 입력의 연성 응답이며 일반적인 궤적 보정값은 아니다.

## 그림과 데이터

| 실험 | 물리 이력 | 궤적·자세·추력 | 독립 적분 오차 |
|---|---|---|---|
| 상승 | [그림](../../../results/verification/project2_segments/physics/ascent/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/ascent/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/ascent/independent_errors.png) |
| 재진입 연소 | [그림](../../../results/verification/project2_segments/physics/entry_burn/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/entry_burn/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/entry_burn/independent_errors.png) |
| 공력 하강 | [그림](../../../results/verification/project2_segments/physics/aero_descent/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/aero_descent/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/aero_descent/independent_errors.png) |
| 최종 동력 하강 | [그림](../../../results/verification/project2_segments/physics/powered_descent/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/powered_descent/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/powered_descent/independent_errors.png) |
| 착륙 | [그림](../../../results/verification/project2_segments/physics/landing/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/landing/plots/episode_summary.png) | 적용하지 않음(폐루프 접지 회귀) |

각 폴더의 `physics.npz`는 실제 실행 상태·하중·추진제 등의 데이터이고 네 지정 입력 사례의
`independent_reference.npz`는 독립 수식 적분 결과다. 여기서 reference는 수학적 검증 기준이며 CRS-11 관측값이 아니다.
리플레이와 더 상세한 구동기 기록은 `results/project2_physics/<case>/`에 저장한다.

다섯 리플레이의 초기·중간·종료·역탐색을 포함한 [WebGL 검사 25개](../../../results/verification/project2_segments/physics/browser_checks.json)가
통과했다. [최종 동력 하강 종료 화면](../../../results/verification/project2_segments/physics/figures/project2_powered_descent_22s.png)을 보존했다.
브라우저 검사는 기록된 자세·형상·구동기 일치를 확인한다. 배경 지면은 원점 접평면의 시각 근사이며
구면 접촉 검증은 네이티브 물리 검사에서 수행한다.
회전지구 착륙 추가 후 전체 회귀 테스트 **400개**, 다섯 phase 검사와 Ruff가 통과했다.

## 재현

```bash
.venv/bin/python scripts/run_single.py scenario=project2_powered_descent hydra.run.dir=results/project2_physics/powered_descent
.venv/bin/python scripts/run_single.py scenario=project2_landing hydra.run.dir=results/project2_physics/landing
.venv/bin/python scripts/check_project2_segments.py
.venv/bin/python scripts/check_rotating_earth.py
.venv/bin/python scripts/view_replay.py results/project2_physics/powered_descent
```

다른 구간은 `project2_ascent`, `project2_entry_burn`, `project2_aero_descent`를 선택한다.
착륙만 검사하려면 `scripts/check_project2_segments.py --scenario landing`을 사용한다.
이 기본 실행과 물리 검사에는 웹캐스트 데이터 비교가 필요하지 않다. 접지가 없는 실행의
`kpi.json success=false`는 착륙 평가용 값이며 물리 실험의 실패를 뜻하지 않는다.
물리 실험의 판정은 위 `verification.json` / `summary.json`을 사용한다.

선택적으로 자료와 비교하려면 `scenario.reference.enabled=true` 또는 검사 스크립트의
`--compare-flight`를 명시한다. 선택 비교 결과는 물리 합격 판정과 분리되며,
[자료의 오차·처리 한계](../../reference/crs11_flight_reference.md)를 함께 해석한다.
