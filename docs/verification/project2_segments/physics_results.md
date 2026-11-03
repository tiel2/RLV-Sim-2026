# Project 2 구간별 물리 실험 결과

2026-11-03. 현재 목적은 **지정한 초기조건과 입력에서 구간별 운동·하중·추진제·구동기 응답을
확인하는 것**이다. 실험은 독립 실행하며, 착륙 성공이나 CRS-11 RMSE 감소를 합격 조건으로
사용하지 않는다. CRS-11 비교는 기본 OFF다.

이 문서만 수동 작성 보고서로 유지한다. 연결된 생성 데이터·그림·실행 설정은 Git에서 제외하는
`results/verification/`에 있으며 아래 재현 명령으로 만든다. 이전 생성 결과도 로컬 보관한다.

## 실행 결과

[실험 명세](../../reference/project2_scenario_spec.md)와 네 `project2_*` YAML을 실행했다.
모든 실험이 지정 시간까지 진행됐고, 마지막 실제 주 엔진 추력은 0 N이었다.

| 실험 | 시간 [s] | 시작 → 종료 고도 [m] | 시작 → 종료 속력 [m/s] | 종료 주 추진제 [kg] |
|---|---:|---|---|---:|
| 상승 | 150 | 0 → 99,144.419 | 0 → 2,182.070 | 20,654.720 |
| 재진입 연소 | 14 | 47,500 → 32,995.344 | 1,260.277 → 944.539 | 9,576.247 |
| 공력 하강 | 45 | 35,000 → 3,741.823 | 953.888 → 310.233 | 8,800.000 |
| 최종 동력 하강 | 22 | 4,700 → 1,523.744 | 319.680 → 80.990 | 3,557.695 |

- 상승은 사전 점화·발사대 해제·지정 pitch 입력·throttle 변화·145초 정지 후 관성비행을 보여준다.
- 재진입 연소는 3개 엔진의 점화·감속·추진제 소모·12초 정지와 그 응답을 보여준다.
- 공력 하강은 엔진 OFF와 중립 핀 노출 상태에서 중력·공력에 따른 속도/자세 변화를 보여준다.
- 최종 동력 하강은 중앙 엔진으로 20초간 감속하고 2초간 정지 응답을 관찰한다. 종료 수직 속도는
  NED down +80.761 m/s로 여전히 하강 중이며, 접지시키는 제어기는 이 실험의 범위에 포함하지 않는다.

이전 33초 연소/35초 실행은 접지 전 상승으로 돌아서 받음각 범위를 벗어났다. 그 실행은
[과거 비교 기록](../../../results/verification/project2_segments/runtime_results.md)에 보존했다. 현재 범위는 하강 중 감속·정지 응답을
해석 가능한 body 공력표 범위 안에서 보여주도록 선택했다. 실비행에 맞춘 피팅은 수행하지 않았다.

## 물리·수치 검사

[판정 요약](../../../results/verification/project2_segments/physics/summary.json)은 네 사례 모두 `passed`다. 판정 대상은 다음과 같다.

- 설정에서 독립적으로 계산한 Newton–Euler 식, 회전행렬 자세, 원통 질량 적분, realistic
  throttle/TVC 및 중립 핀 힘·모멘트의 전 구간 DOP853 적분과 비교한다.
- 기준 적분 tolerance를 줄인 결과와 2/1/0.5 ms native 시간 간격 비교를 확인한다.
- 질량 bookkeeping, 추진제 비음수, 질량 증가 없음, quaternion norm, CG/관성, 비활성 엔진 힘,
  RCS 예산, 실제 추력 정지와 body 공력표 범위를 확인한다.

| 실험 | 최대 받음각 [deg] | body 표의 받음각/Mach 범위 | 실험 실행 완료 |
|---|---:|---|---|
| 상승 | 0.963 | 통과 | 예 |
| 재진입 연소 | 0.805 | 통과 | 예 |
| 공력 하강 | 0.765 | 통과 | 예 |
| 최종 동력 하강 | 0.720 | 통과 | 예 |

모든 사례의 질량 bookkeeping 오차는 0 kg이고 RCS 소모도 0이다.
독립 적분 및 시간 간격별 오차·허용값·해시·설정은 `physics/<case>/verification.json`과
`convergence.json`에 기록한다. 이는 **선언한 모델의 물리·수치 구현 검사**다. 실기체 공력·질량·
추진 모델의 경험적 정확도나 항법 정확도를 검증한 것은 아니다. 특히 상승의 고고도 대기식
연장, 고정 NED/수직 중력, 가정한 기체 계수는 모델 한계로 유지한다.

## 그림과 데이터

| 실험 | 물리 이력 | 궤적·자세·추력 | 독립 적분 오차 |
|---|---|---|---|
| 상승 | [그림](../../../results/verification/project2_segments/physics/ascent/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/ascent/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/ascent/independent_errors.png) |
| 재진입 연소 | [그림](../../../results/verification/project2_segments/physics/entry_burn/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/entry_burn/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/entry_burn/independent_errors.png) |
| 공력 하강 | [그림](../../../results/verification/project2_segments/physics/aero_descent/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/aero_descent/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/aero_descent/independent_errors.png) |
| 최종 동력 하강 | [그림](../../../results/verification/project2_segments/physics/powered_descent/plots/physics.png) | [그림](../../../results/verification/project2_segments/physics/powered_descent/plots/episode_summary.png) | [그림](../../../results/verification/project2_segments/physics/powered_descent/independent_errors.png) |

각 폴더의 `physics.npz`는 실제 실행 상태·하중·추진제 등의 데이터이고 `independent_reference.npz`는
독립 수식 적분 결과다. 여기서 reference는 수학적 검증 기준이며 CRS-11 관측값이 아니다.
리플레이와 더 상세한 구동기 기록은 `results/project2_physics/<case>/`에 저장한다.

현재 리플레이의 초기·중간·종료·역탐색을 포함한 [WebGL 검사 20개](../../../results/verification/project2_segments/physics/browser_checks.json)가
통과했다. [최종 동력 하강 종료 화면](../../../results/verification/project2_segments/physics/figures/project2_powered_descent_22s.png)을 보존했다.
전체 테스트는 **383개**, `ruff check src tests scripts`도 통과했다.

## 재현

```bash
.venv/bin/python scripts/run_single.py scenario=project2_powered_descent hydra.run.dir=results/project2_physics/powered_descent
.venv/bin/python scripts/check_project2_segments.py
.venv/bin/python scripts/view_replay.py results/project2_physics/powered_descent
```

다른 구간은 `project2_ascent`, `project2_entry_burn`, `project2_aero_descent`를 선택한다.
이 기본 실행과 물리 검사에는 웹캐스트 데이터 비교가 필요하지 않다. 접지가 없는 실행의
`kpi.json success=false`는 착륙 평가용 값이며 물리 실험의 실패를 뜻하지 않는다.
물리 실험의 판정은 위 `verification.json` / `summary.json`을 사용한다.

선택적으로 자료와 비교하려면 `scenario.reference.enabled=true` 또는 검사 스크립트의
`--compare-flight`를 명시한다. 선택 비교 결과는 물리 합격 판정과 분리되며,
[자료의 오차·처리 한계](../../reference/crs11_flight_reference.md)를 함께 해석한다.
