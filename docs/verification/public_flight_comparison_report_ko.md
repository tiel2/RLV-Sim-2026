# CRS-11 공개 비행 데이터와 시뮬레이션 비교

[English](public_flight_comparison_report_en.md)

공개된 CRS-11 flight data를 바탕으로 다섯 개의 phase를 구성하고, 계산된 궤적의 차이와 모델 제약 만족 여부를 확인했다. 구간별 고도 RMSE는 약 51–599 m, 속력 RMSE는 11–66 m/s다. 착륙은 특정 초기조건과 이상적 구동기에서 성공했지만 관측 속력의 후반을 재현하지 못했다.

관측에 근거한 초기조건과 가정한 단순 명령으로 계산한 궤적을 비교하였다. 관측 오차를 최소화하도록 각 구간의 비행 방향·자세·명령 이력을 함께 설계한 궤적 최적화나 관측 추종 제어는 수행하지 않았다. 

## 1. 관측에서 궤적을 구성한 과정

자료는 CRS-11 웹캐스트 화면의 시간·고도·속력 숫자를 추출한 것이다. 실제 비행에서 유래했지만 센서 원시 로그는 아니며, 계획 궤적·목표 상태·제어 입력·완전한 위치·자세는 제공되지 않는다. [출처](../reference/crs11_data_provenance_ko.md)와 [관측·초기조건 처리 기준](../reference/crs11_flight_reference.md)에 근거를 정리했다.

1. **초기 고도·총속력:** 구간 시작의 표본을 사용하고 필요하면 인접 유효 표본 사이에서 보간했다. 모델의 구면 고도·지구 상대 속력과 같은 기준이라고 가정하고, 모의 시간 0을 구간 시작에 맞췄다. 원본 기준면·속력 좌표계는 미확인이다.
2. **나머지 초기 상태:** 고도 변화로 수직 속도를 추정하고 총속력으로 수평 속력을 구했다. 수평 방향·위치·기체 자세·각속도는 별도로 가정했다. 착륙은 저고도 표시 특성을 고려한 다른 방법을 사용했다.
3. **모델과 입력:** 기체 모델과 기존 시나리오의 명령 형태를 채택하고, 일부 스로틀·짐벌 크기·질량 후보를 반복 계산해 비교했다. 각 실행의 위치·속도·자세는 병진·회전 운동방정식으로 계산했다. 처음 네 구간에는 관측에서 벗어날 때 이를 수정하는 제어가 없고, 착륙은 별도로 정한 지상 목표를 향해 제어한다.

다섯 구간은 **독립 초기화한 실행**이다. 회복 구간은 앞 실행의 종료 질량·RCS 가스를 다음에 전달하지만 위치·속도·자세를 연결한 전 비행은 아니다. 애니메이션의 방향·자세는 가정과 모델 계산 결과이며, 공개 고도·속력만으로 실제 3D 경로·자세의 일치 여부를 검증할 수 없다.

## 2. 최종 설정

### 초기조건과 모델

| 구간 | 발사 후 분석 시간 [s] | 시작 고도 [m] | 시작 총속력 [m/s] | 시작 질량 [kg] |
|---|---:|---:|---:|---:|
| 상승 | 0–145 | 0 | 0 | 545,200.0 |
| 재진입 연소 | 372–384 | 47,500 | 1,260.3 | 40,000.0 |
| 공력 하강 | 384–429 | 35,000 | 953.9 | 34,996.8 |
| 최종 동력 하강 | 429–452.485 | 4,700 | 319.7 | 34,996.8 |
| 착륙 | 452.485–464.630의 관측 사용 | 300 | 76.1 | 31,998.6 |

재진입 연소·공력 하강·최종 동력 하강은 직전 **4초**의 고도에 시간 가중 2차식을 맞춰 시작 수직 속도를 추정했다. `수평 속력 = √(총속력² − 수직 속도²)`로 분해하고 진행 방향은 북쪽으로 가정했다. 수직 속력이 총속력을 넘는 추정은 거부했다. 초기 자세는 추력축이 속도의 반대를 향하도록 설정했다.

상승은 정지한 수직 기체, 초기 방위각 45°에서 시작한다. 착륙은 수직 자세·방위각 0°다. 모든 구간의 초기 롤각·각속도는 0이다. **방위각·초기 자세·각속도는 계산을 위해 선택한 조건으로, 실비행 관측으로 확인하거나 오차를 최소화하도록 결정한 값이 아니다.** 착륙을 제외한 구간은 각 시작점의 지상 투영점을 원점으로, 북쪽·동쪽 위치를 0으로 두었다. 실제 위도·경도를 복원한 좌표가 아니다.

모델에는 지구 자전, 고도에 따른 중력·대기, 기체·핀 공기력, 연소에 따른 질량 변화를 포함했다. 회복 구조 질량 25,000 kg, 초기 RCS 가스 200 kg, 엔진당 진공 추력 914 kN·비추력 311 s를 사용했다. 질량과 기체 계수는 모델 가정이며 실측값이 아니다. 공력계수는 고정했고 기준 실행에는 바람·센서 잡음·슬로싱을 추가하지 않았다.

### 입력과 제어

| 구간 | 적용 입력 | 명령 형태를 선택한 근거와 한계 |
|---|---|---|
| 상승 | 엔진 9개. 스로틀은 0–50초 **0.908026**, 50–74초 **0.778307**, 74–145초 **0.925023**. 10–12초 짐벌 **0.001869 rad(약 0.107°)**, 이후 0 | 기존 상승 시나리오의 구간별 스로틀·짧은 기울임 명령을 사용. 50·74초는 공개 event를 참고했지만 10–12초 기울임은 공학적 설정. 명령 시각과 형태는 고정 |
| 재진입 연소 | 엔진 3개, 12초 동안 각각 스로틀 **0.484908**. 핀 편향 0 | 공개 연소 구간에 맞춰 단순한 일정 스로틀 감속을 가정. 실제 엔진별 명령 이력은 미확인 |
| 공력 하강 | 엔진 OFF, 핀 전개·편향 0. 짐벌·RCS 명령도 0 | 중립 핀 상태의 공력 운동을 계산. 관측에 맞추는 핀·자세 제어를 구성하지 않음 |
| 최종 동력 하강 | 엔진 1개, 23.485초 동안 스로틀 **0.436689**. 핀 편향 0 | 단일 엔진의 일정 스로틀 감속을 가정. 구간 끝의 관측 고도·속력을 만족시키는 조건은 부여하지 않음 |
| 착륙 | 엔진 1개, LCvx 유도기·PD 제어기. 기울기 제한 **15°**, 이상적 구동기·정확한 모의 상태를 이용한 항법 | 기존 제어기로 가정한 지상 목표에 착륙. 관측 시간 이력의 오차를 줄이는 제어가 아님 |

처음 네 구간은 지정 입력과 구동기 응답 지연·변화율 제한을 사용한다. 스로틀은 0–1의 명령이며 대기압을 반영한 실제 추력 비율과는 다르다. 착륙은 설정한 지상 목표를 향한 폐루프 제어이며, 관측 고도·속력의 시간 이력을 추종하는 제어는 아니다.

**입력 후보를 비교한 범위:** [탐색 설정](../../configs/analysis/flight_reconstruction.yaml)에 따라 상승은 스로틀 3개·짐벌 크기 1개, 회복은 재진입 초기 질량·두 연소 구간의 스로틀만 바꿨다. 각 실행의 고도·속력 RMSE를 각각 100 m·10 m/s로 나눈 값의 제곱을 같은 비중으로 평균해 점수로 사용했다. 회복은 재진입·공력 하강·최종 동력 하강의 점수를 다시 평균했으므로 한 구간이나 한 관측량의 오차가 커져도 선택될 수 있다. 이 가중 척도는 측정 불확실성에서 얻은 값이 아니다.

Powell 수치 탐색은 수행했지만, [상승 기록](../../results/verification/public_flight_comparison/reconstruction_20261107/ascent_search.json)과 [회복 기록](../../results/verification/public_flight_comparison/reconstruction_20261107/recovery_search.json) 모두 각각 **80회·60회 평가 한도에 도달해 종료**됐으며 수렴을 확인하지 못했다. 표의 값은 검사 조건을 통과한 시도 중 점수가 가장 낮았던 값이다. 명령 시각·전체 명령 이력·비행 방향·공력계수·핀 제어는 함께 탐색하지 않았으므로, **flight data의 오차를 최소화한 궤적이나 최적 명령으로 해석할 수 없다.** 공력 하강에는 앞 구간에서 전달된 질량의 간접 영향만 반영됐다.

### 착륙 방향과 목표

고도가 100 m 단위로 표시되어 짧은 차분으로 착륙 초기 방향을 정하기 어렵다. 직전 **6초 동안 하강 속력/총속력 비율이 일정**하다고 가정하고, 속력 적분으로 얻은 고도가 관측과 **100 m 이내**인 비율 범위의 중간값을 택했다. 수평 속력은 **1.242 m/s**, 하강 속력은 **76.101 m/s**다. 100 m는 공학적 허용 차이며 센서 오차 한계가 아니다.

목표는 `수평 속력 × 초기 고도 / 하강 속력`으로 정한 **진행 방향 4.898 m 앞의 지면**이다. 수평·수직 속도가 같은 시간 동안 선형 감소한다는 가정에 따른 목표로, 실제 착륙장이 아니다. 목표를 원점으로 두고 시작점을 남쪽 4.898 m·구면 고도 300 m에 배치했다.

## 3. 관측과의 차이

관측 시각에 모의 값을 보간하고, 인접한 유효 관측 사이의 시간을 가중해 계산한 RMSE다. 초기조건에 사용한 자료·고도 0 표시·1.1초를 넘는 공백은 해당 평가에서 제외했다. 고도와 속력의 유효 여부는 따로 판단한다.

| 구간 | 고도 RMSE [m] | 속력 RMSE [m/s] | 고도 비교시간 [s] | 속력 비교시간 [s] |
|---|---:|---:|---:|---:|
| 상승 | 51.277 | 28.615 | 135.635 | 143.643 |
| 재진입 연소 | 136.654 | 25.959 | 5.939 | 5.939 |
| 공력 하강 | 599.301 | 66.365 | 38.939 | 38.939 |
| 최종 동력 하강 | 273.751 | 32.884 | 17.418 | 17.418 |
| 착륙 | 72.712 | 10.944 | 4.672 | 6.040 |

상승은 약 62 km까지의 고도 변화에서 RMSE 약 51 m를 얻었지만, 마지막 평가 시각의 속력은 관측보다 약 107 m/s 높다. 공력 하강의 차이가 가장 크다. 최종 동력 하강의 종료 상태도 고도 694.693 m·속력 108.030 m/s로, 다음 착륙 초기값 300 m·76.111 m/s와 연결되지 않는다.

착륙은 평가 가능한 고도 관측 4.672초 전부와 속력 관측 12.112초 중 6.040초를 비교했다. 시뮬레이션이 먼저 종료되어 **나머지 6.072초의 속력 관측은 재현하지 못했다.** 종료 뒤 값을 0으로 채우지 않았다.

그림의 검은 점은 관측, 파란 선은 시뮬레이션, 회색은 평가 제외 관측이다. 차이는 모의 값에서 관측값을 뺀 값이다.

![상승 관측과 시뮬레이션](../../results/verification/public_flight_comparison/final_report_20261107/ascent/comparison.png)

## 4. 착륙 결과와 해석

착륙은 시작 후 **6.091초**, 접지 속력 **0.998 m/s**, 설정 목표 오차 **0.455 m**, 잔여 주 추진제 **약 5.33 t**로 연착륙 기준을 만족했다. 실제 접지 시각·속도·위치는 미확인이므로 이 수치는 실제 접지와의 오차가 아니다.

![착륙 관측과 시뮬레이션](../../results/verification/public_flight_comparison/final_report_20261107/landing/comparison.png)

다섯 기준 실행은 기록된 모델 제약과 독립 적분·시간 간격 검사를 통과했다. 하지만 같은 제어기에 초기 방향 ±1°를 적용하면 연착륙 기준을 만족하지 못했고, 추력 −2%에서는 **12.54 m/s**, 응답 지연·변화율 제한 구동기에서는 **24.62 m/s**로 충돌했다. [물리·수치 검사와 민감도](public_flight_sensitivity_ko.md)에 조건과 근거가 있다.

확인한 것은 **가정한 초기조건·단순 명령에서 계산이 일관되는지, 관측과 얼마나 다른지**다. 초기조건이 맞는다는 사실이나 모델 제약 검사 통과만으로 이후 궤적의 실비행 일치도를 보장하지 않는다. 공력 하강의 관측 추종 제어는 구성하지 않았고, 구간 경계 상태 연결·전체 착륙 관측의 추종·실제 구동기의 연착륙 성능도 확보하지 못했다. 같은 자료로 일부 입력을 고르고 평가했으므로 독립 예측 검증도 아니다. 남은 오차에는 가정한 방향·명령과 실제 비행의 차이도 포함되며, 이를 모두 외란·잡음·슬로싱으로 설명할 근거는 없다.

## 5. 실행과 결과 확인

각 구간에서 먼저 시뮬레이션을 실행하고, 다음 명령으로 기존 애니메이터를 연다. `--system`은 `view_replay.py`의 외부 브라우저 옵션이다.

비행 비교 실행에는 **주황 점선의 관측 고도선**만 추가된다. 이는 **같은 시각의 모의 수평 위치에 관측 고도를 놓은 표시**이며 실제 3D 관측 경로가 아니다. 고도 0·결측은 연결하지 않고 모의 종료 이후로 연장하지 않는다. 일반 시나리오에는 표시하지 않는다. 관측선이 화면 밖이면 축소해서 확인하며, 이 기능 추가 전의 `replay.json`은 한 번 다시 생성해야 한다.

상승:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_ascent \
  hydra.run.dir=results/final_flight_repeat/ascent

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/ascent --system
```

재진입 연소:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_entry_burn \
  hydra.run.dir=results/final_flight_repeat/entry_burn

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/entry_burn --system
```

공력 하강:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_aero_descent \
  hydra.run.dir=results/final_flight_repeat/aero_descent

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/aero_descent --system
```

최종 동력 하강:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_powered_descent \
  hydra.run.dir=results/final_flight_repeat/powered_descent

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/powered_descent --system
```

착륙:

```bash
.venv/bin/python scripts/run_single.py \
  scenario=flight_reconstructed_landing \
  hydra.run.dir=results/final_flight_repeat/landing

.venv/bin/python scripts/view_replay.py results/final_flight_repeat/landing --system
```

### 결과 파일

| 구간 | 관측과 시뮬레이션 비교 그림 | 최종 수치·설정·검사 결과 |
|---|---|---|
| 상승 | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/ascent/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/ascent/comparison.pdf) | [상승 결과](../../results/verification/public_flight_comparison/final_report_20261107/ascent/summary.json) |
| 재진입 연소 | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/entry_burn/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/entry_burn/comparison.pdf) | [재진입 연소 결과](../../results/verification/public_flight_comparison/final_report_20261107/entry_burn/summary.json) |
| 공력 하강 | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/aero_descent/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/aero_descent/comparison.pdf) | [공력 하강 결과](../../results/verification/public_flight_comparison/final_report_20261107/aero_descent/summary.json) |
| 최종 동력 하강 | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/powered_descent/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/powered_descent/comparison.pdf) | [동력 하강 결과](../../results/verification/public_flight_comparison/final_report_20261107/powered_descent/summary.json) |
| 착륙 | [PNG](../../results/verification/public_flight_comparison/final_report_20261107/landing/comparison.png) · [PDF](../../results/verification/public_flight_comparison/final_report_20261107/landing/comparison.pdf) | [착륙 결과](../../results/verification/public_flight_comparison/final_report_20261107/landing/summary.json) |

그림·최종 수치·설정·검사 결과는 `results/verification/public_flight_comparison/final_report_20261107/`에 있다. [전체 결과](../../results/verification/public_flight_comparison/final_report_20261107/summary.json), [입력 자료 해시](../../results/verification/public_flight_comparison/final_report_20261107/input_manifest.json), [생성 파일 해시](../../results/verification/public_flight_comparison/final_report_20261107/artifact_manifest.json)를 함께 보관했다. `results/`는 git 추적 대상이 아니므로 다른 환경에서는 자료 전달이나 재생성이 필요하다.

[그림 생성 도구](../../scripts/plot_final_flight_comparison.py)는 보존된 검증 기록에서 최종 실행만 표시한다. 새 시뮬레이션이나 입력 탐색은 하지 않는다.

```bash
.venv/bin/python scripts/plot_final_flight_comparison.py \
  --output results/verification/public_flight_comparison/final_report_repeat
```
