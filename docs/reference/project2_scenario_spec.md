# Project 2 다섯 대표 phase 명세

2026-11-03. 대표 phase는 **상승·재진입 연소·공력 하강·최종 동력 하강·착륙의 5개**다.
앞의 네 구간은 합리적인 초기조건과 지정 입력으로 물리·수치 구현을 확인하고,
다섯 번째 착륙은 `scenario=project2_landing`의 폐루프 유도·제어로 접지까지 실행한다.
다섯 사례 모두 구면 회전지구를 사용한다.
각 구간의 입력·종료 시간은 공학적 실험 조건이다. CRS-11과 같은 궤적이나 RMSE 감소를
현재 완료 기준으로 두지 않는다. 다섯 구간을 연속 비행으로 연결할 필요는 없다.
착륙 사례는 연착륙이 완료 기준이고, 앞의 네 구간에는 접지를 요구하지 않는다.
센서·항법은 이후 구현하며 Project 2 최종 완료에는 필요하다.

## 1. 다섯 대표 phase

모든 시간은 **구간 시작부터의 로컬 시간**이다. NED 속도의 세 번째 성분은 하강 방향이 양수다.
표의 반올림값은 설명용이며 실행은 YAML에 기록된 정밀도를 사용한다.

| 구간 / 실행 설정 | 초기 고도 [m] | 초기 NED 속도 [m/s] | 총질량 [kg] | 입력과 종료 조건 |
|---|---:|---|---:|---|
| [상승](../../configs/scenario/project2_ascent.yaml) | 0 | `[0,0,0]` | 545,200 | 9개 엔진 지정 입력, 145초 정지 명령, 150초 종료 |
| [재진입 연소](../../configs/scenario/project2_entry_burn.yaml) | 47,500 | `[406.9,0,1192.8]` | 40,000 | 엔진 `[0,1,5]` 각 throttle 0.50, 12초 정지, 14초 종료 |
| [공력 하강](../../configs/scenario/project2_aero_descent.yaml) | 35,000 | `[480.6,0,824.0]` | 34,000 | 주 엔진 OFF, 노출된 중립 핀, 45초 종료 |
| [최종 동력 하강](../../configs/scenario/project2_powered_descent.yaml) | 4,700 | `[142.7,0,286.1]` | 32,000 | 중앙 엔진 throttle 0.55, 20초 정지, 22초 종료 |
| [착륙](../../configs/scenario/project2_landing.yaml) | 300 | `[0,0,25]` | 29,500 | 단일 엔진, 회전지구·폐루프 LCvx 유도·PD 제어, 접지 종료(최대 40초) |

지면에 먼저 도달하면 기존 접지 판정으로 실행이 종료된다. 최종 동력 하강은 감속·연료 소모·
엔진 정지 응답을 보여주는 실험이며, 유도·제어를 설계해 연착륙시키는 사례는 아니다.
기존 `pdg`를 바탕으로 만든 회전지구 착륙을 다섯 번째 대표 phase로 포함한다. 최종 동력 하강의 종료 상태를
착륙 초기 상태로 연결하지 않으며, 두 사례는 각각 독립된 초기조건을 사용한다.

상승 입력은 초기 throttle=1, 10–12초 pitch 짐벌 0.002 rad, 50–74초 throttle=0.8,
그 밖에는 1, 145초 OFF다. 초기 점화 완료를 가정하며 발사대 반력/가속도 조건으로 해제한다.
회복 세 사례는 초기 throttle=0, 전 구간 짐벌·핀 편향·RCS 명령=0이다. 핀 노출은 상승 0,
회복 1이며 기존 순간 노출 근사를 사용한다. 기체 자세는 초기값 이후 실제 6-DOF로 전파한다.
앞의 네 구간은 GNC나 항법으로 정해진 궤적을 추종시키지 않는다. 착륙은 기존 폐루프 GNC와
truth 전달 항법을 사용한다.

## 2. 초기조건과 가정

앞의 네 구간의 고도·속도 값은 앞선 CRS-11 관측 분석에서 얻은 값을 하나의 명목 초기조건 예시로 채택한
것이다. 자세·질량·입력은 가정이다. 필요하면 물리적 제약 안에서 조건을 바꿔 다른 실험을 만들 수
있다. 실행용 YAML은 값을 고정해서 보관하므로 기본 실행에 웹캐스트 자료를 읽을 필요가 없다.

아래 표는 앞의 네 지정 입력 실험에 적용한다. 착륙의 구동기·항법 조건은 3.1절에 별도로 기록한다.

| 항목 | 명목 조건과 의미 |
|---|---|
| 좌표 | 원점에서 NED와 정렬한 지구고정 Cartesian 좌표; `R_f=r+[0,0,-R]`, 고도 `h=norm(R_f)-R` |
| 지구 | 구면 중심중력 + 자전; 각 독립 원점의 기준 위도 28.6°는 공학적 가정이며 실비행 지리 재구성이 아님 |
| 상승 수평 위치 | 발사 지점의 구면 표면 원점 `[0,0] m`; 이전 평면 모델의 착륙 원점 오프셋은 제거 |
| 회복 수평 위치/방위 | 각 시작점의 지상 투영점을 `[0,0]`으로 두고 수평 속도를 합성 북쪽으로 설정; 실제 착륙장까지 거리 아님 |
| 상승 자세 | roll/pitch/yaw=`[0,90,45]°`, 각속도 0 |
| 회복 자세 | 초기 body `+x`를 속도 반대로 정렬; roll=0, yaw=180°, pitch 약 71.163 / 59.749 / 63.490°; 각속도 0 |
| 구동기 | 초기 TVC/핀 각도·각속도 0; 회복은 throttle=0에서 시작 |
| 환경 | 구면 중심중력, 코리올리·전체 원심 가속도, 층별 `ussa_poly`, 무풍·무고장·무슬로시 |
| 센서·항법 | 구성하지 않음; 기록된 상태는 모델이 계산한 simulation truth |

수평 원점이 착륙장이라는 가정이 없으므로 원점 기준 착륙 위치 오차를 실비행 검증 지표로 쓰지
않는다. 회복의 초기 retrograde 자세와 0 각속도는 계측값이 아니다. 고도 기준/속력 기준과
모델 파라미터의 실기체 타당성은 미확정이며, 물리 구현 검증과 구분한다.

`q_bn`은 body→원점 기준 지구고정 축, `w_body`는 그 축에 대한 body 각속도다. 관성계 각속도는
`w_i=w_body+C_bn.T Omega`로 계산해 Euler 식·관성 변화율·분사 각운동량 유출에 사용한다.
quaternion 전파와 공력의 회전 응답은 지구 기준 각속도를 사용한다. `omega_inertial_body`가
향후 gyro 모델에 사용할 회전율 truth다. 원점에서 멀어지면 이 좌표의 D축과 실제 local down이
다르므로 고도·비행경로각·접지 수직속도는 반경 방향으로 계산한다.

## 3. 질량과 기체 모델

| 구간 | 구조 [kg] | 부착 상단 [kg] | RCS [kg] | 주 추진제 [kg] |
|---|---:|---:|---:|---:|
| 상승 | 25,000 | 120,000 | 200 | 400,000 |
| 재진입 연소 | 25,000 | 0 | 200 | 14,800 |
| 공력 하강 | 25,000 | 0 | 200 | 8,800 |
| 최종 동력 하강 | 25,000 | 0 | 200 | 6,800 |
| 착륙 (`project2_landing`) | 25,600 | 0 | 200 | 3,700 |

각 질량은 독립된 예산 가정이며 인접 구간의 연료 소모로 연결한 값이 아니다. 구조 질량은 두
추진제 계통을 제외한다. 총질량, 양의 추진제, 탱크 수용량과 연소 예산을 확인한다.
CG/관성은 기존 dual-tank 형상과 질량 적분으로 계산한다.

[공통 회복 설정](../../configs/scenario/_project2_recovery.yaml)은 상승과 일관된 가정을 사용한다.
엔진당 진공 추력 914 kN, Isp 311 s, 배압 면적 0.68 m², booster nose 기준 탱크 바닥
`[-26,-40] m`, 길이 `[25,14] m`, 반경 `[1.8,1.8] m`, 밀도 `[1230,810] kg/m³`, O/F=2.56이다.
기체 길이는 42 m, 구조 CG는 −30 m, 구조 관성은 `[43000,3700000,3700000] kg·m²`다.
이 값들은 선언한 모델의 가정이며 CRS-11의 실측 인증값이 아니다.

일정 입력의 명목 유량과 정지 과도구간 여유로 연료 예산을 사전 검사한다. 이는 실제 구동기
유량 적분이나 후속 복귀에 충분한 예비량을 증명하지 않는다. 실제 소모량과 마지막 추력은 실행
기록으로 확인한다. RCS는 작동시키지 않으며 이 실험으로 RCS 자세 전환 성능을 검증하지 않는다.

### 3.1 착륙 시나리오의 조건

착륙은 [전용 시나리오](../../configs/scenario/project2_landing.yaml)에서
[기존 baseline](../../configs/baseline/pdg.yaml)의 초기조건·단일 엔진 기체·폐루프 유도·제어를
재사용하되 회전지구와 아래 추종 이득을 적용한다.
초기 위치 `[0,0,-300] m`, 속도 `[0,0,25] m/s`, roll/pitch/yaw=`[0,90,0]°`, 각속도 0,
throttle=0이며 목표는 원점에서 정지하는 것이다. 구조·탱크·엔진 파라미터는 앞의 회복 실험과
별도 설정이다. 실제 Falcon 9 착륙 자료를 복원한 초기조건은 아니다.

환경은 **구면 회전지구**, 기준 위도 28.6°(가정), 중심중력·코리올리·전체 원심 가속도,
`ussa_poly`, 무풍·무슬로시·무고장이다.
구동기는 **ideal**, 항법은 **truth 전달**, 유도는 LCvx, 제어는 PD이며 RCS 할당은 꺼져 있다.
plant/nav/control/guidance/log 주기는 각각 1/10/20/1000/100 ms다.
유도기의 일정 명목 중력에는 표면 원심 가속도를 포함한다. 횡방향 추종은 자세 응답을 거치므로
N/E/D 위치 이득 `[0.1,0.1,1] s⁻²`, 속도 이득 `[0.6,0.6,4] s⁻¹`로 분리하고 기존 수직 이득을
유지한다. 이는 비회전·완전 수직 사례에서 드러나지 않던 횡방향 진동을 억제하기 위한 설정이다.
LCvx 계획은 여전히 국소 일정 중력의 3-DOF 근사이며 실제 plant의 곡률·자전은 6-DOF로 전파한다.
회전지구에서 연착륙을 확인했지만 realistic 구동기·센서 항법의 착륙 검증을 주장하지 않는다.
기존 `+baseline=pdg`는 별도의 비회전 회귀 설정으로 유지한다.
접지 후 종료·등급 평가를 수행하며 착륙 다리의 충격·접촉 구조 동역학은 포함하지 않는다.

## 4. 물리·수치 검증 기준

다음 독립 적분·시간 간격·실제 추력 정지 기준은 앞의 네 지정 입력 실험에 적용한다.

- 초기 상태·입력 일정·주기가 설정과 일치하고 시간은 단조 증가해야 한다. 이벤트는 plant 격자에
  놓고, 입력이 끝나는 시각과 실행 종료 시각을 구분해 실제 정지 응답을 관찰한다.
- 질량 bookkeeping 오차는 0.001 kg 이하, quaternion norm 오차는 1e−12 이하로 확인한다.
  추진제 비음수·질량 증가 없음·비활성 엔진 힘 0·RCS 소모 0·마지막 실제 추력 0을 검사한다.
- 선언한 Newton–Euler 식·질량 적분·회전행렬·realistic 구동기·중립 핀을 사용한 별도 DOP853
  적분과 비교한다. 위치 0.01 m, 속도 0.001 m/s, 자세/각속도 1e−6 rad / rad/s, 질량
  0.001 kg 기준과, 기준 적분 tolerance를 줄였을 때 그 1/10 이하의 변화를 요구한다.
- 2/1/0.5 ms 실행의 공유 기록 시각에서 위치 0.1 m, 속도 0.01 m/s, 자세 1e−4 rad,
  질량 0.01 kg, 동압 10 Pa, 종료 시각 0.005 s 이내 차이를 확인한다.
- body 공력표의 받음각/Mach 범위 안인지 검사하며, 이 범위를 벗어나면 실험 합격으로 처리하지
  않는다. 수치 일치는 선언한 모델에 대한 구현 확인이며 실기체 공력계수 검증이 아니다.
- 리플레이의 기체 자세·CG·엔진 수/위치·실제 추력 방향·핀 노출·역탐색을 확인한다.

착륙은 별도의 **폐루프 연착륙 회귀 검사**다. 실제 접지가 최대 시간 전에 발생하고 기존 KPI에서
`landing_class=soft`, `success=true`여야 한다. 유한 상태, quaternion, 추진제 비음수,
질량 bookkeeping 및 공력표 범위도 확인한다. 접지 시 추력이 0일 필요는 없다.
네 구간용 지정 입력 DOP853·시간 간격 검사를 착륙에 적용했다고 표시하지 않는다.

최종 동력 하강의 초기 35초/33초 연소 실험은 하강을 멈추고 다시 상승해 받음각 범위를 벗어났다.
현재는 **20초 연소 + 2초 정지 응답 관찰**로 실험 범위를 정했다. 착륙 성공이나 CRS-11 오차를
맞추기 위한 변경이 아니며, 하강 중 감속과 정지 응답을 공력표 범위 안에서 확인하기 위한 선택이다.

상승은 높은 고도에서 층별 대기식을 연장한다. 대기·공력의 경험적 정확도는 별도로 남는
모델 한계다. 지구는 구면 중심중력 근사이며 타원체/J2·중력경사 토크·조석은 포함하지 않는다.
원·타원궤도와 무토크 회전의 [회전지구 검증](../verification/physics_implementation_report_en.md#11-spherical-rotating-earth-verification)을
수행했다. 바람, 초기 자세/각속도, 질량·추력·항력을 바꾸는 민감도 실험은 추가로 수행할 수 있다.

## 5. 실행과 결과 확인

일반 사용은 아래 실행 인자 중 하나를 선택하고, 같은 뷰어 명령으로 최근 결과를 연다.

| Phase | `scripts/run_single.py`에 붙일 인자 |
|---|---|
| 상승 | `scenario=project2_ascent` |
| 재진입 연소 | `scenario=project2_entry_burn` |
| 공력 하강 | `scenario=project2_aero_descent` |
| 최종 동력 하강 | `scenario=project2_powered_descent` |
| 착륙 | `scenario=project2_landing` |

```bash
.venv/bin/python scripts/run_single.py scenario=project2_landing
.venv/bin/python scripts/view_replay.py --system
```

아래는 결과 경로를 고정해 다섯 사례와 검증을 재현하는 명령이다.

```bash
.venv/bin/python scripts/run_single.py scenario=project2_ascent hydra.run.dir=results/project2_physics/ascent
.venv/bin/python scripts/run_single.py scenario=project2_entry_burn hydra.run.dir=results/project2_physics/entry_burn
.venv/bin/python scripts/run_single.py scenario=project2_aero_descent hydra.run.dir=results/project2_physics/aero_descent
.venv/bin/python scripts/run_single.py scenario=project2_powered_descent hydra.run.dir=results/project2_physics/powered_descent
.venv/bin/python scripts/run_single.py scenario=project2_landing hydra.run.dir=results/project2_physics/landing
.venv/bin/python scripts/check_project2_segments.py
.venv/bin/python scripts/check_rotating_earth.py
.venv/bin/python scripts/view_replay.py results/project2_physics/powered_descent
```

기본 출력은 궤적, 힘·모멘트, 추진제, 자세·각속도, 입력·이벤트와 리플레이다.
물리 검증 스크립트는 네 구간의 독립 방정식·시간 간격·공력표 적용 범위와 착륙 회귀를 확인하며
`results/verification/project2_segments/physics/`에 판정과 물리 그림을 저장한다.
그림·JSON·NPZ·실행 설정 복사본은 Git에서 제외하며 재현 명령으로 생성한다. 새 작업 환경에서는
명령을 실행한 뒤 이 문서의 생성 결과 링크를 열 수 있다. 명세·코드·수동 작성 보고서는 저장소에 남긴다.
[현재 물리 실험 결과](../verification/project2_segments/physics_results.md)를 참조한다.
`run_single.py`의 `validation.json=not_requested`는 별도의 정형 해석해 검사를 지정하지 않았다는
뜻이다. 네 지정 입력 구간의 독립 적분 판정과 착륙의 별도 접지·회귀 판정은 검사 스크립트의
각 `verification.json`에 있다. `--scenario landing`으로 착륙만 검사할 수 있다.

## 6. 선택 분석: CRS-11과의 관측 비교

CRS-11 자료는 실제 비행의 고도·속력·시간 규모를 참고하는 자료다. 웹캐스트 표시를 추출한 값으로,
센서·항법 추정·지상/방송 처리의 오차나 지연을 분리할 수 없다. 완전한 6-DOF 실측 truth가 아니다.
현재의 독립 실험과 실제 비행은 질량·입력·자세·환경도 다르므로 RMSE를 시뮬레이터 오차나
항법 오차로 단정하지 않는다. 현재 실험의 필수 합격 기준에 RMSE를 넣지 않는다.

네 지정 입력 구간은 기본 실행의 `scenario.reference.enabled=false`를 유지하며, 비교하려면
다음처럼 명시한다. 착륙에는 대응하는 공개 비행자료 비교를 배정하지 않는다.

```bash
.venv/bin/python scripts/run_single.py scenario=project2_entry_burn scenario.reference.enabled=true hydra.run.dir=results/project2_optional/entry_burn
.venv/bin/python scripts/check_project2_segments.py --compare-flight --output results/project2_optional/checks
```

초기값 선택 과정의 [관측·미분 분석](../../results/verification/project2_segments/reference_analysis.json)과
[분석 설정](../../configs/analysis/project2_segments.yaml)은 출처 기록이다. 명세에는 실험 시간과
선택 비교용 reference window를 별개로 보관한다. 예를 들어 현재 최종 하강은 22초만 실행하므로,
선택 비교를 해도 기준 비행의 착륙까지를 모두 모사했다고 해석하지 않는다.

기존 관측 처리 방법은 [레퍼런스 계약](crs11_flight_reference.md)에 기록했다. 고도 미분의
2/4/6초 민감도와 불일치 성분 거부, 시간 공백·고도 0 표시의 한계를 유지한다.
[이전 CRS-11 비교 결과](../../results/verification/project2_segments/runtime_results.md)는 초기 35초 최종
하강을 포함한 과거 실행 기록이며 현재 시나리오의 합격/불합격 기준으로 사용하지 않는다.

Project 2의 공개 비행 자료 비교는 이후 초기조건·입력·관측 기준의 대응 관계를 정리해 다룬다.
센서·항법 오차는 별도의 simulation truth와 합성 센서 측정으로 평가한다.
