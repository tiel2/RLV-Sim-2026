# CRS-11 공개 데이터의 취득 근거

조사일: 2026-11-05. 대상은 `Telemetry-Data`의 고정 버전 `b245d3b81aa36b7941ec10f3f4b508999d106a6d`다.

**고도·속력은 방송 화면에 표시된 숫자를 추출한 자료다.** 제공자의 설명과 당시 숫자 인식 코드를 확인했다. 영상 속 로켓의 크기·움직임으로 궤적을 역산한 자료가 아니다. 다만 현재 파일 전체를 원본 영상과 대조하지 않았고, 정확한 생성 코드 버전·옵션·수동 수정 이력은 확보하지 못했다.

## 직접 확인한 근거

| 자료 | 확인한 내용 |
|---|---|
| [고정 버전 README](https://github.com/shahar603/Telemetry-Data/blob/b245d3b81aa36b7941ec10f3f4b508999d106a6d/README.md) | raw는 약 30 fps 웹캐스트 추출, analysed는 raw에서 계산한 채널이라고 설명 |
| [2017-06-04 extract.py](https://github.com/shahar603/SpaceXtract/blob/c5e01a0c4c5b0c3fa061ad28dcdf0bbd52567173/extract.py) | 숫자 영역 자르기, 흑백·임계값 처리, 0–9 이미지 대조, 숫자·소수점·단위 해석 |
| [당시 get_telemetry.py](https://github.com/shahar603/SpaceXtract/blob/c5e01a0c4c5b0c3fa061ad28dcdf0bbd52567173/get_telemetry.py) | 프레임 반복, FPS 기반 시간 증가, 인식 실패·급격한 속력 변화 거부, JSON 저장 |
| [당시 CRS-11 파일](https://github.com/shahar603/SpaceXtract/blob/c5e01a0c4c5b0c3fa061ad28dcdf0bbd52567173/CRS-11/CRS-11%20raw.json) | 13,088행, 마지막 T+455.004 s. 현재 파일과 달라 이 코드로 현재 파일을 그대로 재생성한다고 주장할 수 없음 |
| [2018-03-11 stage-1 파일](https://github.com/shahar603/Telemetry-Data/blob/9019f96ceead37e56db52de4c88ca4221282b80d/SpaceX%20CRS-11/JSON/stage1%20raw.json) | 로컬의 13,489행·T+464.630 s 파일과 바이트 단위로 일치 |

[공식 웹캐스트](https://www.youtube.com/watch?v=JuZBOUMsYws)와 [착륙 항공 영상](https://www.youtube.com/watch?v=GrP3jHuLQ9o)은 원출처 확인용이다. 조사 당시 웹캐스트를 가져오지 못해 프레임별 검증은 수행하지 않았다. [NASA 임무 설명](https://www.nasa.gov/wp-content/uploads/2018/07/spacex_crs-11_mission_overview.pdf)과 [실제 발사 기록](https://www.nasa.gov/blogs/spacestation/2017/06/03/dragon-launches-to-station-cygnus-departs-sunday/)은 임무 식별 근거이며 사전 계획 시간을 실측 시각으로 사용하지 않았다.

## 숫자가 파일이 되는 과정

1. Streamlink/OpenCV로 영상을 읽고 시간·속력·고도·단위가 표시되는 영역을 자른다.
2. 흑백·임계값 처리 후 `cv2.matchTemplate`으로 숫자를 인식하고 위치 순서로 조합한다. 소수점과 단위도 해석한다.
3. km/h 표시는 3.6으로 나누며, 당시 코드의 `rtnd()`는 반올림 대신 소수 자릿수를 절삭한다.
4. 발사 기준 시각과 FPS로 프레임 시간을 구성한다. 모든 소수점 시각을 화면에서 독립적으로 읽은 것은 아니며 센서 timestamp도 아니다.
5. 인식 실패나 비정상적인 속력 변화를 거부하고 `{time, velocity, altitude}`를 저장한다. 당시 코드의 속력 변화율 기준은 55 m/s²였지만 현재 파일에 같은 옵션이 적용됐는지는 미확인이다.

예를 들어 로컬 표본 `{time: 372.004, velocity: 1260.277, altitude: 47.5}`의 속력은 `4537 / 3.6`을 소수 셋째 자리로 절삭한 값과 일치한다. 이는 변환 방식의 수치적 일치이며, 대응 영상의 4537 표시를 직접 확인했다거나 정확도가 0.001 m/s라는 뜻은 아니다.

## 해석의 한계

영상 해상도·압축·숫자 전환·인식·시간 구성에서 오차가 생길 수 있다. 약 0.033초 간격은 추출 빈도이며 독립 계측 주기가 아니다. SpaceX 내부 센서→항법 추정→방송 표시의 처리와 지연도 미확인이다. 실제 비행에서 유래했더라도 외란·슬로싱·센서 잡음의 크기나 보존 정도를 이 자료만으로 분리할 수 없다.

`analysed.json`의 수직·수평 속도, 가속도, 경로각, downrange, 동압은 계산 채널이다. 실제 GPS·자세·압력 센서 기록으로 쓰지 않는다. 원본 영상 해시·행별 프레임 대응표·전체 수정 로그가 없어 모든 표시값의 정확도를 인증할 수는 없다. 비교에서는 [관측 처리 기준](crs11_flight_reference.md)에 따라 raw 고도·총속력만 사용한다.

## 조사한 보조 자료

다음은 같은 고정 버전에서 조사한 stage-1 자료다. 이번 비교에는 채택하지 않았으며, 사용하려면 별도 원본 보존과 품질 검사가 필요하다.

| 자료 | 표본 수 / T+ 범위 [s] / 최대 간격 [s] | 판단 |
|---|---|---|
| [CRS-12](https://github.com/shahar603/Telemetry-Data/tree/b245d3b81aa36b7941ec10f3f4b508999d106a6d/SpaceX%20CRS-12/JSON) | 13,684 / 0–480.079 / 1.469 | 두 번째 비행 후보. 첫 회복 고도 0 표시에서도 속력은 32.222 m/s |
| [NROL-76](https://github.com/shahar603/Telemetry-Data/tree/b245d3b81aa36b7941ec10f3f4b508999d106a6d/NROL-76/JSON) | 16,371 / 0–557.823 / 1.001 | 독립 시간 매핑 필요. 첫 회복 고도 0 표시에서도 속력은 90 m/s |
| [CRS-13](https://github.com/shahar603/Telemetry-Data/tree/b245d3b81aa36b7941ec10f3f4b508999d106a6d/SpaceX%20CRS-13/JSON) | 912 / 422.755–453.753 / 0.267 | 말기 구간만 있어 다섯 구간 공통 기준으로 부적합 |

조사한 raw 파일의 SHA-256:

```text
CRS-12  9b0b7c588c0d81753cb3d5ea139b57af03869bfbb6caeb55f0806beeddac1616
NROL-76 b5316dba05fd735de71f90baacf8489c553ef3638008a97d7853e8e71da40647
CRS-13  674a4639f671a60de6bb2b75e7e6b19554b8953c206da160dec39f6fecf630db
```

[Launch Dashboard API 문서](https://github.com/shahar603/Launch-Dashboard-API)는 후속 탐색 경로로만 확인했고 서비스 응답은 사용하지 않았다. NASA의 [Morpheus/ALHAT 시험](https://ntrs.nasa.gov/citations/20170009170)과 [NDL 시험](https://ntrs.nasa.gov/citations/20160011576)은 착륙 계측 사례지만, 확인한 배포 페이지에서 원시 시계열을 찾지 못했고 다른 기체이므로 Falcon 9의 직접 비교 기준으로 채택하지 않았다.
