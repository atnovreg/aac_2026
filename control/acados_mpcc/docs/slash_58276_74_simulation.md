# Slash VXL 2WD 58276-74 + VESC: 초기 시뮬레이션 모델

이 모델은 공개 제원을 적용한 초기 모델입니다. 실제 차량에서 식별한 모델이나 VESC 펌웨어 시뮬레이터가 아닙니다. VESC/배터리/장비 실측값을 받으면 src/aac_2026/control/acados_mpcc/config/vehicles/slash_58276_74.yaml의 가정을 교체해야 합니다.

## 자료와 파라미터
- 제조사 58276-74 매뉴얼 사본: https://www.astramodel.cz/manualy/5/58276-74-OM-EN-R00.pdf
  - Velineon 3500 rpm/V, 기본 피니언23T/스퍼86T, 내부 감속비2.72. 총 감속비=86/23×2.72=10.1704.
- 모델 번호가 일치하는 제조사 제원표 재게시: https://hobbyall.co.kr/product/detail.html?cate_no=98&display_group=1&product_no=45651
  - 휠베이스335mm, 전후 트랙 표기296mm, 기본 중량2.56kg, 길이568mm.
  - 폭은 바퀴 중심 간격이라고 단정하지 않고 전체 외곽296mm의 임시값으로 사용합니다. 일부 버전/판매처 길이578mm 표기를 고려해 외곽은 보수적으로578mm를 사용합니다. 실제 실차 외곽을 측정하면 대체합니다.
- VESC 공식 소스의 전류/브레이크/RPM 명령은 서로 구분됩니다: https://github.com/vedderb/bldc/blob/master/comm/commands.c
  - 이 작업은 통신 패킷을 보내거나 VESC를 설정하지 않습니다. 현재는 정규화한 가상 전류 명령과 조향각만 저장합니다.

미확인 가정: 기본2.56kg+배터리/장비/ESC 변경분1kg=총3.56kg, CG축간 중앙, 직사각형 근사 yaw관성0.12513kg m², 타이어반지름54.5mm, 마찰계수0.9, 타이어 tanh 강성6/rad, 구동효율80%, 등가 모터전류±30A, 조향각±0.45rad, 조향 명령변화율2.5rad/s. 어느 값도 실제 VESC의 허용 전류를 의미하지 않습니다.

## 2026-09-15 사용자 확인 장비 구성

구매 목록과 사용자 설명으로 확인한 예정 구성입니다. 조립·설정·실측 완료를 뜻하지 않습니다.

| 부품 | 구성 |
| --- | --- |
| 배터리 | Zeee 11.1V 5200mAh 50C 3S LiPo, XT60 |
| VESC | FLIPSKY Dual FSESC6.7 Plus with Aluminum Case, VC-67D-S-100 |
| 전원 분배 보드 | RoboRacer Power Board v2024.1, AE170_PDU_02.04 |
| 컴퓨터 | NVIDIA Jetson Nano (보드·방열 구성 미확인) |
| 카메라 | Intel RealSense D435i |
| LiDAR | Livox MID-360 |

기존 계산의 3S/11.1V는 이번에 확인된 구성과 일치합니다. 이번 업데이트는 장비 정보를 기록한 것으로 동역학 수치는 변경하지 않았고, 동일 조건 시뮬레이션을 재실행하지 않았습니다. 이전 실행의 config.yaml은 실행 당시 가정을 그대로 보존합니다.

전체 장착 무게는 아직 모릅니다. 배터리·보드·컴퓨터·센서·브래킷·배선과 기존 ESC 제거분까지 반영해야 하므로, 부품 이름만으로 기존 임시 총무게 3.56kg를 확정하지 않습니다. 실제 VESC 모터/배터리 전류 제한과 모터 감지 결과도 미확인입니다. 제조사 전류 정격을 시뮬레이션의 ±30A 대신 자동 적용하지 않습니다. 전원 보드의 출력 레일 및 배선 연결도 아직 검증하지 않았습니다.

## 수학 모델
평면 동적 자전거 모델: [x,y,yaw,vx,vy,yaw_rate,progress]. 입력: [등가 전류비율, 앞바퀴 조향각, 가상 진행속도].
정적 전후 축하중은 CG거리에서 계산합니다. 앞바퀴 횡력은 mu*Fzf*tanh(Bf*alpha_f). 후륜 구동·제동력과 횡력은 동일 마찰 예산을 공유합니다. 후륜 종력은 전류→이상 토크 상수60/(2*pi*Kv)→기어/바퀴 환산 요구력을 tanh로 포화하고, 남은 마찰 예산 sqrt((mu*Fzr)^2-Fxr^2)으로 횡력을 제한합니다. 전진 구동에는 단순 속도/전압 한계를 적용합니다. 이 등가 전류 정의는 실제 VESC FOC 전류 정의와 실측으로 맞춰야 합니다.

하중 이동, 서스펜션, 공중 운동, 실제 조향 액추에이터 지연, 배터리 전압강하, VESC 전기/열 과도응답, 타이어 종슬립 동역학은 포함하지 않습니다. 조향은 명령 변화율만 제한한 이상 액추에이터입니다. 저속 특이점은 sqrt(vx²+0.5²)로 정규화하며 수치 plant와 CasADi predictor에 동일하게 적용합니다.

## 지도와 제어 설정
v4 가정 바닥 및 정적 장애물은 변경하지 않고 v5_slash_footprint에578×296mm 외접원+5cm(반지름0.37469m)를 적용했습니다. 이 데이터의20cm 격자에서는 결과 중심 마스크가 이전 마스크와 같아 경로 좌표도 동일합니다. 추가로 모든 적분 위치에서 금지 셀 사각형까지 실제 XY거리를 구해 외접원 여유를 검사했습니다.

RTI/Gauss-Newton, 제어주기20ms, 예측60단계(1.2초), 전진 속도 상한4m/s, 기존 비용 가중치. ORCA 실행(40단계)과 예측 구간이 달라졌으므로 성능 차이를 차량 모델 하나의 효과로 해석하면 안 됩니다. 속도 상한은 비교용14.4km/h이며 자동차의 최고속도 제원이 아닙니다.

## 결과와 재현
결과: results/건지광장/slash_58276_74/{01_outer,02_left_inner,03_both_inner}/각 실행 폴더.
비교: comparison.json/csv, three_routes.png, passage_comparison.png, speed_timing.png.
기계 판독 제원/가정은 각 config.yaml에 통째로 보존합니다. 원래 ORCA 코드는 기본 선택으로 유지하며 기존 결과는 보존했습니다.

```bash
./scripts/acados_python.sh -m control.acados_mpcc --config src/aac_2026/control/acados_mpcc/config/acados_slash_outer_rti.yaml
./scripts/acados_python.sh -m control.acados_mpcc --config src/aac_2026/control/acados_mpcc/config/acados_slash_left_inner_rti.yaml
./scripts/acados_python.sh -m control.acados_mpcc --config src/aac_2026/control/acados_mpcc/config/acados_slash_both_inner_rti.yaml
```

다음 실측: 장비 포함 무게와 축별 무게, 실제 차폭/차장, 조향각 보정 및 응답지연, VESC 전류·속도 제한 및 모터 감지 설정, 현재 기어, 일정 전류에서 가감속 응답, 타이어 마찰/슬립. 기존 값은 실측으로 확인된 항목부터 교체합니다.
