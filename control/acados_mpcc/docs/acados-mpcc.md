# acados MPCC 실행과 검증

기존 ORCA MPCC의 모델·비용·제약을 acados로 이식했습니다. 실측 53.062m 트랙에서
SQP와 SQP_RTI 모두 한 바퀴를 완주했습니다. 기존 MATLAB baseline, 3D 지도,
트랙 입력은 보존되어 있습니다. 이 결과는 ORCA 1:43 모델의 Desktop simulation이며,
F1TENTH 모델·ROS2 제어·Jetson 실시간 성능을 검증한 결과는 아닙니다.

## 실행

workspace 루트에서 실행합니다. 이미 이 PC에 설치했고 Docker 이미지도 빌드했습니다.

```bash
# 오프라인 수렴 검증: SQP + exact Hessian
./scripts/acados_python.sh -m tools.acados_mpcc

# 주기당 한 번의 SQP: RTI + Gauss-Newton
./scripts/acados_python.sh -m tools.acados_mpcc --config config/acados_mpcc_rti.yaml

# 같은 workspace를 마운트하는 Docker 환경
docker compose run --rm -T acados python -m tools.acados_mpcc
docker compose run --rm -T acados python -m tools.acados_mpcc --config config/acados_mpcc_rti.yaml

# 설치 재현이 필요한 경우
./scripts/setup_acados.sh
docker compose build acados
```

매번 `results/acados_mpcc/<UTC>/`에 독립 결과를 생성합니다. 기본 입력은
`tracks/indoor_final/track.csv`입니다. `--track`, `--output`, `--steps`,
`--progress-speed`로 바꿀 수 있습니다. `--progress-speed`는 가상 진행 속도의 상한이며
실제 차량 속도를 고정하는 명령이 아닙니다. `--steps` 안에 한 바퀴를 완료하지 못하면
실패로 종료하므로, 짧은 실행을 성공한 lap으로 표시하지 않습니다.

정확한 설정은 [기본 설정](../config/acados_mpcc.yaml), [RTI 설정](../config/acados_mpcc_rti.yaml)에 있습니다.
두 설정 모두 N=40, 주기 20ms, 예측 구간 0.8s, 초기 vx=1m/s입니다.
실제 vx 상한 4m/s, 가상 진행 속도 상한 6m/s, ORCA 차량 폭 0.06m를 사용합니다.
RTI는 한 번의 SQP만 수행하므로 status=0을 전체 NLP 수렴으로 해석하지 않습니다.
실제 궤적·제약과 solver 잔차를 함께 확인해야 합니다.

## 설치 구성

- 공식 acados v0.6.0, commit `503364817c872d474ab5bed219c26760ac267769`.
- CasADi 3.7.2, Python 3.12, tera renderer 0.2.1.
- HPIPM/BLASFEO도 submodule commit 고정. [버전 기록](../config/acados_versions.json).
- 호스트: `vendor/acados`, `.venv-acados`. 전역 Python 패키지를 변경하지 않음.
- Docker: `/opt/acados`, `/opt/venv`. workspace는 `/ws`에 연결.
- Release shared-library build, GENERIC CPU kernels, OpenMP 비활성화.
- v0.6.0의 upstream Python package metadata가 아직 `0.5.1`을 표시하므로,
  acados 버전 확인에는 source commit을 사용합니다.

설치 스크립트는 공식 pendulum OCP의 C 코드 생성·컴파일·SQP/HPIPM 실행도 검사합니다.
구성은 [공식 설치 문서](https://docs.acados.org/installation/index.html)를 따릅니다.

## 무엇을 이식했는가

| 항목 | 구현 |
|---|---|
| 차량 상태 | `X,Y,yaw,vx,vy,yaw_rate,theta` |
| 물리 입력 | motor duty `D`, 조향각 `delta`, 가상 진행 속도 `vtheta` |
| 추가 상태 | 직전 물리 입력 3개 |
| 최적화 입력 | 정규화된 주기당 입력 변화량 `du` |
| 모델 | 기존 ORCA 질량·관성·축간 거리·Pacejka 타이어·구동 방정식 |
| 트랙 | 원본 XY 좌표와 길이를 보존한 주기 cubic spline |
| solver | acados 생성 C solver, PARTIAL_CONDENSING_HPIPM |
| plant | 같은 원본 ODE를 별도의 SciPy RK45로 고정 입력 적분 |

물리 입력은 `u_new = u_previous + [1,0.35,6] * du`입니다. `du`는 초당 변화율이 아닙니다.
예측 상태는 `z_next = [RK4(x,u_new,dt), u_new]`입니다.

비용은 contour/lag 오차, yaw rate, 직전 입력 크기, 입력 변화량, 진행 보상을 유지합니다.
terminal contour/yaw rate 비용만 10배이며 lag 비용은 그대로입니다. 선형 진행 보상
`-qV*vtheta`는 제곱 완성으로 NONLINEAR_LS에 넣었습니다. 해에 영향을 주지 않는 상수만
추가됩니다. acados의 기본 시간 간격 배율을 덮어써 원본과 같은 stage 비용의 합을 유지합니다.

트랙 제한은 매 제어 주기 초기 예측의 theta에서 얻은 좌우 strip이며,
ORCA half-width×1.5 = 0.045m씩 안쪽으로 축소합니다. 입력 변화량과 물리 state/input
bounds는 hard constraint입니다. soft boundary나 장애물 회피는 추가하지 않았습니다.

## MATLAB과 달라진 계산 방법

같은 물리적 목적함수를 옮겼지만, MATLAB과 수치 결과가 완전히 동일하다고 주장하지 않습니다.

1. MATLAB의 동역학 선형화 후 행렬 지수 이산화 → 비선형 RK4 예측.
2. 감쇠된 순차 QP 두 번 → 수렴형 SQP 또는 주기당 한 번의 RTI.
3. MATLAB spline → SciPy/CasADi의 동일한 주기 spline 표현.
4. MATLAB optimizer의 저속 전용 clamp를 이식하지 않고 원본 simulation ODE를 사용.
   이 모델의 정지점 미분은 특이하므로 현재 검증은 vx=1m/s에서 시작합니다.
5. MATLAB은 outer QP마다 strip을 갱신하지만, 이 구현은 제어 주기마다 갱신하고
   한 SQP solve 안에서는 고정합니다. 실제 plant 경로의 corridor 검사를 별도로 수행합니다.
6. 기본 SQP는 exact Hessian+PROJECT 정규화, RTI는 Gauss-Newton을 사용합니다.

따라서 기존 `quadprog` 실패와 이번 성공만으로 solver 자체의 우열이나 원래 실패 원인을
확정하지 않습니다. 이식 과정에서 warm-start 끝점의 위치를 실제 동역학으로 전파하도록
수정했고, 내부 QP 허용오차를 NLP보다 엄격히 설정했습니다. Gauss-Newton SQP는
15m 부근에서 stationarity 수렴이 느려 반복 한도에 도달했으며 그 실패 기록도 남아 있습니다.

## 검증 결과

호스트 CPU: AMD Ryzen 7 9700X. 아래 시간은 단일 실측 실행의 관측치입니다.

| 설정 | 실제 투영 진행량 | lap 시간 | solver 평균 / p95 / 최대 |
|---|---:|---:|---:|
| SQP, exact Hessian | 53.073m | 13.82s | 17.64 / 112.48 / 275.84ms |
| SQP_RTI, Gauss-Newton | 53.075m | 13.88s | 0.91 / 1.08 / 2.02ms |

RTI의 Python 입력 갱신·진단 수집까지 포함한 controller cycle은 평균 2.37ms,
p95 2.52ms, 최대 19.44ms였습니다. plant simulation까지 포함한 시간과 구분합니다.
SQP의 일부 주기는 20ms를 초과합니다. RTI 결과도 Jetson이나 OS scheduling을 포함한
실시간 보장은 아니며, 해당 하드웨어에서 별도 측정해야 합니다.

- 두 실행 모두 solver status 비정상 0회, 검사된 corridor 이탈 0.
- SQP: 691개 제어 주기, plant 3,455개 검사점. RTI: 694개 주기, 3,470개 검사점.
- SQP의 중간 plant 샘플에서 vx 상한 초과 최대 0.000312m/s.
  설정된 수치 허용오차 0.002 이내이며, 모든 제약이 수학적으로 정확히 만족됐다고 표현하지 않습니다.
- 모델 ODE는 원본 MATLAB 실행 fixture와 최대 약 1.4e-14 차이.
  RK4 20ms 예측과 원본 ode45 결과는 fixture에서 최대 약 2.7e-6 차이.
- 트랙 seam, 실제 위치 투영, 입력 변화 단위, stage/terminal 비용,
  실패 시 NaN 진단 보존을 포함해 자동 테스트 17개 통과.
- 원본 트랙 추출 테스트 5개도 유지.
- Docker에서도 SQP/RTI 한 바퀴와 입력/모델 테스트를 통과했습니다.
  주기별 pose/속도/입력 값은 해당 호스트 실행과 일치했습니다.

실제 위치 투영의 누적 진행량으로 lap을 판정합니다. 가상 theta만 증가시켜서는 성공할 수 없습니다.
각 20ms plant 구간에서 5개 위치를 검사합니다. 별도의 [지도/경계 검토](acados-geometry-audit.json)는
0.045m 여유를 둔 트랙 내부 1,071,913개 점이 원본 경계 polygon과 추론 usable mask 안에 있음을 확인했습니다.
이들은 유한 표본 검사이며 연속 궤적의 해석적 증명은 아닙니다.

실제 주행 경로도 원본 CSV polygon과 지도에 대해 독립 검토했습니다.
1mm 간격으로 보간한 SQP 53,559개, RTI 53,733개 경로점의 영역 밖 관측은 0이었고,
둘 다 실제 위치의 순방향 seam 통과 1회, 역방향 통과 0회였습니다.
[SQP 독립 검증](../../../../../results/acados_mpcc/20260910T131637954247Z/independent_validation.json),
[RTI 독립 검증](../../../../../results/acados_mpcc/20260910T131819003354Z/independent_validation.json)에
검사 방법·입력 hash·제약별 오차를 남겼습니다.

## 결과 파일

- `trajectory.csv`: 시간, 실제 pose/속도, 실제 투영 진행량, 입력, 경계 오차, 계산 시간.
- `simulation.npz`: 주기별 궤적과 주기 사이 plant 샘플, 마지막 상태·입력.
- `solver_statistics.json`, `solver_diagnostics.npz`: solver 상태·반복·잔차·계산 시간.
- `summary.json`: lap 성공 여부, 진행 거리, constraint violation, timing, 실패 이유.
- `simulation.png`: 트랙 위 주행 궤적 및 실제/가상 속도 비교.
- `config.yaml`, `provenance.json`: 모델·설정·버전·입력/코드 hash.
- `generated/`: 생성 C 코드, solver library, 정확한 acados OCP 설정.
- 실패 시 `failed_prediction.npz`: 실패한 예측, 입력 변화량, 실제 상태.

실행 결과 예시:
[SQP 그림](../../../../../results/acados_mpcc/20260910T131637954247Z/simulation.png),
[SQP 결과](../../../../../results/acados_mpcc/20260910T131637954247Z/summary.json),
[RTI 그림](../../../../../results/acados_mpcc/20260910T131819003354Z/simulation.png),
[RTI 결과](../../../../../results/acados_mpcc/20260910T131819003354Z/summary.json).

```bash
./scripts/acados_python.sh -m unittest discover -s tests/acados_mpcc -v
docker compose run --rm -T acados python -m unittest discover -s tests/acados_mpcc -v
python3 -m unittest discover -s tests/track_pipeline -v
```

다음 단계는 F1TENTH 모델 파라미터를 확인해 교체하고, 상태 추정·ROS2 입출력을 연결하는 것입니다.
현재 `map.pcd` 기반 실시간 localization, Jetson 배포, adaptive MPCC는 이 구현의 범위 밖입니다.
