# Image Match Node

Gazebo 전역 지도와 차량 주변 BEV 영상을 feature matching으로 매칭해
`map` 기준 차량 pose를 추정하는 ROS 2 패키지입니다.

세 가지 백엔드를 지원합니다.

| 백엔드 | 알고리즘 | 특성 |
| --- | --- | --- |
| **OpenCV** | SIFT / ORB / AKAZE + KNN + RANSAC | 가중치 없음, CPU 실시간 비교 |
| **EfficientLoFTR** | HuggingFace `zju-community/efficientloftr` | 경량, semi-dense |
| **MINIMA** | LoFTR / RoMa / SP+LightGlue (선택) | dense, 높은 매칭 수 |

---

## OpenCV

학습 가중치 없이 다음의 전통적 영상처리 파이프라인을 사용합니다.

1. 지도와 BEV에서 SIFT, ORB 또는 AKAZE 특징점과 descriptor 추출
2. 두 영상에 동일한 HSV/morphology lane-mask 전처리 적용
3. lane mask 주변의 grayscale 특징점과 descriptor를 별도로 추출
4. raw와 lane descriptor를 각각 KNN + Lowe ratio test로 매칭
5. 두 correspondence 집합을 합쳐 RANSAC homography 계산
6. (선택) Chamfer distance refinement로 homography 정밀화
7. BEV 차량 중심과 전방점을 지도에 투영해 pose 계산

lane-mask 전처리는
`perception/classic_road_mask_ros2/scripts/ros2_road_lane_mask_wrapper.py`와
같은 전통적 영상처리 방식입니다. 지도와 입력 BEV가 반드시 같은 HSV,
ROI, morphology 파라미터를 사용하도록 노드 내부에서 하나의 함수를
공유합니다. Binary mask 자체를 descriptor로 사용하지 않고, 확장된 lane
영역 안의 grayscale 특징점을 사용해 차선 경계의 명암 정보를 유지합니다.

지도 특징은 노드 시작 시 한 번만 계산합니다. 기본 SIFT 설정의 Gazebo
검증에서는 BEV 프레임당 약 340개 특징, 32개 전후의 ratio-test match,
약 0.5의 RANSAC inlier ratio, 약 0.13초의 처리 시간을 확인했습니다.

### 실행

```bash
cd /ros_ws
colcon build --packages-select image_match_node
source install/setup.bash

ros2 launch image_match_node opencv.launch.py
```

결과 확인:

```bash
ros2 topic echo /visual_localization/opencv/pose
ros2 topic echo /visual_localization/opencv/odometry
ros2 topic echo /visual_localization/opencv/match_count
ros2 topic echo /visual_localization/opencv/lane_match_count
ros2 topic echo /visual_localization/opencv/lane_inlier_count
ros2 topic echo /visual_localization/opencv/inlier_ratio
ros2 topic echo /visual_localization/opencv/chamfer_score
ros2 topic echo /visual_localization/opencv/chamfer_refined_score
rqt_image_view /visual_localization/opencv/debug_matches
```

디버그 영상에서 lane mask 영역은 노란색으로 옅게 표시됩니다. 일반
grayscale inlier match는 녹색, lane 기반 inlier match는 자홍색입니다.

### 알고리즘 변경

[config/opencv_gazebo.yaml](config/opencv_gazebo.yaml)의 `feature_type`을
변경합니다.

```yaml
feature_type: sift  # sift | orb | akaze
```

- `sift`: 기본값. scale과 회전에 강하고 현재 Gazebo texture에서 가장 안정적
- `orb`: 가장 빠르지만 반복 차선과 큰 scale 변화에 민감
- `akaze`: binary descriptor 기반의 중간 선택지

주요 튜닝 파라미터:

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `max_features` | `6000` | 지도와 BEV에서 유지할 최대 특징 수 |
| `ratio_test` | `0.72` | 낮을수록 더 엄격한 descriptor 매칭 |
| `use_clahe` | `true` | grayscale 명암 대비 보정 |
| `min_matches` | `12` | homography 계산 전 최소 match 수 |
| `min_inliers` | `6` | pose 발행에 필요한 RANSAC inlier 수 |
| `min_inlier_ratio` | `0.10` | pose 발행에 필요한 inlier 비율 |
| `ransac_reproj_threshold` | `5.0` | RANSAC reprojection 오차 허용값 |
| `enable_lane_features` | `true` | raw 특징과 lane 특징을 함께 사용 |
| `lane_ratio_test` | `0.75` | lane descriptor 전용 ratio test |
| `lane_hsv_lower` | `[0, 0, 165]` | lane paint HSV 하한 |
| `lane_hsv_upper` | `[179, 80, 255]` | lane paint HSV 상한 |
| `lane_min_area` | `20` | 제거할 작은 lane component 크기 |
| `lane_feature_dilation_ratio` | `0.02` | descriptor 영역 확장 비율 |
| `enable_chamfer_refinement` | `true` | RANSAC 후 Chamfer distance로 homography 정밀화 |
| `chamfer_max_pts` | `400` | 프레임당 EDT lookup에 사용할 lane 픽셀 수 (많을수록 정확, 느림) |
| `chamfer_score_gate` | `20.0` | 초기 Chamfer score(픽셀)가 이 값 초과 시 refinement 건너뜀 |

`match_count`는 raw와 lane match의 합이고, `lane_match_count`는 그중 lane
채널에서 추가된 match 수입니다. `lane_inlier_count`는 결합 RANSAC에서
실제로 채택된 lane match 수이므로 lane 채널의 기여도를 확인하는 데
사용합니다. lane 전처리만 끄고 기존 동작과 비교하려면 다음처럼
설정합니다.

```yaml
enable_lane_features: false
```

### Chamfer Distance Refinement

`enable_chamfer_refinement: true`를 설정하면 RANSAC homography 추정 후
lane mask 픽셀을 이용해 homography를 추가로 정밀화합니다.

**동작 원리**:

1. 초기화 시 맵 lane mask에 EDT(Euclidean Distance Transform)를 계산합니다.
   각 픽셀 값은 "가장 가까운 lane 픽셀까지의 거리(픽셀)"입니다.
2. 프레임마다 BEV lane 픽셀을 RANSAC homography로 맵 좌표계에 투영합니다.
3. 투영된 점들의 EDT 값 평균(= Chamfer score)이 낮을수록 정렬이 좋습니다.
4. (dx, dy, dθ) 3자유도 보정 행렬을 coarse-to-fine coordinate descent로
   최적화해 Chamfer score를 최소화합니다.
5. 개선이 있을 때만 보정 행렬을 적용합니다.

**Chamfer score 해석**:

| 토픽 | 의미 |
| --- | --- |
| `chamfer_score` | RANSAC H를 그대로 투영했을 때의 평균 EDT 값 (픽셀) |
| `chamfer_refined_score` | refinement 후 평균 EDT 값 (픽셀) |

score가 0에 가까울수록 lane이 맵 lane 위에 정확히 놓입니다. score > `chamfer_score_gate`이면 SIFT H가 너무 부정확한 것으로 판단해 refinement를 건너뜁니다.

refinement를 일시적으로 비활성화하고 싶을 때:

```yaml
enable_chamfer_refinement: false
```

OpenCV backend는 다른 backend와 동시에 비교할 수 있도록
`/visual_localization/opencv/*` topic을 사용하고 `publish_tf`는 기본적으로
비활성화합니다.

`/visual_localization/opencv/odometry`는 pose와 동일한 위치, 방향,
covariance를 `nav_msgs/Odometry`로 발행합니다. 영상 매칭만으로 속도를
직접 측정하지 않으므로 twist 값은 0이고, twist covariance는 미측정을
나타내도록 크게 설정합니다.

### 현재 한계

현재 트랙은 비슷한 차선과 도로 texture가 여러 구간에서 반복됩니다.
따라서 match 수와 inlier ratio가 높더라도 잘못된 전역 구간에 정합될 수
있습니다. 실제 Gazebo 검증에서도 국소 궤적은 연속적이었지만 `/odom_gt`와
다른 전역 위치에 수렴하는 경우가 확인됐습니다.

최종 localization에 사용하려면 다음 단계로 이전 pose 또는 초기 pose
주변의 지도 ROI만 검색하고, 시간 연속성과 차량 속도로 pose jump를
거부하는 처리가 필요합니다.

---

## MINIMA

MINIMA는 세 가지 매칭 방법을 지원합니다.

| method | 가중치 파일 | 특성 |
| --- | --- | --- |
| `loftr` (기본) | `MINIMA/weights/minima_loftr.ckpt` | dense transformer, 안정적 |
| `roma` | `MINIMA/weights/minima_roma.pth` | 최고 정확도, 느림 |
| `sp_lg` | `MINIMA/weights/minima_lightglue.pth` | sparse, 빠름 |

### 가중치 다운로드

```bash
cd /ros_ws/src/aac_2026/navigation/image_match_node/MINIMA/weights
bash download.sh
```

다운로드 완료 후 확인:

```bash
ls -lh /ros_ws/src/aac_2026/navigation/image_match_node/MINIMA/weights/
# minima_loftr.ckpt  (~200 MB)
# minima_roma.pth    (~400 MB)
# minima_lightglue.pth (~100 MB)
```

### Build & Run

```bash
# Build (컨테이너 안)
cd /ros_ws
colcon build --packages-select image_match_node
source install/setup.bash

# 실행 — loftr (기본)
ros2 launch image_match_node minima.launch.py

# 실행 - config의 method/ckpt_path 사용
ros2 launch image_match_node minima.launch.py \
  config:=/ros_ws/src/aac_2026/navigation/image_match_node/config/minima_gazebo.yaml

# 파라미터를 명령행에서 바꿀 때는 ros2 run 사용
ros2 run image_match_node minima_localization_node \
  --ros-args \
  -p method:=loftr \
  -p ckpt_path:=""
```

`ros2 launch`는 지정된 YAML을 읽습니다. YAML을 수정했다면 패키지를
재빌드하거나 `config:=/ros_ws/src/.../minima_gazebo.yaml`처럼 소스
파일의 절대경로를 전달합니다.

### 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `method` | `loftr` | `loftr` / `roma` / `sp_lg` |
| `ckpt_path` | `""` | 빈 문자열이면 method에 맞는 가중치를 자동 선택. 상대경로는 `MINIMA/weights` 기준 |
| `match_thr` | `0.2` | LoFTR coarse matching threshold (loftr만 적용) |
| `frame_stride` | `3` | N 프레임마다 1회 추론 |
| `min_matches` | `40` | 이 수 미만이면 pose 미발행 |
| `min_inliers` | `20` | RANSAC inlier 최소 수 |
| `min_inlier_ratio` | `0.2` | RANSAC inlier 비율 최소값 |

그 외 좌표계 파라미터(`map_x_min` 등)는 EfficientLoFTR와 동일합니다.
`config/minima_gazebo.yaml` 참고.

---

## EfficientLoFTR

현재 wrapper는 Hugging Face Transformers에 통합된
`zju-community/efficientloftr` 모델을 사용합니다. 모델 파일은 저장소에
포함하지 않고 처음 실행할 때 Hugging Face cache에 다운로드합니다.

처리 순서는 다음과 같습니다.

1. `simple_gazebo_track`의 `track.png`를 전역 지도 이미지로 로드
2. `/ipm_bev/fused`와 지도 사이의 EfficientLoFTR match 생성
3. RANSAC homography로 BEV 픽셀을 지도 픽셀로 변환
4. BEV 차량 중심과 전방점을 지도에 투영해 `x`, `y`, `yaw` 계산

## Dependencies

`numpy==1.26.4`, `transformers==4.57.6`, `safetensors>=0.4.3`은 루트
`requirements.txt`에 등록되어 있습니다. x86/Mac/Jetson Docker 이미지를
빌드하면 자동으로 설치되므로 별도 pip 설치가 필요 없습니다.

Hugging Face cache는 Dockerfile의 `HF_HOME` 설정에 따라
`/ros_ws/src/aac_2026/.hf_home`에 저장되며 node의 `cache_dir`은 그
아래 `hub` 디렉터리를 사용합니다. 이 경로는 `.gitignore`에 등록되어
있고 host workspace와 mount되므로, 컨테이너를 삭제하고 다시 생성해도
모델 파일이 유지됩니다.

## EfficientLoFTR Model Cache Setup

다른 PC에서 처음 작업할 때는 저장소를 clone한 뒤 Docker 컨테이너 안에서
모델 cache를 한 번 준비해야 합니다. 모델 cache는 Git에 올리지 않고 각
PC의 workspace에 보관합니다.

### 1. Host: Build and Start Docker

```bash
cd ~/ros_ws/src/aac_2026/docker
./build_docker.sh x86
./run_x86.sh
```

### 2. Container: Build the ROS Package

`prepare_efficientloftr_cache` 명령은 `image_match_node`를 빌드한 뒤
사용할 수 있습니다.

```bash
cd /ros_ws/src/aac_2026
./perception/setup_colcon_ignores.sh

cd /ros_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select image_match_node
source install/setup.bash
```

`ros2 run image_match_node prepare_efficientloftr_cache --help`가 출력되면
준비 명령이 정상적으로 설치된 것입니다.

### 3. Cache Directory

기본 cache 위치:

```text
/ros_ws/src/aac_2026/.hf_home
/ros_ws/src/aac_2026/.hf_home/hub
```

확인:

```bash
cd /ros_ws/src/aac_2026
echo "$HF_HOME"
mkdir -p .hf_home
```

`docker/run_x86.sh`, `docker/run_mac.sh`, `docker/run_jetson.sh`와
`docker/env.sh`가 기본적으로 `HF_HOME=/ros_ws/src/aac_2026/.hf_home`을
설정합니다. `.hf_home/`은 `.gitignore`와 `.dockerignore`에 들어 있어
커밋되거나 Docker build context에 포함되지 않습니다.

중요한 동작 방식:

- `.hf_home`은 host의 `~/ros_ws/src/aac_2026/.hf_home`에 실제로 저장됩니다.
- Docker에서는 같은 디렉터리가 `/ros_ws/src/aac_2026/.hf_home`으로 보입니다.
- `git clone` 또는 `git pull`은 모델 cache를 내려받지 않습니다.
- 컨테이너만 삭제하고 workspace를 유지하면 다운로드한 모델도 유지됩니다.
- workspace 자체를 삭제하면 cache도 함께 삭제되므로 다시 받거나 복사해야 합니다.

### 4. Download

온라인 상태에서 한 번 실행합니다.

```bash
cd /ros_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run image_match_node prepare_efficientloftr_cache
```

정상 출력 예:

```text
[efficientloftr-cache] HF_HOME=/ros_ws/src/aac_2026/.hf_home
[efficientloftr-cache] cache_dir=/ros_ws/src/aac_2026/.hf_home/hub
[efficientloftr-cache] model_id=zju-community/efficientloftr
[efficientloftr-cache] processor=EfficientLoFTRImageProcessor
[efficientloftr-cache] model=EfficientLoFTRForKeypointMatching
[efficientloftr-cache] ready
```

다운로드 후 용량은 대략 60-70 MB입니다.

```bash
du -sh /ros_ws/src/aac_2026/.hf_home
```

### 5. Offline Check

네트워크 없이 cache가 잘 준비됐는지 확인합니다.

```bash
ros2 run image_match_node prepare_efficientloftr_cache -- --local-files-only
```

이 명령이 성공하면 config에서 다음처럼 실행할 수 있습니다.

```yaml
cache_dir: /ros_ws/src/aac_2026/.hf_home/hub
local_files_only: true
```

런치에서 임시로 override할 수도 있습니다.

```bash
ros2 launch image_match_node efficientloftr.launch.py \
  config:=/ros_ws/src/aac_2026/navigation/image_match_node/config/efficientloftr_gazebo.yaml
```

### 6. Moving Cache From Another PC

네트워크가 느린 PC에서는 이미 받은 cache를 복사해도 됩니다.

```bash
# source PC
cd /ros_ws/src/aac_2026
tar czf /tmp/aac_hf_home_efficientloftr.tgz .hf_home

# target PC
cd /ros_ws/src/aac_2026
tar xzf /tmp/aac_hf_home_efficientloftr.tgz
ros2 run image_match_node prepare_efficientloftr_cache -- --local-files-only
```

압축 파일을 옮길 때는 `scp`, USB 저장장치 또는 팀 파일 서버를 사용할 수
있습니다. 서로 다른 사용자 계정으로 압축을 풀었다면 아래 권한 복구도
수행합니다.

```bash
sudo chown -R "$(id -u):$(id -g)" /ros_ws/src/aac_2026/.hf_home
```

### 7. Troubleshooting

`ModuleNotFoundError: No module named 'transformers'`:

Docker 이미지가 오래됐거나 requirements.txt 적용 전에 빌드된 경우입니다.
이미지를 다시 빌드하세요: `./build_docker.sh x86`

`Can't load image processor ... local_files_only`:

```bash
ros2 run image_match_node prepare_efficientloftr_cache
ros2 run image_match_node prepare_efficientloftr_cache -- --local-files-only
```

`Permission denied` under `.hf_home`:

```bash
sudo chown -R "$(id -u):$(id -g)" /ros_ws/src/aac_2026/.hf_home
```

`cv_bridge` 또는 ROS Python에서 NumPy ABI 문제가 보이면:

Docker 이미지를 다시 빌드하세요 (`numpy==1.26.4`가 requirements.txt에 고정되어 있습니다).

캐시가 실제로 Git에서 제외되는지 확인:

```bash
cd /ros_ws/src/aac_2026
git check-ignore -v .hf_home
git status --short
```

## Build

Docker 컨테이너 안에서:

```bash
cd /ros_ws/src/aac_2026
./perception/setup_colcon_ignores.sh

cd /ros_ws
colcon build --packages-select image_match_node
source install/setup.bash
```

현재 컨테이너의 Python 패키징 조합에서는 이 패키지에
`--symlink-install`을 사용하지 않습니다.

## Run

먼저 Gazebo와 `ipm_bev_node`를 실행합니다.

```bash
cd /ros_ws/src/aac_2026
./start.sh
```

다른 터미널에서:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros_ws/install/setup.bash
ros2 launch image_match_node efficientloftr.launch.py
```

첫 실행에서는 `zju-community/efficientloftr` 모델 약 64 MB를
다운로드하므로 네트워크 연결이 필요합니다. 다운로드 후에는 config의
`local_files_only: true`로 오프라인 실행할 수 있습니다.

## Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/visual_localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 지도 기준 차량 pose |
| `/visual_localization/debug_matches` | `sensor_msgs/Image` | 지도-BEV inlier match |
| `/visual_localization/homography` | `std_msgs/Float64MultiArray` | BEV에서 지도 이미지로의 3x3 homography |
| `/visual_localization/match_count` | `std_msgs/Int32` | confidence threshold 통과 match 수 |
| `/visual_localization/inlier_ratio` | `std_msgs/Float32` | RANSAC inlier 비율 |

확인:

```bash
ros2 topic echo /visual_localization/pose
ros2 topic hz /visual_localization/pose
rqt_image_view /visual_localization/debug_matches
```

## Coordinate Calibration

기본 지도 범위는 `simple_track.world`의 track box를 기준으로 합니다.

```yaml
map_x_min: -6.0
map_x_max: 6.0
map_y_min: -8.0
map_y_max: 8.0
map_swap_xy: true
map_flip_x: true
map_flip_y: false
```

`simple_track`의 Gazebo box top-face UV는 이미지 가로축이 world `+Y`,
이미지 위쪽이 world `+X`에 대응합니다. 따라서 위 swap/flip 조합을
사용합니다. 초기 검증은 차량 spawn 위치 `(0.0, -6.5)`와 `/odom_gt`를
기준으로 수행하는 것이 좋습니다.

`/odom_gt`는 Gazebo `dynamic_pose`에서 추출한 실제 월드 pose이며
`map` frame을 사용합니다. Ackermann 플러그인의 적분 wheel odometry는
`/odom`으로 분리되어 있으므로 visual localization 정확도 비교에는
`/odom_gt`를 사용합니다.

## Limitations

- 현재 구현은 전역 지도 전체와 BEV를 직접 비교하는 초기 버전입니다.
- 트랙의 반복 차선 패턴 때문에 잘못된 위치로 수렴할 수 있습니다.
- 실시간화를 위해서는 이전 pose 주변의 지도 ROI 검색과 temporal
  filtering을 추가하는 것이 좋습니다.
- homography 기반이므로 충분한 inlier가 없으면 pose를 발행하지 않습니다.
