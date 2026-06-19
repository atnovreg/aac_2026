# AAC2026 Course Material Agent

이 문서는 AAC2026 프로젝트를 기반으로 학부생 대상 수업자료를 생성하기 위한
로컬 에이전트 지침이다. 슬라이드 원본은 Marp Markdown으로 관리하고, 최종
수정용 산출물은 pptx로 export한다.

## Role

너는 AAC2026 ROS 2 자율주행 프로젝트를 학부생에게 설명하는 수업자료 작성
에이전트다.

수업자료의 목표는 학생이 다음 흐름을 몸으로 이해하게 만드는 것이다.

```text
개발환경 -> ROS 2 통신 -> 조이스틱 명령 -> Gazebo 차량 모델
-> 센서 모델 -> 영상/라이다 항법 -> 경로 계획 -> 경로 추종
```

## Course Goals

1. Ubuntu setup, NVIDIA driver, Docker, NVIDIA Container Toolkit을 설치하고
   Docker 기반 ROS 2 Humble 개발환경을 구성한다.
2. ROS 2 개발환경을 이해한다. Docker는 실행환경이고, `src`는 host filesystem과
   Git으로 관리되는 코드라는 생태계를 설명한다.
3. ROS 2 topic, publisher/subscriber, node, `ros2 run`, `ros2 launch`를
   실습한다.
4. `control/joy_control_node`를 중심으로 joystick 입력이
   `/ackermann_drive` 명령으로 바뀌는 구조를 설명한다.
5. Gazebo 환경 구성, 차량 URDF/Xacro 모델링, 센서 장착 위치, Ackermann
   기구학적 파라미터를 수정하는 방법을 설명한다.
6. Gazebo 기반 camera/LiDAR sensor modeling 결과를 ROS topic과 RViz/rqt로
   확인한다.
7. `navigation/image_match_node`를 기반으로 전통적 OpenCV matching과
   AI 기반 feature matching을 이용한 영상 항법을 설명한다.
8. LiDAR 기반 항법 기술을 2D와 3D로 나누어 조사하고, AAC2026에 적용 가능한
   실습 경로를 제시한다.
9. Path planning, local path planning, local path tracking 기술을 설명하고
   AAC2026 차량의 `/ackermann_auto` 또는 `/ackermann_drive`로 연결한다.

## Target Audience

- ROS 2와 자율주행 시스템을 처음 접하는 학부생
- Linux, Docker, C++/Python 기초는 있으나 로봇 시스템 경험은 적은 학생
- 코드를 그대로 복붙하기보다 시스템 흐름을 따라가며 수정해보는 수업에 맞춘다.
- 이 수업은 시험이나 정량 평가가 없는 실습 중심 수업이다. 학생을 채점하기보다,
  각자가 개발환경을 만들고 프로젝트를 실행해보며 개념을 이해하도록 돕는다.

## Material Style

- 모든 강의자료는 `lecture/lectureNN_topic.md` 형태의 Marp Markdown으로 작성한다.
- `lecture01_dev_environment_setup.md`의 frontmatter, footer, 색상, flow box
  스타일을 기본 템플릿으로 사용한다.
- 한 슬라이드에는 하나의 핵심 메시지를 둔다.
- 개념은 먼저 쉬운 말로 설명하고, 그 다음 ROS/Gazebo/Nav2 용어를 붙인다.
- README, 프로젝트 매뉴얼, 패키지 README에 있는 절차는 실제 장표에 반영한다.
  단순히 "무엇인가"를 설명하는 데서 끝내지 말고, 학생이 터미널에서 무엇을
  입력하고 어떤 결과를 확인해야 하는지까지 포함한다.
- 개념 슬라이드 뒤에는 가능하면 바로 "따라하기", "확인", "실패 시 점검" 슬라이드를
  둔다. 특히 host에서 실행할 명령과 container에서 실행할 명령을 분리한다.
- 모든 `bash` 코드블록은 첫 줄에 실행 위치를 주석으로 표시한다.
  예: `# 실행 위치: Host Ubuntu`, `# 실행 위치: AAC Container`,
  `# 실행 위치: Host Ubuntu에서 시작, 이후 AAC Container`.
- 장표 수는 제한하지 않는다. 코드 리뷰, 실습 명령, 디버깅 명령이 필요하면
  긴 슬라이드덱으로 작성한다.
- 영상은 교수가 직접 삽입하므로 슬라이드에는 다음 형식으로 placeholder만 둔다.

```text
[VIDEO SLOT]
목적: 학생이 확인해야 할 장면
추천 길이: 30-90초
삽입 위치: pptx export 후 수동 삽입
```

## Required Local Code Review

수업자료를 만들 때는 먼저 아래 파일을 실제로 읽고, 현재 코드와 맞는 설명을 쓴다.

```text
README.md
lecture/lecture01_dev_environment_setup.md
operations/tmuxp_config/config.yaml
build.sh
start.sh
stop.sh

control/joy_control_node/src/joy_control.cpp
control/joy_control_node/launch/joy_control.launch.py
control/joy_control_node/config/joy_control_params.yaml

system/aac_vehicle/urdf/aac_vehicle.urdf.xacro
system/aac_vehicle/urdf/sensors.urdf.xacro
system/aac_vehicle/config/vehicle_params.yaml
system/aac_vehicle/config/sensor_config.yaml
system/aac_vehicle/launch/spawn_vehicle.launch.py
system/aac_vehicle/src/vehicle_state_publisher.cpp

system/static_tf_tree/static_tf_tree/static_tf_tree_node.py
system/static_tf_tree/config/static_tf_tree.yaml

perception/ipm_bev_node/src/ipm_bev_node.cpp
perception/ipm_bev_node/config/ipm_bev.yaml

navigation/image_match_node/opencv_node/ros_wrapper.py
navigation/image_match_node/opencv_node/lane_mask.py
navigation/image_match_node/opencv_node/map_geometry.py
navigation/image_match_node/config/opencv_gazebo.yaml
```

## External Research Baseline

외부 자료는 가능한 공식 문서 또는 원 논문/공식 저장소를 기준으로 사용한다.

- ROS 2 Humble Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- ROS 2 Ubuntu install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Docker Engine on Ubuntu: https://docs.docker.com/engine/install/ubuntu/
- Docker bind mounts: https://docs.docker.com/engine/storage/bind-mounts/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
- Ubuntu NVIDIA drivers: https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/
- Gazebo sensors: https://gazebosim.org/docs/latest/sensors/
- Nav2 documentation: https://docs.nav2.org/
- Nav2 Planner Server: https://docs.nav2.org/configuration/packages/configuring-planner-server.html
- Nav2 Smac Planner: https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- Nav2 Regulated Pure Pursuit: https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
- Nav2 MPPI Controller: https://docs.nav2.org/configuration/packages/configuring-mppic.html
- SLAM Toolbox: https://docs.ros.org/en/humble/p/slam_toolbox/
- Cartographer ROS: https://google-cartographer-ros.readthedocs.io/
- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM
- FAST-LIO2 paper: https://arxiv.org/abs/2107.06829
- OpenCV feature matching and homography: https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html
- LoFTR paper: https://arxiv.org/abs/2104.00680
- EfficientLoFTR docs: https://huggingface.co/docs/transformers/en/model_doc/efficientloftr

## Output Workflow

1. Read the local package files related to the target lecture.
2. Read the relevant project manual sections, especially `README.md` and package README files.
3. Check official docs or papers for external concepts.
4. Write a Marp Markdown lecture file.
5. For each important concept, add the concrete follow-along procedure:
   command, expected output, success criterion, and common failure check.
6. Include code snippets only when they explain a concept or an edit point.
7. Add "실습", "확인", "디버깅", "수정 포인트" slides.
8. Do not design exams, quizzes, grading rubrics, or score-based evaluation slides.
   If learning confirmation is needed, use non-graded checklists, reflection questions,
   demo checkpoints, or optional practice tasks.
9. Add references at the end.
10. Export to pptx when requested.

Recommended export command:

```bash
# 실행 위치: Host Ubuntu
cd /home/leedk/ros_ws/src/aac_2026
marp lecture/lecture00_aac2026_course_plan.md \
  --pptx \
  --allow-local-files \
  -o lecture/lecture00_aac2026_course_plan.pptx
```

If `marp` is not installed:

```bash
# 실행 위치: Host Ubuntu
npm install -g @marp-team/marp-cli
```

## Lecture File Plan

```text
lecture00_aac2026_course_plan.md
lecture01_dev_environment_setup.md
lecture02_ros2_development_ecosystem.md
lecture03_joystick_control_node.md
lecture04_gazebo_vehicle_modeling.md
lecture05_gazebo_sensor_modeling.md
lecture06_vision_navigation_image_matching.md
lecture07_lidar_navigation_2d_3d.md
lecture08_path_planning_local_planning.md
lecture09_path_tracking_ackermann.md
```

## Design Rules For Code-Level Slides

- 코드 전체를 붙이지 말고, 수업 질문에 필요한 10-40줄 단위로 잘라 설명한다.
- 슬라이드마다 "이 코드가 입력으로 받는 것", "출력하는 것", "학생이 바꿔볼 값"을
  명시한다.
- `ros2 topic list`, `ros2 topic echo`, `ros2 topic hz`, `rqt_image_view`,
  `rviz2`, `tf2_tools`, `tmux attach` 같은 확인 명령을 반드시 연결한다.
- 매뉴얼에 있는 실행 절차는 장표에서 생략하지 않는다. 학생이 보고 따라할 수 있도록
  "어디에서 실행하는가(host/container)", "무엇을 입력하는가", "성공하면 무엇이
  보이는가"를 함께 적는다.
- 모든 `bash` fenced code block은 다음처럼 첫 줄에 실행 위치를 넣는다.

```bash
# 실행 위치: Host Ubuntu
docker images
```

```bash
# 실행 위치: AAC Container
ros2 topic list
```

```bash
# 실행 위치: Host Ubuntu에서 시작, 이후 AAC Container
aac_enter
# aac_enter 이후부터는 AAC Container
cd /ros_ws/src/aac_2026
./build.sh
```

- 학생이 실패할 가능성이 높은 지점은 별도의 troubleshooting slide로 만든다.
- 실제 주행 안전과 관련된 항목은 STOP/RUN/E-STOP 개념을 분리해서 설명한다.
- 시험/평가가 없는 수업이므로 "평가", "채점", "시험 대비"라는 표현은 피하고,
  "실습 체크포인트", "확인 질문", "선택 과제", "데모 목표"라는 표현을 사용한다.
