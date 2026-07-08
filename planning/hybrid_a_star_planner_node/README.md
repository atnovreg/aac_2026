# Hybrid A* Planner Node

Ackermann vehicle용 전역 Hybrid A* planner입니다. RViz의 `2D Goal Pose`
도구가 publish하는 `/goal_pose`를 받으면 현재 위치부터 목표까지의
`nav_msgs/Path`를 생성합니다.

주차장 미션을 위해 OMPL의 Dubins/Reeds-Shepp state space를 analytic
expansion으로 사용합니다. 기본 설정은 `reed_shepp` + reverse search이며,
목표 근처에서 충돌 없는 Reeds-Shepp 곡선이 나오면 grid search를 끝내고
그 곡선을 path tail로 붙입니다.

## 실행

```bash
ros2 launch hybrid_a_star_planner_node hybrid_a_star_planner.launch.py use_sim_time:=true
```

기본 시작 위치는 TF의 `map -> base_footprint`입니다. TF 없이 RViz의
`2D Pose Estimate`를 시작점으로 쓰려면 다음처럼 실행합니다.

```bash
ros2 launch hybrid_a_star_planner_node hybrid_a_star_planner.launch.py \
  use_sim_time:=true \
  start_pose_source:=initialpose
```

현재 `/map`이 없거나 stale map 때문에 goal이 collision으로 거절되면,
free-space fallback만 쓰도록 아래처럼 실행할 수 있습니다.

```bash
ros2 launch hybrid_a_star_planner_node hybrid_a_star_planner.launch.py \
  use_sim_time:=true \
  use_occupancy_grid:=false
```

전진만 허용하는 일반 주행/Dubins 모드로 쓰려면:

```bash
ros2 launch hybrid_a_star_planner_node hybrid_a_star_planner.launch.py \
  use_sim_time:=true \
  reverse_enabled:=false \
  analytic_expansion_type:=dubins
```

## 토픽

- 입력: `/goal_pose` (`geometry_msgs/PoseStamped`)
- 입력: `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, 선택)
- 입력: `/map` (`nav_msgs/OccupancyGrid`, 선택)
- 출력: `/hybrid_astar/path` (`nav_msgs/Path`)
- 출력: `/hybrid_astar/markers` (`visualization_msgs/MarkerArray`)

`/map`이 없으면 `config/hybrid_a_star_planner.yaml`의 fallback bounds 안을
free space로 보고 계획합니다. 현재 local planner에 바로 연결하려면
local planner의 `global_path_topic`을 `/hybrid_astar/path`로 바꿔서 쓰면 됩니다.

주의: `nav_msgs/Path`에는 기어/진행방향 필드가 없습니다. Reeds-Shepp path의
후진 구간은 인접 pose delta와 pose yaw의 dot product가 음수인 구간으로 추정할
수 있습니다. RViz marker에서는 전진 구간을 초록색, 후진 구간을 주황색으로
표시합니다. 실제 주차 제어까지 하려면 downstream controller가 이 방향 정보를
사용해 reverse command를 낼 수 있어야 합니다.
