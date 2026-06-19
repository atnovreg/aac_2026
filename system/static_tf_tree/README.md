# static_tf_tree

AAC의 고정 TF를 한 노드와 한 YAML에서 관리하는 ROS 2 패키지다.

```bash
ros2 launch static_tf_tree static_tf_tree.launch.py use_sim_time:=true
```

기본 구성:

```text
map -> odom
lidar_link -> aac_vehicle/base_footprint/lidar
base_footprint_lidar -> lidar_link_lidar
```

변환 값과 활성화 여부는 `config/static_tf_tree.yaml`에서 관리한다.

`map -> odom`은 현재 실험을 위한 identity fallback이다. 실제 localization이
동적인 `map -> odom`을 발행할 때는 `map_to_odom.enabled`를 `false`로 바꿔
동일한 child frame을 두 노드가 발행하지 않도록 해야 한다.
