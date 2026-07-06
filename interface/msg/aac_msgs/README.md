# aac_msgs

ROS 2 interface package for AAC planning messages.

This package is intentionally small. It currently keeps only the messages used
by `frenet_local_planning_node`:

- `PathWithCost`
- `PathWithCostArray`

The older ROS 1 perception/control message definitions were removed because
they are not used by the current frenet local planner.
