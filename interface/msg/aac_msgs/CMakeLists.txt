cmake_minimum_required(VERSION 3.8)
project(aac_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MpccStateStamped.msg"
  "msg/PathWithCost.msg"
  "msg/PathWithCostArray.msg"
  DEPENDENCIES
    nav_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
