cmake_minimum_required(VERSION 3.20)
project(uav_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ControllerState.msg
  msg/VoxelMap.msg
  action/ComputePath.action
  action/FollowPath.action
  srv/GetReplanStart.srv
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
