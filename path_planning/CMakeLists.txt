cmake_minimum_required(VERSION 3.20)
project(path_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(bonxai_ros REQUIRED)   # Bonxai: bonxai_core + bonxai_map (ProbabilisticMap)
find_package(ompl REQUIRED)

# Map and collision checks, without ROS: also linked by the unit tests.
add_library(${PROJECT_NAME}_map STATIC
  src/occupancy_map.cpp
  src/collision_world.cpp
)
set_target_properties(${PROJECT_NAME}_map PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_map PUBLIC
  bonxai_ros::bonxai_map
  Eigen3::Eigen
)

add_library(${PROJECT_NAME} SHARED
  src/informed_rrt_planner.cpp
  src/path_trajectory.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_map
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  message_filters::message_filters
  ${px4_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
  ${OMPL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "path_planning::InformedRRTPlanner"
  EXECUTABLE path_planning_node
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY sim DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_trajectory test/test_path_trajectory.cpp src/path_trajectory.cpp)
  target_include_directories(test_path_trajectory PRIVATE include)
  target_link_libraries(test_path_trajectory Eigen3::Eigen)

  ament_add_gtest(test_occupancy_map test/test_occupancy_map.cpp)
  target_link_libraries(test_occupancy_map ${PROJECT_NAME}_map)
endif()

ament_package()
