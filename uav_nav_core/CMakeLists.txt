cmake_minimum_required(VERSION 3.20)
project(uav_nav_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(bonxai_ros REQUIRED)   # Bonxai: bonxai_core + bonxai_map (ProbabilisticMap)

# Map, collision checks, polylines, the VoxelMap type adapter and the planner plugin
# interfaces: everything the servers share.
add_library(${PROJECT_NAME} SHARED
  src/collision_world.cpp
  src/occupancy_map.cpp
  src/polyline.cpp
  src/voxel_map.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  bonxai_ros::bonxai_map
  Eigen3::Eigen
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${uav_nav_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_occupancy_map test/test_occupancy_map.cpp)
  target_link_libraries(test_occupancy_map ${PROJECT_NAME})
  ament_add_gtest(test_polyline test/test_polyline.cpp)
  target_link_libraries(test_polyline ${PROJECT_NAME})
  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_voxel_map ${PROJECT_NAME} gtest)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle std_msgs uav_nav_msgs Eigen3 bonxai_ros)
ament_package()
