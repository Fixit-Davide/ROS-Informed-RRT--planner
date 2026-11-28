cmake_minimum_required(VERSION 3.20)
project(uav_planners)

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
find_package(pluginlib REQUIRED)
find_package(uav_nav_core REQUIRED)
find_package(ompl REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/astar.cpp
  src/astar_planner.cpp
  src/informed_rrt_star.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  uav_nav_core::uav_nav_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  ${OMPL_LIBRARIES}
)

pluginlib_export_plugin_description_file(uav_nav_core plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar test/test_astar.cpp src/astar.cpp)
  target_include_directories(test_astar PRIVATE include)
  target_link_libraries(test_astar uav_nav_core::uav_nav_core)
  ament_add_gtest(test_plugins test/test_plugins.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_plugins uav_nav_core::uav_nav_core pluginlib::pluginlib gtest)
endif()

ament_package()
