cmake_minimum_required(VERSION 3.8)
project(moveit_utils_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(dual_arm_pose_commander src/dual_arm_pose_commander.cpp)
target_compile_features(dual_arm_pose_commander PUBLIC cxx_std_17)
ament_target_dependencies(dual_arm_pose_commander
  rclcpp geometry_msgs moveit_ros_planning_interface moveit_msgs)
target_link_libraries(dual_arm_pose_commander Threads::Threads)

install(TARGETS dual_arm_pose_commander DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright/license metadata is inherited and still needs an owner decision.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
