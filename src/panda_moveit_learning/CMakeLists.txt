cmake_minimum_required(VERSION 3.16)
project(panda_moveit_learning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(joint_goal_demo src/joint_goal_demo.cpp)
target_compile_features(joint_goal_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  joint_goal_demo
  rclcpp
  moveit_ros_planning_interface
)

add_executable(pose_goal_demo src/pose_goal_demo.cpp)
target_compile_features(pose_goal_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  pose_goal_demo
  geometry_msgs
  rclcpp
  moveit_ros_planning_interface
)

add_executable(planning_scene_obstacle_demo src/planning_scene_obstacle_demo.cpp)
target_compile_features(planning_scene_obstacle_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  planning_scene_obstacle_demo
  geometry_msgs
  moveit_msgs
  rclcpp
  shape_msgs
  std_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  joint_goal_demo
  pose_goal_demo
  planning_scene_obstacle_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
