cmake_minimum_required(VERSION 3.22)
project(new_cobot_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(ee_velocity src/ee_velocity.cpp)
ament_target_dependencies(ee_velocity
  rclcpp
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

add_executable(multi_waypoint_cartesian src/multi_waypoint_cartesian.cpp)
ament_target_dependencies(multi_waypoint_cartesian
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  Eigen3
)

install(TARGETS
  ee_velocity
  multi_waypoint_cartesian
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf meshes config launch src
    DESTINATION share/${PROJECT_NAME})

ament_package()