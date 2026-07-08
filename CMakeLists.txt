cmake_minimum_required(VERSION 3.8)
project(tomato_arm_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_executable(arm_ik src/arm_ik.cpp)
ament_target_dependencies(arm_ik
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(arm_ik_joy src/arm_ik_joy.cpp)
ament_target_dependencies(arm_ik_joy
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

add_executable(crane_control src/crane_control.cpp)
ament_target_dependencies(crane_control
  rclcpp
  sensor_msgs
  dynamixel_sdk
)

add_executable(ax_mx_joy src/ax_mx_joy.cpp)
ament_target_dependencies(ax_mx_joy
  rclcpp
  sensor_msgs
  dynamixel_sdk
)

install(TARGETS
  arm_ik
  arm_ik_joy
  crane_control
  ax_mx_joy
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
