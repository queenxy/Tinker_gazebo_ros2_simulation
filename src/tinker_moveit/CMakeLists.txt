cmake_minimum_required(VERSION 3.8)
project(tinker_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(moveit REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(tinker_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(tinker_arm_controller src/arm_controller.cpp)
ament_target_dependencies(tinker_arm_controller rclcpp moveit_msgs moveit_ros_planning_interface moveit_ros_move_group tinker_msgs tf2 tf2_ros geometry_msgs)


install(TARGETS
  tinker_arm_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
