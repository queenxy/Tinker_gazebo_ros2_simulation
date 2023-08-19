cmake_minimum_required(VERSION 3.8)
project(tinker_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# find_package(<dependency> REQUIRED)

generate_parameter_library(tinker_controller_parameters
  src/tinker_controller_parameter.yaml
)

add_library(tinker_controller SHARED
  src/tinker_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_compile_features(tinker_controller PUBLIC cxx_std_17)
target_include_directories(tinker_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/tinker_controller>
)
target_link_libraries(tinker_controller PUBLIC tinker_controller_parameters)
ament_target_dependencies(tinker_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tinker_controller PRIVATE "TINKER_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface tinker_controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_tinker_controller
    test/test_load_tinker_controller.cpp
  )
  ament_target_dependencies(test_load_tinker_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/tinker_controller
)
install(TARGETS tinker_controller tinker_controller_parameters
  EXPORT export_tinker_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES tinker_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_tinker_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
