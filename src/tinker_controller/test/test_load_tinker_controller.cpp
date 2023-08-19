#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadTinkerController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(ros2_control_test_assets::diffbot_urdf),
    executor, "test_controller_manager");

  ASSERT_NE(
    cm.load_controller("test_tinker_controller", "tinker_controller/TinkerController"),
    nullptr);

  rclcpp::shutdown();
}