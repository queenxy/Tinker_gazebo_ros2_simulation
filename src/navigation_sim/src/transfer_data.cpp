#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DataTransfer : public rclcpp::Node
{
  public:
    DataTransfer()
    : Node("data_transfer"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/tinker_controller/cmd_vel_unstamped", 10);
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&DataTransfer::topic_callback, this, _1));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    size_t count_;
    geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
    void topic_callback(const geometry_msgs::msg::Twist & msg) const
    {
        publisher_->publish(msg);
    } 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataTransfer>());
  rclcpp::shutdown();
  return 0;
}