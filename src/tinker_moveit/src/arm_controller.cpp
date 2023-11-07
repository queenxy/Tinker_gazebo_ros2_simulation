#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <tinker_msgs/srv/ur_control_service.hpp>

#include <memory>

using namespace std;

class ArmController : public rclcpp::Node
{
  public:
    ArmController():
      Node("tinker_arm_controller"),
      move_group(nullptr)    //move group class need the shared_ptr of current node, so we initialize it later.
    {
        service = this->create_service<tinker_msgs::srv::URControlService>("tinker_arm_control_service", 
                            std::bind(&ArmController::callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    bool init_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _move_group) {
        move_group = _move_group;
    }

  private:
    rclcpp::Service<tinker_msgs::srv::URControlService>::SharedPtr service;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;    // we pass the shared_ptr of move group
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group;

    void callback(const std::shared_ptr<tinker_msgs::srv::URControlService::Request> request,
          std::shared_ptr<tinker_msgs::srv::URControlService::Response> response);

};

void ArmController::callback(const std::shared_ptr<tinker_msgs::srv::URControlService::Request> request,
          std::shared_ptr<tinker_msgs::srv::URControlService::Response> response)
{   
    move_group->setPoseTarget(request->target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool plan_success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool move_success = (move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(this->get_logger(), "Planning (pose goal) %s", plan_success ? "" : "FAILED");
    RCLCPP_INFO(this->get_logger(), "Move to (pose goal) %s", move_success ? "" : "FAILED");

    response->success = move_success;

}

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);

    auto arm_controller_node = std::make_shared<ArmController>();
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_controller_node, "ur_manipulator");
    arm_controller_node->init_move_group(move_group);

    
    rclcpp::spin(arm_controller_node);
    rclcpp::shutdown();
    return 0;
}