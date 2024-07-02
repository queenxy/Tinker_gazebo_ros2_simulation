#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tinker_msgs/srv/ur_control_service.hpp>

#include <memory>
#include <iostream>

using namespace std;

class ArmController : public rclcpp::Node
{
  public:
    ArmController():
      Node("tinker_arm_controller"),
      move_group(nullptr)    //move group class need the shared_ptr of current node, so we initialize it later.
    {
        control_service = this->create_service<tinker_msgs::srv::URControlService>("tinker_arm_control_service", 
                            std::bind(&ArmController::callback, this, std::placeholders::_1, std::placeholders::_2));
        initial_service = this->create_service<std_srvs::srv::Trigger>("tinker_arm_initial_service", 
                            std::bind(&ArmController::initial, this, std::placeholders::_1, std::placeholders::_2));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    bool init_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _move_group) {
        move_group = _move_group;
        // current_state = move_group->getCurrentState(10)
    }

  private:
    rclcpp::Service<tinker_msgs::srv::URControlService>::SharedPtr control_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initial_service;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;    // we pass the shared_ptr of move group
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // moveit::core::RobotStatePtr current_state = move_group->getCurrentState(10);

    void callback(const std::shared_ptr<tinker_msgs::srv::URControlService::Request> request,
          std::shared_ptr<tinker_msgs::srv::URControlService::Response> response);
    void initial(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};

void ArmController::callback(const std::shared_ptr<tinker_msgs::srv::URControlService::Request> request,
          std::shared_ptr<tinker_msgs::srv::URControlService::Response> response)
{   
    geometry_msgs::msg::Pose target_pose;
    target_pose = request->target_pose;
    // cout << target_pose.position.x << endl;

    // geometry_msgs::msg::TransformStamped t;
    // try {
    //       t = tf_buffer_->lookupTransform(
    //         "base_link", "wrist_camera",
    //         tf2::TimePointZero);
    //     } catch (const tf2::TransformException & ex) {
    //       RCLCPP_INFO(
    //         this->get_logger(), "Could not transform wrist_camera to base_link");
    //       return;
    //     }

    // geometry_msgs::msg::Vector3 translation;
    // geometry_msgs::msg::Quaternion quaternion;
    // translation = t.transform.translation;
    // quaternion = t.transform.rotation;

    // cout << target_pose.position.x <<' ' << target_pose.position.y << ' ' <<target_pose.position.z << endl;
    // cout << translation.x << ' ' <<translation.y << ' ' << translation.z << endl;

    // tf2::Quaternion q_ori, q_rot, q_new;
    // tf2::convert(target_pose.orientation,q_ori);
    // tf2::convert(quaternion,q_rot);
    // tf2::Matrix3x3 mat(q_rot);

    // float x_ori, y_ori, z_ori;
    // x_ori = target_pose.position.x;
    // y_ori = target_pose.position.y;
    // z_ori = target_pose.position.z;
    // target_pose.position.x = x_ori * mat[0][0] + y_ori * mat[0][1] + z_ori * mat[0][2] + translation.x;
    // target_pose.position.y = x_ori * mat[1][0] + y_ori * mat[1][1] + z_ori * mat[1][2] + translation.y;
    // target_pose.position.z = x_ori * mat[2][0] + y_ori * mat[2][1] + z_ori * mat[2][2] + translation.z;
    // q_new = q_rot * q_ori;
    // q_new.normalize();
    // target_pose.orientation = tf2::toMsg(q_new);
    // cout << target_pose.position.x << ' ' << target_pose.position.y << ' ' << target_pose.position.z << endl;
    
    move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool plan_success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool move_success = (move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(this->get_logger(), "Planning (pose goal) %s", plan_success ? "" : "FAILED");
    RCLCPP_INFO(this->get_logger(), "Move to (pose goal) %s", move_success ? "" : "FAILED");

    response->success = move_success;

}

void ArmController::initial(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{   
    std::vector<double> joint_group_positions(6);
    
    joint_group_positions[0] = -2.35;  // radians
    joint_group_positions[1] = -1.22;
    joint_group_positions[2] = -0.785;
    joint_group_positions[3] = -2.1;
    joint_group_positions[4] = -4.65;
    joint_group_positions[5] = 0.0;

    bool within_bounds = move_group->setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
      RCLCPP_WARN(this->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }

    move_group->setMaxVelocityScalingFactor(0.05);
    move_group->setMaxAccelerationScalingFactor(0.05);

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