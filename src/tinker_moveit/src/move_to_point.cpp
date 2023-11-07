/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "tinker_msgs/srv/ur_control_service.hpp"

// void reach(moveit::planning_interface::MoveGroupInterface* move_group,
//           const std::shared_ptr<tinker_msgs::srv::URControlService::Request> request,
//           std::shared_ptr<tinker_msgs::srv::URControlService::Response>      response)
// {
//     move_group->setPoseTarget(request->target_pose);

//     // Now, we call the planner to compute the plan and visualize it.
//     // Note that we are just planning, not asking move_group
//     // to actually move the robot.
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//     bool plan_success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     bool move_success = (move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);

//     // RCLCPP_INFO(LOGGER, "Planning (pose goal) %s", plan_success ? "" : "FAILED");
//     // RCLCPP_INFO(LOGGER, "Move to (pose goal) %s", move_success ? "" : "FAILED");

//     response->success = move_success;

// }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("tinker_moveit_node");

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  //set up planning group
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // // set up the service 
  // rclcpp::Service<tinker_msgs::srv::URControlService>::SharedPtr service =
  //   move_group_node->create_service<tinker_msgs::srv::URControlService>("tinker_moveit_node", std::bind(&reach, &move_group, std::placeholders::_1, std::placeholders::_2));
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.2;
  target_pose.position.z = 1.0;

  move_group.setPoseTarget(target_pose);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool plan_success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool move_success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
  // rclcpp::spin(move_group_node);
  // rclcpp::shutdown();
  
  return 0;
}