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


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "abb_data/action/move_l.hpp"
#include "abb_move_group_interface/abb_movel_action_server.hpp"

// Declaration of global constants:
const double pi = 3.14159265358979;

namespace composition
{

MoveLActionServer::MoveLActionServer(const rclcpp::NodeOptions &options)
: Node("MoveLActionServer", options)
{}  

MoveLActionServer::MoveLActionServer(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_obj, const rclcpp::NodeOptions &options)
: Node("MoveLActionServer", options)
{
  robot_arm = move_group_interface_obj;

  this->action_server_ = rclcpp_action::create_server<MoveL>(
    this,
    "MoveL",
    std::bind(&MoveLActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MoveLActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&MoveLActionServer::handle_accepted, this, std::placeholders::_1));
}  

rclcpp_action::GoalResponse MoveLActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveL::Goal> goal)
{
    // auto JointGoal = ros2_data::msg::JointPose();
    // JointGoal =  goal->goal;
    RCLCPP_INFO(get_logger(), "Received a goal request, with joint pose:");
    //(void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept and execute the goal received.
}

void MoveLActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread(
        [this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    
}

rclcpp_action::CancelResponse MoveLActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
    robot_arm->stop();

    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveLActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting MoveL motion to desired waypoint...");

    const auto goal = goal_handle->get_goal();

    // double j1 = goal->joint1;
    // double j2 = goal->joint2;
    // double j3 = goal->joint3;
    // double j4 = goal->joint4;
    // double j5 = goal->joint5;
    // double j6 = goal->joint6;
    // double j7 = goal->joint7;

    // RCLCPP_INFO(this->get_logger(), "J1 '%f'", j1);
    // RCLCPP_INFO(this->get_logger(), "J2 '%f'", j2);
    // RCLCPP_INFO(this->get_logger(), "J3 '%f'", j3);
    // RCLCPP_INFO(this->get_logger(), "J4 '%f'", j4);
    // RCLCPP_INFO(this->get_logger(), "J5 '%f'", j5);
    // RCLCPP_INFO(this->get_logger(), "J6 '%f'", j6);
    // RCLCPP_INFO(this->get_logger(), "J7 '%f'", j7);


    auto result = std::make_shared<MoveL::Result>();

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = goal->ox;
    target_pose1.orientation.y = goal->oy;
    target_pose1.orientation.z = goal->oz;
    target_pose1.orientation.w = goal->ow;

    target_pose1.position.x = goal->x;
    target_pose1.position.y = goal->y;
    target_pose1.position.z = goal->z;
    robot_arm->setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (robot_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    robot_arm->execute(my_plan);

    result->result = "MoveJ:SUCCEEED";
    goal_handle->succeed(result);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MoveLActionServer)
