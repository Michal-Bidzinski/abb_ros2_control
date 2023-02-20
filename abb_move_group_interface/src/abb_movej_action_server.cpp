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

#include "abb_data/action/move_j.hpp"
#include "abb_data/msg/joints.hpp"
#include "abb_move_group_interface/abb_movej_action_server.hpp"

// Declaration of global constants:
const double pi = 3.14159265358979;

namespace composition
{

MoveJActionServer::MoveJActionServer(const rclcpp::NodeOptions &options)
: Node("MoveJActionServer", options)
{}  

MoveJActionServer::MoveJActionServer(std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
                                     move_group_interface_obj, std::string move_group_name, 
                                     const rclcpp::NodeOptions &options)
: Node("MoveJActionServer", move_group_name, options)
{
  robot_arm = move_group_interface_obj;
  group_name = move_group_name;

  this->action_server_ = rclcpp_action::create_server<MoveJ>(
    this,
    "MoveJ",
    std::bind(&MoveJActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MoveJActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&MoveJActionServer::handle_accepted, this, std::placeholders::_1));
}  

rclcpp_action::GoalResponse MoveJActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJ::Goal> goal)
{
    //TODO
    RCLCPP_INFO(get_logger(), "Received a goal request, with joint pose:");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
}

void MoveJActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread:
    std::thread(
        [this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    
}

rclcpp_action::CancelResponse MoveJActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
    robot_arm->stop();

    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveJActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting MoveJ motion to desired point...");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveJ::Result>();
    
    robot_arm->setPlannerId("PTP");
    
    // get current state
    const moveit::core::JointModelGroup* joint_model_group = 
        robot_arm->getCurrentState()->getJointModelGroup(group_name);

    moveit::core::RobotStatePtr current_state = robot_arm->getCurrentState(10);

    // copy positions
    std::vector<double> joint_goal_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_goal_positions);

    // check number of joints goal positions
    if (joint_goal_positions.size() < (goal->joints_state.size() * 6))
    {
        result->result = "MoveJ:FAIL, too many joints goal positions";
        goal_handle->abort(result);
    }

    else
    {
        // set target 
        for (std::size_t i = 0; i < goal->joints_state.size(); ++i)
        {
            joint_goal_positions[i*6+0] = goal->joints_state[i].joints[0]; 
            joint_goal_positions[i*6+1] = goal->joints_state[i].joints[1]; 
            joint_goal_positions[i*6+2] = goal->joints_state[i].joints[2]; 
            joint_goal_positions[i*6+3] = goal->joints_state[i].joints[3]; 
            joint_goal_positions[i*6+4] = goal->joints_state[i].joints[4]; 
            joint_goal_positions[i*6+5] = goal->joints_state[i].joints[5]; 
        }

        robot_arm->setJointValueTarget(joint_goal_positions);

        // plan and execute path
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        bool success = 
            (robot_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        robot_arm->execute(my_plan);


        for (std::size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); ++i)
        {
            // Joints trajectory;
            for (std::size_t j = 0; j < my_plan.trajectory_.joint_trajectory.points[i].positions.size(); ++j)
            {
                // trajectory.joints.push_back(my_plan.trajectory_.joint_trajectory.points[i].positions[j]);
                std::cout << "Point " <<  i << " " << j << " :" << my_plan.trajectory_.joint_trajectory.points[i].positions[j] * 180.0/3.14 << std::endl;
            }
            // joint_trajectory.push_back(trajectory);
        }


        bool executed = false;
        int count = 0;

        // check accuracy of execution
        while (!executed  && rclcpp::ok() && count <=20)
        {
            std::vector<double> joint_current_positions;
            current_state = robot_arm->getCurrentState(10);
            current_state->copyJointGroupPositions(joint_model_group, joint_current_positions);

            executed = true;
            
            for (std::size_t i = 0; i < joint_current_positions.size(); ++i)
            {
                if (abs(joint_current_positions[i] - joint_goal_positions[i]) > 0.0137)
                {
                    executed = false;
                } 
            }
            if (!executed)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                count++;
            }
        }
        // return result
        if (executed)
        {
            result->result = "SUCCEED";
        }
        else
        {
            result->result = "FAIL";
        }
        
        goal_handle->succeed(result);
    }
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MoveJActionServer)
