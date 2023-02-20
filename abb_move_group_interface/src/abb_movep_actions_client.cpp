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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "abb_move_group_interface/visibility_control.h"

#include "abb_data/action/move_p.hpp"
#include "abb_data/msg/position.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_action_client");

  using MoveP = abb_data::action::MoveP;
  using Position = abb_data::msg::Position;

  auto action_client = rclcpp_action::create_client<MoveP>(node, "/dual_ur_manipulator/MoveP");


  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return 1;
  }


  // Populate a goal
  auto goal_msg = MoveP::Goal();
  Position r1_pose;
  Position r2_pose;

  // r1_pose.x = 0.28;
  // r1_pose.y = 0.2-0.3;
  // r1_pose.z = 0.6;
  // r1_pose.ox = 1.0;
  // r1_pose.oy = 0.0;
  // r1_pose.oz = 0.0;
  // r1_pose.ow = 0.0;

  r1_pose.x = 0.353;
  r1_pose.y = 0.086;
  r1_pose.z = 0.367;
  r1_pose.ox = 0.5;
  r1_pose.oy = 0.5;
  r1_pose.oz = 0.5;
  r1_pose.ow = 0.5;
  r1_pose.frame = "r1_tool0";

  // r1_pose.x = 0.362;
  // r1_pose.y = 0.2;
  // r1_pose.z = 0.345;
  // r1_pose.ox = 0.1;
  // r1_pose.oy = 0.0;
  // r1_pose.oz = 0.0;
  // r1_pose.ow = 0.0;
  // r1_pose.frame = "r1_endeffector";

  // r2_pose.x = 0.28;
  // r2_pose.y = 0.2+0.3;
  // r2_pose.z = 0.6;
  // r2_pose.ox = 1.0;
  // r2_pose.oy = 0.0;
  // r2_pose.oz = 0.0;
  // r2_pose.ow = 0.0;

  goal_msg.positions = {r1_pose};

  RCLCPP_INFO(node->get_logger(), "Sending goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<MoveP>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<MoveP>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return 1;
  }

  RCLCPP_INFO(node->get_logger(), "result received %s", wrapped_result.result->result.c_str());

  
  RCLCPP_INFO(node->get_logger(), "result received %i", wrapped_result.result->joint_trajectory.size());


  for (std::size_t i = 0; i <  wrapped_result.result->joint_trajectory.size(); ++i)
  {
      for (std::size_t j = 0; j <  wrapped_result.result->joint_trajectory[i].joints.size(); ++j)
      {
          std::cout << "Point " <<  i << " " << j << " :" << wrapped_result.result->joint_trajectory[i].joints[j] * 180.0/3.14 << std::endl;
      }
  }


  rclcpp::shutdown();
  return 0;
}
