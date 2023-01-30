// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ABB_MOVE_GROUP_INTERFACE__MOVEP_ACTION_SERVER_HPP_
#define ABB_MOVE_GROUP_INTERFACE__MOVEP_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "abb_move_group_interface/visibility_control.h"
#include "abb_data/action/move_p.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"


namespace composition
{

class MovePActionServer : public rclcpp::Node
{
public:
  ABB_MOVE_GROUP_INTERFACE_PUBLIC
  explicit MovePActionServer(const rclcpp::NodeOptions & options);
  ABB_MOVE_GROUP_INTERFACE_PUBLIC
  explicit MovePActionServer(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_obj, std::string move_group_name, std::string urdf_file, const rclcpp::NodeOptions & options);
  
  using MoveP = abb_data::action::MoveP;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveP>;

private:

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const MoveP::Goal> goal);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_action::Server<MoveP>::SharedPtr action_server_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> robot_arm;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::string group_name;
  std::string urdf_arm;
};

}  // namespace composition

#endif  // ABB_MOVE_GROUP_INTERFACE__MOVEP_ACTION_SERVER_HPP_