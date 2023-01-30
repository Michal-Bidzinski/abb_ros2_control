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

#ifndef ABB_MOVE_GROUP_INTERFACE__MOVEJ_ACTION_SERVER_HPP_
#define ABB_MOVE_GROUP_INTERFACE__MOVEJ_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "abb_move_group_interface/visibility_control.h"
#include "abb_data/action/move_j.hpp"


namespace composition
{

class MoveJActionServer : public rclcpp::Node
{
public:
  ABB_MOVE_GROUP_INTERFACE_PUBLIC
  explicit MoveJActionServer(const rclcpp::NodeOptions & options);
  ABB_MOVE_GROUP_INTERFACE_PUBLIC
  explicit MoveJActionServer(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_obj, std::string move_group_name, const rclcpp::NodeOptions & options);
  
  using MoveJ = abb_data::action::MoveJ;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveJ>;

private:

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const MoveJ::Goal> goal);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_action::Server<MoveJ>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> robot_arm;
  std::string group_name;
};

}  // namespace composition

#endif  // ABB_MOVE_GROUP_INTERFACE__MOVEJ_ACTION_SERVER_HPP_