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

#ifndef ABB_MOVE_GROUP_INTERFACE__ADD_MESH_SERVER_HPP_
#define ABB_MOVE_GROUP_INTERFACE__ADD_MESH_SERVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "abb_move_group_interface/visibility_control.h"
#include "abb_data/srv/add_mesh.hpp"


namespace composition
{

class AddMeshServer : public rclcpp::Node
{
public:
  ABB_MOVE_GROUP_INTERFACE_PUBLIC
  explicit AddMeshServer(const rclcpp::NodeOptions & options);
  
  using AddMesh = abb_data::srv::AddMesh;

private:
  void add_mesh_obejct(const std::shared_ptr<AddMesh::Request> request,
                       const std::shared_ptr<AddMesh::Response> response);

  rclcpp::Service<AddMesh>::SharedPtr srv_;
};

}  // namespace composition

#endif  // ABB_MOVE_GROUP_INTERFACE__MOVEL_ACTION_SERVER_HPP_