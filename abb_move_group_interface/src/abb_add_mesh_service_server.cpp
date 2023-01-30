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

#include "abb_data/srv/add_mesh.hpp"
#include "abb_move_group_interface/abb_add_mesh_service_server.hpp"

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

// Declaration of global constants:
const double pi = 3.14159265358979;

namespace composition
{

AddMeshServer::AddMeshServer(const rclcpp::NodeOptions &options)
: Node("AddMeshServer", options)
{
    srv_ = create_service<AddMesh>("AddMesh", std::bind(&AddMeshServer::add_mesh_obejct, this, std::placeholders::_1, std::placeholders::_2));
}  


void AddMeshServer::add_mesh_obejct(const std::shared_ptr<AddMesh::Request> request,
                                    std::shared_ptr<AddMesh::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received a goal request, with joint pose:");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const auto id = request->id;
    const auto path = request->path;

    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = "base_link";

    collision_object2.id = request->id;

    Eigen::Vector3d b(0.01, 0.01, 0.01);
    // shapes::Mesh * m = shapes::createMeshFromResource("file:///root/ws_robot_panda/src/moveit_tutorial_abb/panda_resource/meshes/visual/link0.dae");
    // shapes::Mesh * m = shapes::createMeshFromResource("file:///root/ws_robot_panda/src/moveit_tutorial_abb/panda_resource/meshes/collision/link0.stl");
    shapes::Mesh * m = shapes::createMeshFromResource("file://" + request->path);

    shape_msgs::msg::Mesh shelf_mesh;
    shapes::ShapeMsg shelf_mesh_msg;
    shapes::constructMsgFromShape(m,shelf_mesh_msg);
    shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

    geometry_msgs::msg::Pose box_pose2;
    box_pose2.orientation.z = request->ox;
    box_pose2.orientation.y = request->oy;
    box_pose2.orientation.z = request->oz;
    box_pose2.orientation.w = request->ow;
    box_pose2.position.x = request->x;
    box_pose2.position.y = request->y;
    box_pose2.position.z = request->z;

    collision_object2.meshes.push_back(shelf_mesh);
    collision_object2.mesh_poses.push_back(box_pose2);

    collision_object2.operation = collision_object2.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects2;
    collision_objects2.push_back(collision_object2);
    planning_scene_interface.addCollisionObjects(collision_objects2);

    RCLCPP_INFO(this->get_logger(), "Add an object into the world");

    response->result =  "Succees";
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(composition::AddMeshServer)
