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

// #include <moveit_visual_tools/moveit_visual_tools.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "abb_move_group_interface/visibility_control.h"

#include "abb_move_group_interface/abb_movej_action_server.hpp"
#include "abb_move_group_interface/abb_movep_action_server.hpp"
#include "abb_move_group_interface/abb_movel_action_server.hpp"
#include "abb_move_group_interface/abb_add_mesh_service_server.hpp"


int main(int argc, char** argv)
{
    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);

    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto move_group_node = std::make_shared<rclcpp::Node>(
        "moveit2_interface", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    rclcpp::executors::MultiThreadedExecutor exec;

    // get urdf file 
    std::string urdf_file = move_group_node->get_parameter("robot_description").get_parameter_value().get<std::string>();

    // get list of prefixes
    std::vector<std::string> prefixes = move_group_node->get_parameter("prefixes_list").get_parameter_value().get<std::vector<std::string>>();

    // create group names list
    std::vector<std::string> group_names_list;
    for (int i = 0; i < prefixes.size(); ++i)
    {
        std::cout << prefixes[i] << std::endl;
        group_names_list.push_back(prefixes[i] + "_manipulator");
    }
    
    // List of servers to get them in main scope
    std::vector<std::shared_ptr<composition::MoveJActionServer> > movej_action_servers_list;
    std::vector<std::shared_ptr<composition::MovePActionServer> > movep_action_servers_list;
    std::vector<std::shared_ptr<composition::MoveLActionServer> > movel_action_servers_list;

    rclcpp::NodeOptions options;

    for (int i = 0; i < group_names_list.size(); ++i)
    {
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, group_names_list[i]);

        options.arguments({"--ros-args", "-r", "__node:=MoveJ_action_server"});
        auto movej_action_server = std::make_shared<composition::MoveJActionServer>(move_group, group_names_list[i], options);  
        movej_action_servers_list.push_back(movej_action_server);
        exec.add_node(movej_action_server);

        options.arguments({"--ros-args", "-r", "__node:=MoveP_action_server"});
        auto movep_action_server = std::make_shared<composition::MovePActionServer>(move_group, group_names_list[i], urdf_file, options);  
        movep_action_servers_list.push_back(movep_action_server);
        exec.add_node(movep_action_server);

        options.arguments({"--ros-args", "-r", "__node:=MoveL_action_server"});
        auto movel_action_server = std::make_shared<composition::MoveLActionServer>(move_group, group_names_list[i], urdf_file, options);  
        movel_action_servers_list.push_back(movel_action_server);
        exec.add_node(movel_action_server);

    }

    // auto add_mesh_service_server = std::make_shared<composition::AddMeshServer>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));  
    // exec.add_node(add_mesh_service_server );

    exec.spin();

    rclcpp::shutdown();
    return 0;
}