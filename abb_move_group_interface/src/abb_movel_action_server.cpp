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
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "abb_data/action/move_l.hpp"
#include "abb_data/msg/position.hpp"
#include "abb_move_group_interface/abb_movel_action_server.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"

#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include <iostream>


// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

// Declaration of global constants:
const double pi = 3.14159265358979;

namespace composition
{

MoveLActionServer::MoveLActionServer(const rclcpp::NodeOptions &options)
: Node("MoveLActionServer", options)
{}  

MoveLActionServer::MoveLActionServer(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_obj, std::string move_group_name, std::string urdf_file, const rclcpp::NodeOptions &options)
: Node("MoveLActionServer", move_group_name, options)
{
    robot_arm = move_group_interface_obj;
    group_name = move_group_name;
    urdf_arm = urdf_file;

    this->action_server_ = rclcpp_action::create_server<MoveL>(
        this,
        "MoveL",
        std::bind(&MoveLActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveLActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveLActionServer::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options());    
}  

rclcpp_action::GoalResponse MoveLActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveL::Goal> goal)
{
    //TODO
    RCLCPP_INFO(get_logger(), "Received a goal request, with joint pose:");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
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
    auto result = std::make_shared<MoveL::Result>();

    robot_arm->setPlannerId("LIN");

    // create model from xml 
    pinocchio::Model model;
    std::cout << "urdf_ arm" << urdf_arm << std::endl;
    pinocchio::urdf::buildModelFromXML(urdf_arm,model);
    
    // create data required by the algorithms
    pinocchio::Data data(model);  

    std::vector<Eigen::VectorXd> targets(goal->positions.size());
    bool IK_success_all = true;

    for (std::size_t i = 0; i < goal->positions.size(); ++i)
    {
        // set goal
        int ID_endeffector = model.getFrameId(goal->positions[0].frame);
        std::cout << "ID:" << std::endl << ID_endeffector << std::endl;

        Eigen::Quaterniond quat;
        quat.x() = goal->positions[0].ox;
        quat.y() = goal->positions[0].oy;
        quat.z() = goal->positions[0].oz;
        quat.w() = goal->positions[0].ow;

        Eigen::Matrix3d R = quat.normalized().toRotationMatrix();

        const pinocchio::SE3 oMdes(R, Eigen::Vector3d(goal->positions[0].x, goal->positions[0].y, goal->positions[0].z));

        // parameters for IK
        Eigen::VectorXd q = pinocchio::neutral(model);
        const double eps  = 1e-1;
        const int IT_MAX  = 4000;
        const double DT   = 1e-1;
        const double damp = 1e-12;
        
        // count IK
        pinocchio::Data::Matrix6x J(6,model.nv);
        J.setZero();
        
        bool success = false;
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        Vector6d err;
        Eigen::VectorXd v(model.nv);
        for (int i=0;;i++)
        {
            pinocchio::forwardKinematics(model,data,q);
            pinocchio::framesForwardKinematics(model,data); 
            const pinocchio::SE3 dMi = oMdes.actInv(data.oMf[ID_endeffector]);

            err = pinocchio::log6(dMi).toVector();
            if(err.norm() < eps)
            {
            success = true;
            break;
            }
            if (i >= IT_MAX)
            {
            success = false;
            break;
            }
            pinocchio::computeFrameJacobian(model,data,q,ID_endeffector,J);
            pinocchio::Data::Matrix6 JJt;
            JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += damp;
            v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
            q = pinocchio::integrate(model,q,v*DT);
            if(!(i%10))
            std::cout << i << ": error = " << err.transpose() << std::endl;
        }

        
        if(success) 
        {
            std::cout << "Convergence achieved!" << std::endl;
        }
        else 
        {
            std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
        }
            
        std::cout << "\nresult: " << q.transpose() << std::endl;
        std::cout << "\nfinal error: " << err.transpose() << std::endl;

        std::cout << typeid(q[0]).name() << std::endl;

        targets[i] = q;
        if (!success)
        {
            IK_success_all = false;
        }
    }


    if (IK_success_all)
    {
        const moveit::core::JointModelGroup* joint_model_group = robot_arm->getCurrentState()->getJointModelGroup(group_name);
        moveit::core::RobotStatePtr current_state = robot_arm->getCurrentState(10);
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        for (std::size_t i = 0; i < targets.size(); ++i)
        {
            auto target = targets[i]; 
            joint_group_positions[i*6+0] = targets[i][0]; 
            joint_group_positions[i*6+1] = targets[i][1]; 
            joint_group_positions[i*6+2] = targets[i][2]; 
            joint_group_positions[i*6+3] = targets[i][3]; 
            joint_group_positions[i*6+4] = targets[i][4]; 
            joint_group_positions[i*6+5] = targets[i][5]; 
        }

        robot_arm->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (robot_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        robot_arm->execute(my_plan);

        result->result = "SUCCEED";
        goal_handle->succeed(result);
    }
    else
    {
        result->result = "FAIL, not found solution for inverse kinematic";
        goal_handle->abort(result);
    }

}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(composition::MoveLActionServer)
