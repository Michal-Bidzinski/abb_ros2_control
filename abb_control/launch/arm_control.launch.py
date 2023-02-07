import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    param_file = LaunchConfiguration("param_file")

    # ROBOT DESCRIPTION
    params = load_yaml(
        runtime_config_package.perform(context), "param/" + param_file.perform(context)
    )

    arm_params = params['robots']

    num_of_robots = len(arm_params)
    print("Num of robots: ", num_of_robots)

    # ROBOT DESCRIPTION
    robot_description_args = ''
    for i, key in enumerate(arm_params):
        if num_of_robots > 1:
            num = "_" + str(i+1)
        else:
            num = ""
        robot_description_args = "".join(
            [robot_description_args,
            " ",
            "prefix" + num + ":=",
            key,
            " ",
            "use_fake_hardware" + num + ":=",
            arm_params[key]['robot_description']['use_fake_hardware'],
            " ",
            "fake_sensor_commands" + num + ":=",
            arm_params[key]['robot_description']['fake_sensor_commands'],
            " ",
            "rws_ip" + num + ":=",
            arm_params[key]['robot_description']['rws_ip'],
            " ",
            "rws_port" + num + ":=",
            arm_params[key]['robot_description']['rws_port'],
            " ",
            "egm_port" + num + ":=",
            arm_params[key]['robot_description']['egm_port'],
            " ",   
            "x_trans" + num + ":=",
            arm_params[key]['robot_description']['x_trans'],
            " ",   
            "y_trans" + num + ":=",
            arm_params[key]['robot_description']['y_trans'],
            " ",
            "z_trans" + num + ":=",
            arm_params[key]['robot_description']['z_trans'],
            " ",
            "x_rot" + num + ":=",
            arm_params[key]['robot_description']['x_rot'],
            " ",
            "y_rot" + num + ":=",
            arm_params[key]['robot_description']['y_rot'],
            " ",
            "z_rot" + num + ":=",
            arm_params[key]['robot_description']['z_rot'],
            " ",
            "velocity" + num + ":=",
            params['velocity']['value'],
            " ",]
        )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(runtime_config_package.perform(context)),
                 "config", 
                 params['urdf']['description_file']]
            ),
            robot_description_args    
        ]
    )

    robot_description = {"robot_description": robot_description_content}


    # ROBOT DESCRIPTION SEMANTIC
    robot_description_semantic_args = ''
    for i, key in enumerate(arm_params):
        if num_of_robots > 1:
            num = "_" + str(i+1)
        else:
            num = ""
        robot_description_semantic_args = "".join(
            [robot_description_semantic_args,
            " ",
            "prefix" + num + ":=",
            key,
            " "]
        )

    robot_description_semantic_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(runtime_config_package.perform(context)),
                 "config",
                 params['srdf']['srdf_file']]
            ),
            robot_description_semantic_args
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config.perform(
            context
        )
    }


    #KINEMATICS
    kinematics_yaml = load_yaml(
        runtime_config_package.perform(context), 
        "config/" + params['config_folder']['name'] + "/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}


    # Planning Functionality
    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # ompl_planning_yaml = load_yaml(
    #     runtime_config_package.perform(context),
    #     "config/" + params['planning_pipeline']['planning_config']
    # )
    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    planning_pipeline_config= {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "planning_adapters": """default_planner_request_adapters/FixWorkspaceBounds
                                default_planner_request_adapters/FixStartStateBounds
                                default_planner_request_adapters/FixStartStateCollision
                                default_planner_request_adapters/FixStartStatePathConstraints""",
            "default_planner_config": "PTP",
            "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction
                            pilz_industrial_motion_planner/MoveGroupSequenceService""",
        },
        'robot_description_planning': {}
    }

    joint_limits = load_yaml(
        runtime_config_package.perform(context),
        "config/" + params['config_folder']['name'] + "/" + params['moveit']['joint_limits']
        )
    
    cartesian_limits = load_yaml(
        runtime_config_package.perform(context),
        "config/cartesian_limits.yaml"
        )
    
    print(type(cartesian_limits))

    planning_pipeline_config['robot_description_planning'].update(cartesian_limits)
    planning_pipeline_config['robot_description_planning'].update(joint_limits)

    # Trajectory Execution Functionality
    moveit_controllers_yaml = load_yaml(
        runtime_config_package.perform(context),
        "config/" + params['config_folder']['name'] + "/" + params['moveit']['controllers']
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # robot_description_planning,
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory(runtime_config_package.perform(context)),
        "rviz",
        params['rviz']['config']
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(params['rviz']['launch_rviz']),
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "r_base"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory(runtime_config_package.perform(context)),
        "config",
        params['config_folder']['name'],
        params['controller_manager']['ros_controllers'],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    ros2_controllers_yaml = load_yaml(
        runtime_config_package.perform(context),
        "config/" + params['config_folder']['name'] + "/" + params['controller_manager']['ros_controllers']
    )

    controllers = []
    controllers_skip_list = ['update_rate']
    for key, value in ros2_controllers_yaml['controller_manager']['ros__parameters'].items():
        if key not in controllers_skip_list:
            controllers.append(key)

    # Load controllers
    load_controllers = []
    for controller in controllers:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        )

    # Move group interface - move action servers
    move_group_interface = Node(
        name="abb_actions_main",
        package="abb_move_group_interface",
        executable="abb_actions_main",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"prefixes_list": params['move_group']['robots_list']}],
    )

    nodes_to_start = [
                      rviz_node,
                      static_tf,
                      robot_state_publisher,
                      run_move_group_node, 
                      ros2_control_node,
                      move_group_interface
                      ] + load_controllers
    
    return nodes_to_start    

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="abb_irb_120_dual_config",
            description='Package with the configuration and description files',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "param_file",
            default_value="irb_120_dual_real.yaml",
            description='File with the all parameters',
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
