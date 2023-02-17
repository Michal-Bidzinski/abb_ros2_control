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
from launch.conditions import IfCondition, UnlessCondition


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


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    print("script_filename: ", script_filename.perform(context))
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    print("Num of robots: ", num_of_robots)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur3_dual_config"), "config", "arm.urdf.xacro"]),
            " ",
            "robot_ip_1:=",
            '150.254.47.148',
            " ",
            # "joint_limit_params:=",
            # joint_limit_params,
            # " ",
            # "kinematics_params:=",
            # kinematics_params,
            # " ",
            # "physical_params:=",
            # physical_params,
            # " ",
            # "visual_params:=",
            # visual_params,
            # " ",
            "safety_limits_1:=",
            'true',
            " ",
            "safety_pos_margin_1:=",
            '0.15',
            " ",
            "safety_k_position_1:=",
            '20',
            " ",
            "name_1:=",
            'ur_1',
            " ",
            "script_filename_1:=",
            script_filename.perform(context),
            " ",
            "input_recipe_filename_1:=",
            input_recipe_filename.perform(context),
            " ",
            "output_recipe_filename_1:=",
            output_recipe_filename.perform(context),
            " ",
            "prefix_1:=",
            'r1_',
            " ",
            "use_fake_hardware_1:=",
            'false',
            " ",
            "fake_sensor_commands_1:=",
            "false",
            " ",
            "headless_mode_1:=",
            "true",
            " ",
            "use_tool_communication_1:=",
            "false",
            " ",
            "tool_parity_1:=",
            "0",
            " ",
            "tool_baud_rate_1:=",
            "115200",
            " ",
            "tool_stop_bits_1:=",
            "1",
            " ",
            "tool_rx_idle_chars_1:=",
            "1.5",
            " ",
            "tool_tx_idle_chars_1:=",
            "3.5",
            " ",
            "tool_device_name_1:=",
            "/tmp/ttyUR",
            " ",
            "tool_tcp_port_1:=",
            "54321",
            " ",
            "tool_voltage_1:=",
            "0",
            " ",
            "reverse_ip_1:=",
            "0.0.0.0",
            " ",
            "script_command_port_1:=",
            "56004",
            " ",
            "x_pose_1:=",
            "0.0",
            " ",
            "y_pose_1:=",
            "0.0",
            " ",
            "z_pose_1:=",
            "0.0",
            " ",
            "reverse_port_1:=",
            "56001",
            " ",
            "script_sender_port_1:=",
            "56002",
            " ",
            "trajectory_port_1:=",
            "56003",
            " ",
            " ",
            "robot_ip_2:=",
            '150.254.47.148',
            " ",
            # "joint_limit_params:=",
            # joint_limit_params,
            # " ",
            # "kinematics_params:=",
            # kinematics_params,
            # " ",
            # "physical_params:=",
            # physical_params,
            # " ",
            # "visual_params:=",
            # visual_params,
            # " ",
            "safety_limits_2:=",
            'true',
            " ",
            "safety_pos_margin_2:=",
            '0.15',
            " ",
            "safety_k_position_2:=",
            '20',
            " ",
            "name_2:=",
            'ur_2',
            " ",
            "script_filename_2:=",
            script_filename.perform(context),
            " ",
            "input_recipe_filename_2:=",
            input_recipe_filename.perform(context),
            " ",
            "output_recipe_filename_2:=",
            output_recipe_filename.perform(context),
            " ",
            "prefix_2:=",
            'r2_',
            " ",
            "use_fake_hardware_2:=",
            'false',
            " ",
            "fake_sensor_commands_2:=",
            "false",
            " ",
            "headless_mode_2:=",
            "true",
            " ",
            "use_tool_communication_2:=",
            "false",
            " ",
            "tool_parity_2:=",
            "0",
            " ",
            "tool_baud_rate_2:=",
            "115200",
            " ",
            "tool_stop_bits_2:=",
            "1",
            " ",
            "tool_rx_idle_chars_2:=",
            "1.5",
            " ",
            "tool_tx_idle_chars_2:=",
            "3.5",
            " ",
            "tool_device_name_2:=",
            "/tmp/ttyUR",
            " ",
            "tool_tcp_port_2:=",
            "54321",
            " ",
            "tool_voltage_2:=",
            "0",
            " ",
            "reverse_ip_2:=",
            "0.0.0.0",
            " ",
            "script_command_port_2:=",
            "56004",
            " ",
            "x_pose_2:=",
            "0.0",
            " ",
            "y_pose_2:=",
            "0.0",
            " ",
            "z_pose_2:=",
            "0.0",
            " ",
            "reverse_port_2:=",
            "56001",
            " ",
            "script_sender_port_2:=",
            "56002",
            " ",
            "trajectory_port_2:=",
            "56003",
            " ",
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

    # ros2_controllers_yaml = load_yaml(
    #     runtime_config_package.perform(context),
    #     "config/" + params['config_folder']['name'] + "/" + params['controller_manager']['ros_controllers']
    # )

    # controllers = []
    # controllers_skip_list = ['update_rate']
    # for key, value in ros2_controllers_yaml['controller_manager']['ros__parameters'].items():
    #     if key not in controllers_skip_list:
    #         controllers.append(key)

    # # Load controllers
    # load_controllers = []
    # for controller in controllers:
    #     load_controllers.append(
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=[controller, "-c", "/controller_manager"],
    #         )
    #     )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r1_arm_controller", "-c", "/controller_manager"],
    )

    

    # Move group interface - move action servers
    move_group_interface = Node(
        name="abb_actions_main",
        package="abb_move_group_interface",
        executable="abb_actions_main",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"prefixes_list": params['move_group']['robots_list']}],
    )

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "config",
            "ur3" + "_update_rate.yaml",
        ]
    )

    # ros2_controllers_yaml = load_yaml(
    #     runtime_config_package.perform(context),
    #     "config/" + params['config_folder']['name'] + "/" + params['controller_manager']['ros_controllers']
    # )

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package.perform(context)), "config", params['config_folder']['name'] + "/" + params['controller_manager']['ros_controllers']]
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[robot_description, update_rate_config_file, initial_joint_controllers],
        output="screen",
    )

    # r2_ur_control_node = Node(
    #     package="ur_robot_driver",
    #     executable="ur_ros2_control_node",
    #     parameters=[robot_description, update_rate_config_file, initial_joint_controllers],
    #     output="screen",
    # )
    # r1_dashboard_client_node = Node(
    #     package="ur_robot_driver",
    #     condition=UnlessCondition('true'),
    #     executable="dashboard_client",
    #     name="dashboard_client",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[{"robot_ip": '150.254.47.148'}],
    # )
    # r2_dashboard_client_node = Node(
    #     package="ur_robot_driver",
    #     condition=UnlessCondition(arm_params["r2_"]['robot_description']['use_fake_hardware']),
    #     executable="dashboard_client",
    #     name="dashboard_client",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[{"robot_ip": arm_params["r2_"]['robot_description']['robot_ip']}],
    # )

    nodes_to_start = [
                      rviz_node,
                      static_tf,
                      robot_state_publisher,
                      run_move_group_node, 
                      ros2_control_node,
                      # move_group_interface
                      # ur_control_node,
                    #   r1_dashboard_client_node,
                    #   r2_dashboard_client_node,
                    #   r2_ur_control_node,
                      ] 
    
    return nodes_to_start    

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur3_dual_config",
            description='Package with the configuration and description files',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "param_file",
            default_value="irb_120_dual_sim.yaml",
            description='File with the all parameters',
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
