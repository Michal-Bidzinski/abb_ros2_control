import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="abb_actions_main",
        package="abb_move_group_interface",
        executable="abb_actions_main",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"prefixes_list": params['move_group']['robots_list']}],
    )
    
    return [move_group_demo]  

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
            default_value="irb_120_dual_sim.yaml",
            description='File with the all parameters',
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
