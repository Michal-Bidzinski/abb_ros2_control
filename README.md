## ABB_ros2_control

ROS2 repository to control ABB robots with MoveIt!2.


## Requirements

* ROS2 Humble

## Installation

1. Clone repositories:
   ```sh
   cd src
   git clone https://github.com/Michal-Bidzinski/abb_ros2.git -b humble
   git clone https://github.com/FlexBE/flexbe_app.git
   ```
2. Build:
   ```sh
   cd src
   vcs import < abb_ros2/abb.repos
   cd ..
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On -Wall -Wextra -Wpedantic
   . install.setup.bash
   ```
   
## Usage
To run full stack:
    ```sh
   ros2 launch abb_control arm_control.launch.py
   ```
This roslaunch takes two parameters: *runtime_config_package* and *param_file*. 
First parameter is package name, which contain all necessary config file. This repository contain four packages with configs:
* abb_irb_120_config
* abb_irb_120_dual_config
* abb_irb_4600_config (under development)
* abb_irb_4600_dual_config (under development)

Second, *param_file* is a name of files with all necessary parameters for robots. 
In each packages are minimum two files with parameters to run simulation or to connect with real robot.
If you want, you can change this parameters.

Package "abb_move_group_interface" contain *action-servers* and examples "action-clients" to set control the arm.
The following actions are available:
* MoveJ - joint movement
* MoveL - linear movement
* MoveP - joint movement to point

If you run simulation you will see that all names contain namespace depend of name of robot's prefix, for example for two robots:
```sh
/r1_abb_arm/MoveJ
/r1_abb_arm/MoveL
/r1_abb_arm/MoveP
/r2_abb_arm/MoveJ
/r2_abb_arm/MoveL
/r2_abb_arm/MoveP
/dual_abb_arm/MoveJ
/dual_abb_arm/MoveL
/dual_abb_arm/MoveP
```
