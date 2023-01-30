cmake_minimum_required(VERSION 3.5)
project(abb_move_group_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(abb_data REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

add_library(abb_movej_action_server SHARED
  src/abb_movej_action_server.cpp)
target_compile_definitions(abb_movej_action_server
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(abb_movej_action_server
  "moveit_ros_planning_interface"
  "abb_data"
  "moveit_visual_tools"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(abb_movej_action_server "composition::MoveJActionServer")
set(node_plugins "${node_plugins}composition::MoveJActionServer ;$<TARGET_FILE:abb_movej_action_server>\n")

add_library(abb_movel_action_server SHARED
  src/abb_movel_action_server.cpp)
target_compile_definitions(abb_movel_action_server
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(abb_movel_action_server
  "moveit_ros_planning_interface"
  "abb_data"
  "moveit_visual_tools"
  "rclcpp"
  "rclcpp_components"
  "pinocchio")
rclcpp_components_register_nodes(abb_movel_action_server "composition::MoveLActionServer")
set(node_plugins "${node_plugins}composition::MoveLActionServer ;$<TARGET_FILE:abb_movel_action_server>\n")

add_library(abb_movep_action_server SHARED
  src/abb_movep_action_server.cpp)
target_compile_definitions(abb_movep_action_server
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(abb_movep_action_server
  "moveit_ros_planning_interface"
  "abb_data"
  "moveit_visual_tools"
  "rclcpp"
  "rclcpp_components"
  "pinocchio")
rclcpp_components_register_nodes(abb_movep_action_server "composition::MovePActionServer")
set(node_plugins "${node_plugins}composition::MovePActionServer ;$<TARGET_FILE:abb_movep_action_server>\n")

add_library(abb_add_mesh_service_server SHARED
  src/abb_add_mesh_service_server.cpp)
target_compile_definitions(abb_add_mesh_service_server
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(abb_add_mesh_service_server
  "moveit_ros_planning_interface"
  "abb_data"
  "moveit_visual_tools"
  "rclcpp"
  "rclcpp_components"
  "geometric_shapes")
rclcpp_components_register_nodes(abb_add_mesh_service_server "composition::MoveLActionServer")
set(node_plugins "${node_plugins}composition::MoveLActionServer ;$<TARGET_FILE:abb_add_mesh_service_server>\n")

add_executable(abb_actions_main
    src/abb_actions_main.cpp)
target_include_directories(abb_actions_main
    PUBLIC include)
target_link_libraries(abb_actions_main
  abb_movej_action_server
  abb_movep_action_server
  abb_movel_action_server
  abb_add_mesh_service_server)    
ament_target_dependencies(abb_actions_main
    rclcpp moveit_ros_planning_interface moveit_visual_tools abb_data)       

add_executable(abb_movej_actions_client
    src/abb_movej_actions_client.cpp)
target_include_directories(abb_movej_actions_client
    PUBLIC include)
target_link_libraries(abb_movej_actions_client)    
ament_target_dependencies(abb_movej_actions_client
    rclcpp rclcpp_action abb_data)        

add_executable(abb_movel_actions_client
    src/abb_movel_actions_client.cpp)
target_include_directories(abb_movel_actions_client
    PUBLIC include)
target_link_libraries(abb_movel_actions_client)    
ament_target_dependencies(abb_movel_actions_client
    rclcpp rclcpp_action abb_data)   

add_executable(abb_movep_actions_client
    src/abb_movep_actions_client.cpp)
target_include_directories(abb_movep_actions_client
    PUBLIC include)
target_link_libraries(abb_movep_actions_client)    
ament_target_dependencies(abb_movep_actions_client
    rclcpp rclcpp_action abb_data)  

install(TARGETS abb_actions_main abb_movej_actions_client abb_movel_actions_client abb_movep_actions_client
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
