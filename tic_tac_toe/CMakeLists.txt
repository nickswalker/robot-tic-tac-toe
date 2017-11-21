cmake_minimum_required(VERSION 2.8.3)
project(tic_tac_toe)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  bwi_msgs
  cv_bridge
  geometry_msgs
  kinova_msgs
  roscpp
  rospy
  message_generation
  pcl_ros
  pcl_conversions
  moveit_msgs
  move_base_msgs
  moveit_ros_planning_interface
  moveit_utils
  nav_msgs
  segbot_arm_manipulation
  segbot_arm_perception
  sensor_msgs
  std_msgs
  tf
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    GameState.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ExecuteGameAction.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(game_state_detector src/game_state_detector.cpp)
add_dependencies(game_state_detector tic_tac_toe_generate_messages_cpp)
target_link_libraries(game_state_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CMAKE_BUILD_TYPE MATCHES Debug)
 target_compile_definitions(game_state_detector PUBLIC DEBUG=1)
elseif(CMAKE_BUILD_TYPE MATCHES Release)
 target_compile_definitions(game_state_detector PUBLIC DEBUG=0)
endif()

add_executable(action_executor src/action_executor.cpp)
add_dependencies(action_executor tic_tac_toe_generate_messages_cpp)
target_link_libraries(action_executor ${catkin_LIBRARIES})

