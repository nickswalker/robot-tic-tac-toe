cmake_minimum_required(VERSION 2.8.3)
project(tic_tac_toe)

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

find_package(PCL 1.7 REQUIRED)


catkin_python_setup()


add_message_files(
    FILES
    GameState.msg
 )

add_service_files(
   FILES
        DetectBoard.srv
   ExecuteGameAction.srv
)
generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
        INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs segbot_arm_manipulation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)


add_executable(game_state_detector src/game_state_detector_node.cpp src/GameStateDetector.cpp)
add_dependencies(game_state_detector tic_tac_toe_generate_messages_cpp)
target_link_libraries(game_state_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(game_state_detector_cli src/game_state_detector_cli.cpp src/GameStateDetector.cpp)
add_dependencies(game_state_detector_cli tic_tac_toe_generate_messages_cpp)
target_link_libraries(game_state_detector_cli ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CMAKE_BUILD_TYPE MATCHES Debug)
 target_compile_definitions(game_state_detector PUBLIC DEBUG=1)
 target_compile_definitions(game_state_detector_cli PUBLIC DEBUG=1)
elseif(CMAKE_BUILD_TYPE MATCHES Release)
 target_compile_definitions(game_state_detector PUBLIC DEBUG=0)
 target_compile_definitions(game_state_detector_cli PUBLIC DEBUG=0)
endif()

add_executable(action_executor src/action_executor.cpp src/MicoIdleBehavior.cpp src/sound.cpp src/utils.cpp include/tic_tac_toe/utils.h)
add_dependencies(action_executor tic_tac_toe_generate_messages_cpp)
target_link_libraries(action_executor ${catkin_LIBRARIES})

add_executable(action_tester src/action_tester.cpp src/MicoIdleBehavior.cpp src/sound.cpp src/utils.cpp include/tic_tac_toe/utils.h)
add_dependencies(action_tester tic_tac_toe_generate_messages_cpp)
target_link_libraries(action_tester ${catkin_LIBRARIES})
