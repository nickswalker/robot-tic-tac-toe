#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tic_tac_toe/GameState.h"

#include <sstream>

#include "ros/ros.h"
#include "tic_tac_toe/ExecuteGameAction.h"

bool execute_cb(tic_tac_toe::ExecuteGameAction::Request  &req,
         tic_tac_toe::ExecuteGameAction::Response &res)
{
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("execute_game_action", execute_cb);
  ros::spin();

  return 0;
}
