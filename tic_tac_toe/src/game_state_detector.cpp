#include "ros/ros.h"
#include "tic_tac_toe/GameState.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "game_state_detector");

  ros::NodeHandle n;
  ros::Publisher state_pub = n.advertise<tic_tac_toe::GameState>("game_state", 1000);

  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    tic_tac_toe::GameState msg;

    state_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
