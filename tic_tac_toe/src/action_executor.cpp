#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tic_tac_toe/GameState.h"
#include <sstream>
#include "ros/ros.h"
#include "tic_tac_toe/ExecuteGameAction.h"
#include "tic_tac_toe/MicoIdleBehavior.h"
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>

#include <segbot_arm_manipulation/MicoManager.h>
#include <tic_tac_toe/utils.h>


//true if Ctrl-C is pressed
bool g_caught_sigint = false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};


// Blocking call for user input
void pressEnter(std::string message) {
    std::cout << message;
    while (true) {
        char c = std::cin.get();
        if (c == '\n')
            break;
        else if (c == 'q') {
            ros::shutdown();
            exit(1);
        } else {
            std::cout << message;
        }
    }
}


bool execute_cb(tic_tac_toe::ExecuteGameAction::Request &req,
                tic_tac_toe::ExecuteGameAction::Response &res) {
    printf("output = %d\n", req.action_location);
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "action_executor");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("execute_game_action", execute_cb);

    //create subscribers for arm topics
    MicoManager *mico = new MicoManager(n);
    MicoIdleBehavior idleBehavior(n, mico);
    //register ctrl-c
    signal(SIGINT, sig_handler);

    mico->wait_for_data();

    geometry_msgs::PoseStamped ready_pose = get_ready_pose();
    // close hand and move home
    pressEnter("Press [Enter] to close the hand and move home.");
    mico->close_hand();
    mico->move_to_pose_moveit(ready_pose);

    // shutdown
    ros::shutdown();

    return 0;
}
