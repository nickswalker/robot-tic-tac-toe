#include "ros/ros.h"
#include <tic_tac_toe/GameState.h>
#include <tic_tac_toe/ExecuteGameAction.h>
#include <tic_tac_toe/MicoIdleBehavior.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <segbot_arm_manipulation/Mico.h>
#include <signal.h>
#include <tic_tac_toe/utils.h>


//true if Ctrl-C is pressed
bool g_caught_sigint=false;


/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};


// Blocking call for user input
void pressEnter(std::string message){
    std::cout << message;
    while (true){
        char c = std::cin.get();
        if (c == '\n')
            break;
        else if (c == 'q'){
            ros::shutdown();
            exit(1);
        }
        else {
            std::cout <<  message;
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "action_executor");
    ros::NodeHandle n;
    //create subscribers for arm topics
    auto *mico = new segbot_arm_manipulation::Mico(n);
    MicoIdleBehavior idleBehavior(mico);
    //register ctrl-c
    signal(SIGINT, sig_handler);

    mico->wait_for_data();

    // close hand and move home
    pressEnter("Press [Enter] to close hand and move to ready position.");
    mico->move_fingers(5500, 5500); 
    geometry_msgs::PoseStamped ready_pose = get_ready_pose();
    mico->move_to_pose_moveit(ready_pose);
    int target_duration = 40;

    srand(time(0));
    while (ros::ok()) {
        pressEnter("Press [Enter] to make next move.");

        uint32_t behavior = rand() % 4;
        uint32_t dest = rand() % 9;

        // run the idle bahavior based on the current iteration
        if (behavior == 0) {
            ROS_INFO("Incremental");
            idleBehavior.move_incremental(dest, target_duration);
        } else if (behavior == 1 ) {
            ROS_INFO("Tap");
            idleBehavior.tap_fingers(target_duration);
        } else if (behavior == 2) {
            ROS_INFO("Scratch");
            idleBehavior.scratch_chin(target_duration);
        } else if (behavior == 3) {
            ROS_INFO("Exaggerate");
            idleBehavior.move_exaggerated(target_duration);
        }

        // move to grid coordinate
        ROS_INFO("Moving to grid square %d.", dest);
        idleBehavior.point(dest, 10);

        // move back home
        pressEnter("Press [Enter] to move arm back to ready position.");
        mico->move_to_pose_moveit(ready_pose);
    }

    pressEnter("Press [Enter] to go home");
    mico->move_home();

    // shutdown
    ros::shutdown();

    return 0;
}
