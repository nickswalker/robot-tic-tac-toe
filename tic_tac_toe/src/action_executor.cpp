#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tic_tac_toe/GameState.h"
#include <sstream>
#include "ros/ros.h"
#include "tic_tac_toe/ExecuteGameAction.h"
#include <tic_tac_toe/sound.h>
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>

#include <segbot_arm_manipulation/MicoManager.h>
#include <tic_tac_toe/utils.h>
#include <tic_tac_toe/MicoIdleBehavior.h>

MicoManager *mico;
MicoIdleBehavior* idleBehavior;

//true if Ctrl-C is pressed
bool g_caught_sigint = false;

bool used_behaviors[4] = {}; // keeps track of random behaviors that have been chosen

bool run_idle_behaviors = true; // Whether to add idle behaviors, true = add idle behaviors

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
    printf("Received a new message, req.action_location = %d\n", req.action_location);

    if(req.action_location == -1){ // human has won already, or game is scratch
      idleBehavior->game_over();
      return true;
    }

    if(run_idle_behaviors){
        // choose a random behavior that has not been encountered yet
        uint32_t behavior;
        do{
          behavior = rand() % 4;
        } while(used_behaviors[behavior] == true);
        used_behaviors[behavior] = true;

       // run chosen idle behavior
       if (behavior == 0) {
           ROS_INFO("Incremental");
           idleBehavior->move_incremental(req.action_location);
       } else if (behavior == 1 ) {
           ROS_INFO("Tap");
           idleBehavior->tap_fingers();
       } else if (behavior == 2) {
           ROS_INFO("Scratch");
           idleBehavior->scratch_chin();
       } else if (behavior == 3) {
           ROS_INFO("Exaggerate");
           idleBehavior->move_exaggerated();
       }
    }
    else{
      play_file_non_blocking("thinking.wav");
      sleep(5);
    }

    // move to grid coordinate
    ROS_INFO("Moving to grid square %d.", req.action_location);
    idleBehavior->point(req.action_location);

    // move back to ready position
    geometry_msgs::PoseStamped ready_pose = get_ready_pose();
    mico->move_to_pose_moveit(ready_pose);

    ROS_INFO("Spinning, waiting for input on server");
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "action_executor");
    ros::NodeHandle n;
    ros::NodeHandle ph("~");

    ros::ServiceServer service = n.advertiseService("execute_game_action", execute_cb);

    //create subscribers for arm topics
    mico = new MicoManager(n);
    idleBehavior = new MicoIdleBehavior(mico);
    //register ctrl-c
    signal(SIGINT, sig_handler);

    mico->wait_for_data();

    bool success = ph.getParam("use_idle_behaviors", run_idle_behaviors);
    if (!success) {
        ROS_ERROR("Pass the use_idle_behaviors parameter");        
        exit(1);
    }
    ROS_INFO("use_idle_behaviors is %s", run_idle_behaviors ? "true" : "false");
    // close hand and move to ready position
    ROS_INFO("Closing hand and moving to ready position.");
    mico->move_fingers(5500, 5500);
    geometry_msgs::PoseStamped ready_pose = get_ready_pose();
    mico->move_to_pose_moveit(ready_pose);

    // set random seed
    srand(time(0));

    // never returns
    ROS_INFO("Spinning, waiting for input on server");
    ros::spin();

    // shutdown
    ros::shutdown();

    return 0;
}
