#include "ros/ros.h"
#include <tic_tac_toe/GameState.h>
#include <tic_tac_toe/ExecuteGameAction.h>
#include <tic_tac_toe/MicoIdleBehavior.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <segbot_arm_manipulation/MicoManager.h>
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


geometry_msgs::PoseStamped point(uint8_t grid_square)
{
    geometry_msgs::PoseStamped p_target;

    p_target.header.frame_id = "m1n6s200_link_base";

    switch(grid_square){
        case(0): {
            p_target.pose.position.x = 0.426824212074;
            p_target.pose.position.y = 0.284858942032;
            p_target.pose.position.z = 0.0402360036969;
            p_target.pose.orientation.x = -0.0495440475643;
            p_target.pose.orientation.y = -0.9955971241;
            p_target.pose.orientation.z = -0.0676593929529;
            p_target.pose.orientation.w = -0.0418801791966;
            break;
        }
        case(1): {
            p_target.pose.position.x = 0.427185595036;
            p_target.pose.position.y = 0.179492980242;
            p_target.pose.position.z = 0.0362501107156;
            p_target.pose.orientation.x = -0.0409061983228;
            p_target.pose.orientation.y = -0.998338639736;
            p_target.pose.orientation.z = -0.00312207173556;
            p_target.pose.orientation.w = 0.0404587648809;
            break;
        }
        case(2): {
            p_target.pose.position.x = 0.431104779243;
            p_target.pose.position.y = 0.0673867613077;
            p_target.pose.position.z = 0.022669646889;
            p_target.pose.orientation.x = -0.0227509662509;
            p_target.pose.orientation.y = -0.997758924961;
            p_target.pose.orientation.z = -0.0346958227456;
            p_target.pose.orientation.w = 0.0524955913424;
            break;
        }
        case(3): {
            p_target.pose.position.x = 0.323083162308;
            p_target.pose.position.y = 0.290391534567;
            p_target.pose.position.z = 0.0327621549368;
            p_target.pose.orientation.x = -0.0466325692832;
            p_target.pose.orientation.y = -0.998385667801;
            p_target.pose.orientation.z = -0.0323525331914;
            p_target.pose.orientation.w = 0.00221162941307;
            break;
        }
        case(4): {
            p_target.pose.position.x = 0.314559876919;
            p_target.pose.position.y = 0.182191491127;
            p_target.pose.position.z = 0.0249998774379;
            p_target.pose.orientation.x = -0.0345241464674;
            p_target.pose.orientation.y = -0.999021053314;
            p_target.pose.orientation.z = -0.0276549495757;
            p_target.pose.orientation.w = -0.000544465205166;
            break;
        }
        case(5): {
            p_target.pose.position.x = 0.304246366024;
            p_target.pose.position.y = 0.0765624493361;
            p_target.pose.position.z = 0.0174890849739;
            p_target.pose.orientation.x = -0.0159492921084;
            p_target.pose.orientation.y = -0.999459207058;
            p_target.pose.orientation.z = -0.00590369151905;
            p_target.pose.orientation.w = -0.0281426385045;
            break;
        }
        case(6): {
            p_target.pose.position.x = 0.207779467106;
            p_target.pose.position.y = 0.295100569725;
            p_target.pose.position.z = 0.0326444283128;
            p_target.pose.orientation.x = -0.0451364554465;
            p_target.pose.orientation.y = -0.997986733913;
            p_target.pose.orientation.z = -0.0272142477334;
            p_target.pose.orientation.w = -0.0352782607079;
            break;
        }
        case(7): {
            p_target.pose.position.x = 0.195141017437;
            p_target.pose.position.y = 0.187146872282;
            p_target.pose.position.z = 0.0303510185331;
            p_target.pose.orientation.x = -0.0113975815475;
            p_target.pose.orientation.y = -0.999932646751;
            p_target.pose.orientation.z = -0.00210533523932;
            p_target.pose.orientation.w = 0.000616318488028;
            break;
        }
        case(8): {
            p_target.pose.position.x = 0.201366573572;
            p_target.pose.position.y = 0.0808693170547;
            p_target.pose.position.z = 0.0229740720242;
            p_target.pose.orientation.x = -0.0264183524996;
            p_target.pose.orientation.y = -0.997659921646;
            p_target.pose.orientation.z = -0.0139531157911;
            p_target.pose.orientation.w = 0.0614984557033;
            break;
        }
    }

    return p_target;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "action_executor");
    ros::NodeHandle n;
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

    srand(time(0));
    while (ros::ok()) {
        uint8_t behavior = rand() % 4;
        uint8_t dest = rand() % 10;
        // run the idle bahavior based on the current iteration
        if (behavior == 0) {
            ROS_INFO("Incremental");
            idleBehavior.move_incremental(dest);
        } else if (behavior == 1 ) {
            ROS_INFO("Tap");
            idleBehavior.tap_fingers();
        } else if (behavior == 2) {
            ROS_INFO("Scratch");
            idleBehavior.scratch_chin();
        } else if (behavior == 3) {
            ROS_INFO("Exaggerate");
            idleBehavior.move_exaggerated();
        }

        // move to grid coordinate
        pressEnter("Press [Enter] to move to next grid coordinate");
        ROS_INFO("Moving to grid square %d", dest);
        idleBehavior.point(dest);

        // move back home
        pressEnter("Press [Enter] to move arm back to ready");

        mico->move_to_pose_moveit(ready_pose);
    }

    pressEnter("Press [Enter] to go home");
    mico->move_home();

    // shutdown
    ros::shutdown();

    return 0;
}
