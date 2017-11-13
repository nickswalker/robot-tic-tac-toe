#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tic_tac_toe/GameState.h"
#include <sstream>
#include "ros/ros.h"
#include "tic_tac_toe/ExecuteGameAction.h"
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/FingerPosition.h>


//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>

#define NUM_JOINTS 8 //6+2 for the arm


//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
kinova_msgs::FingerPosition current_finger;


bool heardJoinstState;
bool heardPose;
bool heardFingers;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};

//Joint positions cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& msg) {
    
    if (msg->position.size() == NUM_JOINTS){
        current_state = *msg;
        heardJoinstState = true;
    }
}

//tool pose cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
    current_pose = msg;
    heardPose = true;
}

//fingers state cb
void fingers_cb (const kinova_msgs::FingerPositionConstPtr& msg) {
    current_finger = *msg;
    heardFingers = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
    
    heardJoinstState = false;
    heardPose = false;
    heardFingers = false;
    
    ros::Rate r(40.0);
    
    while (ros::ok()){
        ros::spinOnce();    
        
        if (heardJoinstState && heardPose && heardFingers)
            return;
        
        r.sleep();
    }
}


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


bool execute_cb(tic_tac_toe::ExecuteGameAction::Request  &req,
         tic_tac_toe::ExecuteGameAction::Response &res)
{
  return true;
}


geometry_msgs::PoseStamped point(int8_t grid_square)
{
  geometry_msgs::PoseStamped p_target;
 
  p_target.header.frame_id = "m1n6s200_link_base";
  p_target.pose.position.z = 0.02;

  switch(grid_square){
    case(0): {
      p_target.pose.position.x = 0.429870784283;
      p_target.pose.position.y = 0.277002158165;
      p_target.pose.orientation.x = -0.735577583313;
      p_target.pose.orientation.y = -0.660595536232;
      p_target.pose.orientation.z = -0.142605945468;
      p_target.pose.orientation.w = -0.0469318404794;
      break;
    }
    case(1): {
      p_target.pose.position.x = 0.416978001595; 
      p_target.pose.position.y = 0.188035085797;
      p_target.pose.orientation.x = -0.712904572487; 
      p_target.pose.orientation.y = -0.695405781269;
      p_target.pose.orientation.z = -0.0817181617022;
      p_target.pose.orientation.w = -0.0387297943234;
      break;
    }
    case(2): {
      p_target.pose.position.x = 0.41749227047;
      p_target.pose.position.y = 0.0784174501896;
      p_target.pose.orientation.x = -0.73127913475;
      p_target.pose.orientation.y = -0.678451240063;
      p_target.pose.orientation.z = -0.0684093385935;
      p_target.pose.orientation.w = -0.0159641727805;   
      break;
    }
    case(3): {
      p_target.pose.position.x = 0.319933205843;
      p_target.pose.position.y = 0.298095226288;
      p_target.pose.orientation.x = -0.728000819683;
      p_target.pose.orientation.y = -0.683962523937;
      p_target.pose.orientation.z = -0.0343066714704;
      p_target.pose.orientation.w = 0.0321400351822;
      break;
    }
    case(4): {
      p_target.pose.position.x = 0.32172358036;
      p_target.pose.position.y = 0.192896842957;
      p_target.pose.orientation.x = -0.739264786243;
      p_target.pose.orientation.y = -0.670717418194;
      p_target.pose.orientation.z = -0.0416212826967;
      p_target.pose.orientation.w = 0.0435139834881;
      break;
    }
    case(5): {
      p_target.pose.position.x = 0.320281594992;
      p_target.pose.position.y = 0.0820605158806;
      p_target.pose.orientation.x = -0.734941363335;
      p_target.pose.orientation.y = -0.676072537899;
      p_target.pose.orientation.z = -0.0153863821179;
      p_target.pose.orientation.w = 0.0505022183061;
      break;
    }
    case(6): {
      p_target.pose.position.x = 0.193733692169;
      p_target.pose.position.y =  0.288643151522;
      p_target.pose.orientation.x = -0.0211249664426;
      p_target.pose.orientation.y = -0.988549888134;
      p_target.pose.orientation.z = -0.112252391875;
      p_target.pose.orientation.w = 0.0986009389162;
      break;
    }
    case(7): {
      p_target.pose.position.x = 0.22457306087;
      p_target.pose.position.y = 0.18780374527;
      p_target.pose.orientation.x = -0.734695315361;
      p_target.pose.orientation.y = -0.666424036026;
      p_target.pose.orientation.z = -0.0120352953672;
      p_target.pose.orientation.w = 0.126320615411;
      break;
    }
    case(8): {
      p_target.pose.position.x = 0.214110553265;
      p_target.pose.position.y = 0.0770715326071;
      p_target.pose.orientation.x = -0.712083637714;
      p_target.pose.orientation.y = -0.686031162739;
      p_target.pose.orientation.z = 0.0380877107382;
      p_target.pose.orientation.w = 0.144386216998;
      break;
    }
  }
   
  return p_target;
}

void scratch_chin(ros::NodeHandle n)
{
  pressEnter("Press [Enter] to scratch chin");
  
  // set coordinates for arm
  geometry_msgs::PoseStamped p_target;
  p_target.header.frame_id = "m1n6s200_link_base";
  p_target.pose.position.x = -0.236974254251;
  p_target.pose.position.y = 0.006457015872;
  p_target.pose.position.z = 0.640369296074;
  p_target.pose.orientation.x = -0.645233333111;
  p_target.pose.orientation.y = 0.0155444145203;
  p_target.pose.orientation.z = 0.638653099537;
  p_target.pose.orientation.w = 0.418992251158;

  // move arm to head
  segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);

  // scratch chin
  for(int i = 0; i < 3; i++){
    segbot_arm_manipulation::moveFingers(100,100);
    segbot_arm_manipulation::moveFingers(5500,5500);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("execute_game_action", execute_cb);
  
  //create subscribers for arm topics
  
  //joint positions
  ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
  
  //cartesean tool position and orientation
  ros::Subscriber sub_tool = n.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);

  //finger positions
  ros::Subscriber sub_finger = n.subscribe("/m1n6s200_driver/out/finger_position", 1, fingers_cb);
   
  //register ctrl-c
  signal(SIGINT, sig_handler);
      
  listenForArmData();

  // close hand and move home
  pressEnter("Press [Enter] to close the hand and move home."); 
  segbot_arm_manipulation::moveFingers(5500,5500);   // close the hand
  segbot_arm_manipulation::homeArm(n);               // move to home
 
  // scratch chin  
  scratch_chin(n);
 
  // iterate through all the 9 grid squares.
  for(int i = 0; i < 9; i++){
    pressEnter("Press [Enter] to move to next grid coordinate");
    geometry_msgs::PoseStamped p_target = point(i);
    ROS_INFO("Moving to target x=%f, y=%f, z=%f", p_target.pose.position.x, p_target.pose.position.y, p_target.pose.position.z);
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
  }

  //home arm using service call to arm driver
  pressEnter("Press [Enter] to go home");
  segbot_arm_manipulation::homeArm(n);

  // shutdown
  ros::shutdown();

  return 0;
}
