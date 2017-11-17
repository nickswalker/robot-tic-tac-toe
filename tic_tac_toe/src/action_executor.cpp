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
#include <time.h>


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

void move_home(ros::NodeHandle n){
  geometry_msgs::PoseStamped p_target;
 
  p_target.header.frame_id = "m1n6s200_link_base";
  p_target.pose.position.x = 0.295476645231;
  p_target.pose.position.y = -0.126079112291;
  p_target.pose.position.z = 0.0612120516598;
  p_target.pose.orientation.x = 0.0154021102935;
  p_target.pose.orientation.y = -0.999735891819;
  p_target.pose.orientation.z = 0.00489564239979;
  p_target.pose.orientation.w = -0.0163363721222;
  segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
}


geometry_msgs::PoseStamped point(int8_t grid_square)
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

void scratch_chin(ros::NodeHandle n)
{
  pressEnter("Press [Enter] to scratch chin");
  
  // set coordinates for arm
  geometry_msgs::PoseStamped p_target;
  p_target.header.frame_id = "m1n6s200_link_base";
  p_target.pose.position.x = -0.169198065996;
  p_target.pose.position.y = 0.193875789642;
  p_target.pose.position.z = 0.601638615131;
  p_target.pose.orientation.x = 0.191084221005;
  p_target.pose.orientation.y = -0.631144821644;
  p_target.pose.orientation.z = 0.527726113796;
  p_target.pose.orientation.w = 0.535395443439;

  // move arm to head
  segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);

  // scratch chin
  for(int i = 0; i < 3; i++){
    segbot_arm_manipulation::moveFingers(100,100);
    segbot_arm_manipulation::moveFingers(5500,5500);
  }
}


void move_incremental(ros::NodeHandle n, int dest)
{
  pressEnter("Press [Enter] to move incremental");
  
  // set coordinates for arm
  geometry_msgs::PoseStamped p_target;
  p_target.header.frame_id = "m1n6s200_link_base";
  p_target.pose.position.x = 0.311155617237;
  p_target.pose.position.y = 0.189424604177;
  p_target.pose.position.z = 0.213762119412;
  p_target.pose.orientation.x = 0.665959119797;
  p_target.pose.orientation.y = -0.745930075645;
  p_target.pose.orientation.z = 0.00926198810339;
  p_target.pose.orientation.w = 0.0010175104253;

  // move arm to incremental start position
  segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);

  switch(dest){
    // first quadrant
    case 0:
    case 1:
    case 3:
      p_target.pose.position.x = 0.359608054161;
      p_target.pose.position.y = 0.243571102619;
      p_target.pose.position.z = 0.0770178735256;
      p_target.pose.orientation.x = -0.996901810169;
      p_target.pose.orientation.y = 0.0625432953238;
      p_target.pose.orientation.z = -0.0410128049552;
      p_target.pose.orientation.w = 0.0243524741381;
      break;

    // second quadrant
    case 2:
    case 5:
      p_target.pose.position.x = 0.354002892971;
      p_target.pose.position.y = 0.143095493317;
      p_target.pose.position.z = 0.0753956288099;
      p_target.pose.orientation.x = -0.998840987682;
      p_target.pose.orientation.y = 0.0366943664849;
      p_target.pose.orientation.z = -0.0288470163941;
      p_target.pose.orientation.w = 0.0117430668324;
      break;

    // third quadrant
    case 4:
    case 6:
      p_target.pose.position.x = 0.354002892971;
      p_target.pose.position.y = 0.143095493317;
      p_target.pose.position.z = 0.0753956288099;
      p_target.pose.orientation.x = -0.998840987682;
      p_target.pose.orientation.y = 0.0366943664849;
      p_target.pose.orientation.z = -0.0288470163941;
      p_target.pose.orientation.w = 0.0117430668324;
      break;

    // fourth quadrant
    case 7:
    case 8:
      p_target.pose.position.x = 0.259484708309;
      p_target.pose.position.y = 0.143753558397;
      p_target.pose.position.z = 0.0591824762523;
      p_target.pose.orientation.x = 0.995885074139;
      p_target.pose.orientation.y = -0.0568450242281;
      p_target.pose.orientation.z = 0.0699009969831;
      p_target.pose.orientation.w = 0.00976687949151;
      break;
  }
  // move to new incremental position based off of destination
  segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);

  ros::Publisher pub_angular_velocity = n.advertise<kinova_msgs::JointAngles>("/m1n6s200_driver/in/joint_velocity", 10);
  kinova_msgs::JointAngles msg;
  for(int i = 0; i < 2; i++){
  msg.joint1 = 0.0;
  msg.joint2 = 0.0;
  msg.joint3 = 0.0;
  msg.joint4 = 0.0;
  msg.joint5 = 45;
  msg.joint6 = 0.0; 

  double duration = 0.75;  // 0.75 seconds
  double elapsed_time = 0.0;
	
  double pub_rate = 100.0;
  ros::Rate r(pub_rate);
	
  while (ros::ok()){
    //collect messages
    ros::spinOnce();
	
    //publish velocity message
    pub_angular_velocity.publish(msg);
		
    r.sleep();
		
    elapsed_time += (1.0/pub_rate);
    if (elapsed_time > duration)
      break;
  }

  msg.joint5 = -45;

  duration = 1.5;  // 1.5 seconds
  elapsed_time = 0.0;
	
  while (ros::ok()){
    //collect messages
    ros::spinOnce();
	
    //publish velocity message
    pub_angular_velocity.publish(msg);
		
    r.sleep();
		
    elapsed_time += (1.0/pub_rate);
    if (elapsed_time > duration)
      break;
  }

  msg.joint5 = 45;

  duration = 0.75;  // 0.75 seconds
  elapsed_time = 0.0;
	
  while (ros::ok()){
    //collect messages
    ros::spinOnce();
	
    //publish velocity message
    pub_angular_velocity.publish(msg);
		
    r.sleep();
		
    elapsed_time += (1.0/pub_rate);
    if (elapsed_time > duration)
      break;
  }
  }

	
  //publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
  msg.joint5 = 0; 
  pub_angular_velocity.publish(msg);
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
  move_home(n);
    
  // iterate through all the 9 grid squares.
  bool positions[9] = {};            // keeps track of random positions that have been chosen        
  srand(time(NULL));           
  for(int i = 0; i < 9; i++){

    // choose a random position that has not encountered yet
    int randNum;
    do{
      randNum = rand() % 10;
    } while(positions[randNum] == true);
    positions[randNum] = true;

    
    if(i % 2 == 0){
      // move incremental
      move_incremental(n, randNum);
    }
    else{
      // scratch chin  
      scratch_chin(n);
    }

    // move to grid coordinate
    //pressEnter("Press [Enter] to move to next grid coordinate");
    geometry_msgs::PoseStamped p_target = point(randNum);
    ROS_INFO("Moving to grid square %d", randNum);
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);

    // move back home
    pressEnter("Press [Enter] to move arm back home");
    move_home(n);
  }

  //home arm using service call to arm driver
  pressEnter("Press [Enter] to go home");
  move_home(n);

  // shutdown
  ros::shutdown();

  return 0;
}
