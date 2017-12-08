#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/JointAngles.h>
#include <tic_tac_toe/MicoIdleBehavior.h>
#include <segbot_arm_manipulation/MicoManager.h>
#include <tic_tac_toe/sound.h>
#include <tic_tac_toe/utils.h>
#include <unistd.h>

double grid_cell_poses[][7] = {{0.426824212074,
                                       0.284858942032,
                                       0.0402360036969,
                                       -0.0495440475643,
                                       -0.9955971241,
                                       -0.0676593929529,
                                       -0.0418801791966},
                               {0.427185595036,
                                       0.179492980242,
                                       0.0362501107156,
                                       -0.0409061983228,
                                       -0.998338639736,
                                       -0.00312207173556,
                                       0.0404587648809},
                               {0.431104779243,
                                       0.0673867613077,
                                       0.022669646889,
                                       -0.0227509662509,
                                       -0.997758924961,
                                       -0.0346958227456,
                                       0.0524955913424},
                               {0.323083162308,
                                       0.290391534567,
                                       0.0327621549368,
                                       -0.0466325692832,
                                       -0.998385667801,
                                       -0.0323525331914,
                                       0.00221162941307},
                               {0.314559876919,
                                       0.182191491127,
                                       0.0249998774379,
                                       -0.0345241464674,
                                       -0.999021053314,
                                       -0.0276549495757,
                                       -0.000544465205166},
                               {0.304246366024,
                                       0.0765624493361,
                                       0.0174890849739,
                                       -0.0159492921084,
                                       -0.999459207058,
                                       -0.00590369151905,
                                       -0.0281426385045},
                               {0.207779467106,
                                       0.295100569725,
                                       0.0326444283128,
                                       -0.0451364554465,
                                       -0.997986733913,
                                       -0.0272142477334,
                                       -0.0352782607079},
                               {0.195141017437,
                                       0.187146872282,
                                       0.0303510185331,
                                       -0.0113975815475,
                                       -0.999932646751,
                                       -0.00210533523932,
                                       0.000616318488028,},
                               {0.201366573572,
                                       0.0808693170547,
                                       0.0229740720242,
                                       -0.0264183524996,
                                       -0.997659921646,
                                       -0.0139531157911,
                                       0.0614984557033}};

double const chin_pose[] = {-0.121613927186,
                            0.15239700675,
                            0.586950242519,
                            0.552960515022,
                            -0.396972090006,
                            0.234359323978,
                            0.694063067436};

double tap_pose[] = {
        0.404385447502,
        -0.218024283648,
        0.0239445753396,
        0.818471372128,
        -0.05943377316,
        0.571143269539,
        -0.0191759094596
};
double exaggerated_staging_pose[] = {
        0.311155617237,
        0.189424604177,
        0.213762119412,
        0.665959119797,
        -0.745930075645,
        0.00926198810339,
        0.0010175104253
};

double quadrant_approach_poses[][7] = {{0.359608054161,
                                               0.243571102619,
                                               0.0770178735256,
                                               -0.996901810169,
                                               0.0625432953238,
                                               -0.0410128049552,
                                               0.0243524741381},
                                       {0.354002892971,
                                               0.143095493317,
                                               0.0753956288099,
                                               -0.998840987682,
                                               0.0366943664849,
                                               -0.0288470163941,
                                               0.0117430668324},
                                       {0.259093165398,
                                               0.249157130718,
                                               0.070698030293,
                                               -0.718941569328,
                                               -0.69405412674,
                                               -0.0322042442858,
                                               -0.0193572230637},
                                       {0.259484708309,
                                               0.143753558397,
                                               0.0591824762523,
                                               0.995885074139,
                                               -0.0568450242281,
                                               0.0699009969831,
                                               0.00976687949151}};

double incremental_start_pose[] = {0.311155617237,
                                   0.189424604177,
                                   0.213762119412,
                                   0.665959119797,
                                   -0.745930075645,
                                   0.00926198810339,
                                   0.0010175104253};


void MicoIdleBehavior::point(uint8_t grid_square, int target_duration) {
    mico->move_to_pose_moveit(array_to_pose(grid_cell_poses[grid_square]));
    play_file("place-my-block-here.wav");
    ros::Duration(3).sleep();
}

void MicoIdleBehavior::scratch_chin(int target_duration) {
    ros::Time start_stamp = ros::Time::now();
    ros::Duration target(target_duration);
    // move arm to head
    mico->move_to_pose_moveit(array_to_pose(chin_pose));

    play_file_non_blocking("hmm.wav");
    // scratch chin
    while (ros::Time::now() - start_stamp < target) {
        mico->move_fingers(100, 100);
        mico->move_fingers(5500, 5500);
    }
}

void MicoIdleBehavior::move_incremental(int dest, int target_duration) {
    ros::Time start_stamp = ros::Time::now();
    ros::Duration target(target_duration);
    // move arm to incremental start position
    mico->move_to_pose_moveit(array_to_pose(incremental_start_pose));
    uint8_t quadrant;
    switch (dest) {
        // first quadrant
        case 0:
        case 1:
        case 3:
            quadrant = 0;
            break;

            // second quadrant
        case 2:
        case 5:
            quadrant = 1;
            break;

            // third quadrant
        case 4:
        case 6:
            quadrant = 2;
            break;

            // fourth quadrant
        case 7:
        case 8:
            quadrant = 3;
            break;
    }
    // move to new incremental position based off of destination
    mico->move_to_pose_moveit(array_to_pose(quadrant_approach_poses[quadrant]));

    kinova_msgs::JointAngles msg;
    play_file_non_blocking("maybe-here-or-maybe-here.wav");
    while (ros::Time::now() - start_stamp < target) {
        // wave wrist around
        msg.joint5 = 20;
        mico->move_with_angular_velocities(msg, 1.25);
        ros::Duration(.75).sleep();
        msg.joint5 = -20;
        mico->move_with_angular_velocities(msg, 1.25);

    }

}

void MicoIdleBehavior::move_exaggerated(int target_duration) {
    ros::Time start_stamp = ros::Time::now();
    ros::Duration target(target_duration);
    // move to exaggerated staging position
    mico->move_to_pose_moveit(array_to_pose(exaggerated_staging_pose));

    play_file_non_blocking("wind-up.wav");

    // rotate wrist around
    kinova_msgs::JointAngles msg;
    msg.joint6 = 45;
    mico->move_with_angular_velocities(msg, 8);

    play_file_non_blocking("stretch.wav");

    mico->move_fingers(500, 500);
    while (ros::Time::now() - start_stamp < target) {
        // open and close finger
        mico->move_fingers(500, 1000);
        mico->move_fingers(1000, 500);
    }
    mico->close_hand();

}

void MicoIdleBehavior::tap_fingers(int target_duration) {
    ros::Time start_stamp = ros::Time::now();
    ros::Duration target(target_duration);
    // move to tapping staging position
    mico->move_to_pose_moveit(array_to_pose(tap_pose));

    // rotate wrist into position
    kinova_msgs::JointAngles msg;
    msg.joint6 = 45;

    mico->move_with_angular_velocities(msg, 2.5);
    play_file_non_blocking("tapping.wav");

    // open and close finger
     while (ros::Time::now() - start_stamp < target) {
        mico->move_fingers(100, 5500);
        mico->move_fingers(5500, 5500);
    }

    geometry_msgs::PoseStamped p_target = array_to_pose(tap_pose);
    // raise hand up before finishing so that it does not collide with table
    p_target.pose.position.z += 0.1;
    p_target.pose.position.y += 0.1;
    mico->move_to_pose(p_target);
}

void MicoIdleBehavior::game_over() {
    ROS_INFO("Game Over");
    play_file("good-game.wav");
}
