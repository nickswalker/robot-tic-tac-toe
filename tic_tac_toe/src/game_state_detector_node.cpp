#include "ros/ros.h"
#include "tic_tac_toe/GameState.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tic_tac_toe/GameStateDetector.h>

using namespace cv;

GameStateDetector *detector;

void image_cb(const sensor_msgs::ImageConstPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat img(cv_ptr->image);
    detector->detect(img);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "game_state_detector");

    ros::NodeHandle n;
    ros::NodeHandle ph("~");
    std::string camera_topic;
    if (!ph.hasParam("camera_topic")) {
        ROS_ERROR("Please set the camera_topic parameter");
        ros::shutdown();
    }

    bool show_detections = false;
    ph.getParam("camera_topic", camera_topic);
    ph.param<bool>("show_detections", show_detections, true);
    ROS_INFO_STREAM(camera_topic);



    detector = new GameStateDetector(show_detections);

    ros::Subscriber camera_sub = n.subscribe<sensor_msgs::Image>(camera_topic, 1, image_cb);
    ros::Publisher state_pub = n.advertise<tic_tac_toe::GameState>("game_state", 1000);

    ros::Rate loop_rate(10);


    while (ros::ok()) {
        if (!detector->lastObservedWasValid) {
            ros::spinOnce();
            continue;
        }
        tic_tac_toe::GameState msg;
        for (int i = 0; i < 9; i++) {
            msg.board_state.push_back(detector->lastObservedState[i]);
        }

        state_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
