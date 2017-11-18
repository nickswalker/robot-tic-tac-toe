#include "ros/ros.h"
#include "tic_tac_toe/GameState.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace cv;

SimpleBlobDetector *redDetector;
SimpleBlobDetector *blueDetector;

void image_cb(const sensor_msgs::ImageConstPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat img(cv_ptr->image);
    Mat bgr [3];
    split(img, bgr);
    threshold(bgr[0], bgr[0], 40, 255, CV_THRESH_BINARY);
    threshold(bgr[2], bgr[2], 60, 255, CV_THRESH_BINARY);


    // Storage for blobs
    std::vector<KeyPoint> keypoints;

    // Detect blobs
    blobDetector->detect( img, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
    // the size of the circle corresponds to the size of blob

    Mat im_with_keypoints;
    drawKeypoints(img, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Update GUI Window
    imshow("red_channel", bgr[2]);
    imshow("blue_channel", bgr[0]);
    imshow("blobs", im_with_keypoints);
    waitKey(3);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "game_state_detector");

    ros::NodeHandle n;
    ros::NodeHandle ph;
    std::string camera_topic;
    if (!ph.hasParam("camera_topic")) {
      ROS_ERROR("Please set the camera_topic parameter");
      ros::shutdown();
    }

    camera_topic = "/usb_cam/image_raw";
    //ph.getParam("~camera_topic", camera_topic);
    ROS_INFO_STREAM(camera_topic);
  
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 1500;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

  
      blobDetector = new SimpleBlobDetector(params);
      
      ros::Subscriber camera_sub = n.subscribe<sensor_msgs::Image>(camera_topic, 1, image_cb);
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
