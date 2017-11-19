#include "ros/ros.h"
#include "tic_tac_toe/GameState.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace cv;

SimpleBlobDetector *blobDetector;
bool show_detections;

void filterChannel(Mat &src, Mat &dest) {
    int erosion_size = 4;
    Mat element = getStructuringElement(MORPH_ELLIPSE,
                                       Size(2*erosion_size + 1, 2*erosion_size+1));

    erode(src, dest, element);
    dilate(dest, dest, element);
}

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
    Mat hsv, red, blue;
    cvtColor(img, hsv, CV_BGR2HSV);
    inRange(hsv, Scalar(0, 50, 50), Scalar(20,255,255), red);
    inRange(hsv, Scalar(100, 50, 50), Scalar(115, 255, 255), blue);

    filterChannel(red, red);
    filterChannel(blue, blue);
   

    // Storage for blobs
    std::vector<KeyPoint> red_keypoints;
    std::vector<KeyPoint> blue_keypoints;
    
    // Detect blobs
    blobDetector->detect(red, red_keypoints);
    blobDetector->detect(blue, blue_keypoints);

    // Display preprocessed channels (for debugging)
    /*
    Mat display;
    cvtColor(red, display, CV_GRAY2BGR);
    imshow("red_channel", display);
    cvtColor(blue, display, CV_GRAY2BGR);
    imshow("blue_channel", display);
    */
    if (show_detections) {
        Mat im_with_keypoints;
        drawKeypoints(img, red_keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        drawKeypoints(im_with_keypoints, blue_keypoints, im_with_keypoints, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        imshow("blobs", im_with_keypoints);
        waitKey(3);
    }
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

    ph.getParam("camera_topic", camera_topic);
    ph.param<bool>("show_detections", show_detections, true);
    ROS_INFO_STREAM(camera_topic);
  
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    params.filterByColor = true;
    params.blobColor = 255;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 1500;
    //params.maxArea = 2000;

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


    while (ros::ok()) {
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
