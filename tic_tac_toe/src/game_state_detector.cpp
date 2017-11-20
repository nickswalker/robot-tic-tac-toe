#include "ros/ros.h"
#include "tic_tac_toe/GameState.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_processing.h>
#include <algorithm>
using namespace cv;

SimpleBlobDetector *blobDetector;
bool show_detections;


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
    Mat hsv, red, blue, grid;
    cvtColor(img, hsv, CV_BGR2HSV);
    inRange(hsv, Scalar(0, 50, 50), Scalar(20, 255, 255), red);
    inRange(hsv, Scalar(100, 50, 50), Scalar(125, 255, 255), blue);

    filterChannel(red, red);
    filterChannel(blue, blue);

    // Storage for blobs
    std::vector<KeyPoint> red_keypoints;
    std::vector<KeyPoint> blue_keypoints;

    // Detect blobs
    blobDetector->detect(red, red_keypoints);
    blobDetector->detect(blue, blue_keypoints);

    Mat edge;
    inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 80), grid);
    //filterChannel(grid, grid);
    Canny(grid, edge, 50, 200, 3);
    //cvtColor(edge, edge, CV_GRAY2BGR);
    vector<Vec2f> s_lines;
    //HoughLines(edge, s_lines, 1, CV_PI / 180, 80, 0, 0);
    //mergeRelatedLines(s_lines, img);
    //s_lines.erase(std::remove_if(s_lines.begin(), s_lines.end(), notVerticalHorizontal), s_lines.end());

    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.08;
    Mat corner, corner_norm;
    /// Detecting corners
    //cornerHarris(edge, corner, blockSize, apertureSize, k, BORDER_DEFAULT);
    //kmeans();
    /// Normalizing
    //double min, max;
    //cv::minMaxLoc(corner, &min, &max);
    //normalize(corner, corner_norm, 0, 255, NORM_MINMAX, CV_32FC1);
    //convertScaleAbs(corner_norm, corner);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours(edge, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    if (show_detections) {
        // Display preprocessed channels (for debugging)
        Mat display;
        cvtColor(red, display, CV_GRAY2BGR);
        imshow("red_channel", display);
        cvtColor(blue, display, CV_GRAY2BGR);
        imshow("blue_channel", display);
        cvtColor(grid, display, CV_GRAY2BGR);
        imshow("grid_channel", display);
        cvtColor(edge, display, CV_GRAY2BGR);
        imshow("edge_channel", edge);

        Mat detections_img;
        drawKeypoints(img, red_keypoints, detections_img, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        drawKeypoints(detections_img, blue_keypoints, detections_img, Scalar(255, 0, 0),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        /// Drawing a circle around corners
        /*for( int j = 0; j < corner_norm.rows ; j++ ) {
            for( int i = 0; i < corner_norm.cols; i++ ) {
                if( (int) corner_norm.at<float>(j,i) > 200 ) {
                    circle(detections_img, Point(i, j), 5,  Scalar(0), 2, 8, 0);
                }
            }
        }*/
        RNG rng(12345);
        /// Get the moments
        vector<Moments> mu(contours.size());
        for( int i = 0; i < contours.size(); i++ )
        { mu[i] = moments( contours[i], false ); }

        /// Get the areas
        vector<double> areas(contours.size());
        for( int i = 0; i < contours.size(); i++ )
        { areas[i] = contourArea( contours[i], false ); }

        ///  Get the mass centers:
        vector<Point2f> mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

        /// Draw contours
        for( int i = 0; i< contours.size(); i++ ) {
            if(areas[i] <1000) continue;
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( detections_img, contours, i, color, 2, 8, hierarchy, 0, Point() );
            circle( detections_img, mc[i], 4, color, -1, 8, 0 );
        }
        /*
        for (size_t i = 0; i < s_lines.size(); i++) {
            float r = s_lines[i][0], t = s_lines[i][1];
            double cos_t = cos(t), sin_t = sin(t);
            double x0 = r * cos_t, y0 = r * sin_t;
            double alpha = 1000;

            Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
            Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
            line(detections_img, pt1, pt2, Scalar(0, 255, 0), 3);
        }*/


        imshow("detections_img", detections_img);
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

    params.filterByArea = true;
    params.minArea = 650;
    params.maxArea = 1500;

    params.filterByCircularity = true;
    params.minCircularity = 0.1;

    params.filterByConvexity = true;
    params.minConvexity = 0.87;

    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;


    blobDetector = new SimpleBlobDetector(params);

    ros::Subscriber camera_sub = n.subscribe<sensor_msgs::Image>(camera_topic, 1, image_cb);
    ros::Publisher state_pub = n.advertise<tic_tac_toe::GameState>("game_state", 1000);

    ros::Rate loop_rate(10);


    while (ros::ok()) {
        tic_tac_toe::GameState msg;

        state_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
