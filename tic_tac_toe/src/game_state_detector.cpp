#include "ros/ros.h"
#include "tic_tac_toe/GameState.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_processing.h>
#include <drawing_helpers.h>
#include <piece.h>

using namespace cv;

SimpleBlobDetector *blobDetector;
bool show_detections;
RNG rng(12345);

int lastObservedState[9];
bool lastObservedWasValid = false;


void assignPiecesToCells(vector<Piece> pieces, vector<RotatedRect> cells, int *dest) {
    memset(dest,0,9 * sizeof(int));
    Point2f vertices[4];
    for (int i = 0; i < pieces.size(); i++) {
        Piece currentPiece = pieces[i];
        for (int j = 0; j < cells.size(); j++) {
            RotatedRect cell = cells[j];
            cell.points(vertices);
            vector<Point2f> cellCorners(vertices, vertices + 4);
            bool isInside = pointPolygonTest(cellCorners, currentPiece.center ,false) > 0;
            if (isInside) {
                dest[j] = currentPiece.team;
                break;
            }
        }
    }
}

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

    vector<Piece> pieces = extractPieces(img, *blobDetector);

    vector<RotatedRect> gridCells = extractGrid(img);

    assignPiecesToCells(pieces, gridCells, lastObservedState);
    lastObservedWasValid = true;
    if (show_detections) {
        Mat detections_img = img;
        drawPieces(detections_img, pieces);
        drawGrid(detections_img, gridCells);
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
    params.minArea = 350;
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
        if (!lastObservedWasValid) {
            ros::spinOnce();
            continue;
        }
        tic_tac_toe::GameState msg;
        for (int i = 0; i < 9; i++) {
            msg.board_state.push_back(lastObservedState[i]);
        }

        state_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
