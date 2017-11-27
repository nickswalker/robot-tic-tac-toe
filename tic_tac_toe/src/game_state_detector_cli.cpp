#include "ros/ros.h"
#include "tic_tac_toe/GameState.h"
#include "sensor_msgs/Image.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tic_tac_toe/GameStateDetector.h>


using namespace cv;

int main(int argc, char **argv) {
    ros::init(argc, argv, "game_state_detector");

    GameStateDetector detector(true);
    string path = argv[1];

    Mat img = imread(path, CV_LOAD_IMAGE_COLOR);
    if (img.empty()) {
        cerr << "Couldn't load image from " << path << "." << endl;
        exit(1);
    }
    detector.detect(img);
    if (!detector.lastObservedWasValid) {
        cerr << "Couldn't detect state" << endl;
        exit(2);
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cout << " " << detector.lastObservedState[i * 3 + j];
        }
        cout << endl;
    }
    waitKey(0);
    return 0;
}
