#ifndef TIC_TAC_TOE_GAME_STATE_DETECTOR_H
#define TIC_TAC_TOE_GAME_STATE_DETECTOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#define IMAGE_AREA 1920 * 1080

struct Piece;

class GameStateDetector {
    cv::Ptr<cv::SimpleBlobDetector> blobDetector;
    bool showDetections;;

public:
    GameStateDetector(bool showDetections = false): showDetections(showDetections){
        // Setup SimpleBlobDetector parameters.
        cv::SimpleBlobDetector::Params params;

        params.filterByColor = true;
        params.blobColor = 255;

        params.filterByArea = true;
        params.minArea = IMAGE_AREA * 0.001;
        params.maxArea = IMAGE_AREA * 0.01;

        params.filterByCircularity = false;
        params.minCircularity = 0.01;

        params.filterByConvexity = true;
        params.minConvexity = 0.87;

        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;
        blobDetector = cv::SimpleBlobDetector::create(params);
    }

    bool detect(const cv::Mat &img, int state[]);
    bool detect_on_board(const cv::Mat &img, const std::vector<cv::RotatedRect> &board, int result[]);
    bool detect_board(const cv::Mat &img, std::vector<cv::RotatedRect> &board);


    void assignPiecesToCells(std::vector<Piece> pieces, std::vector<cv::RotatedRect> cells, int dest[]);
};


#endif //TIC_TAC_TOE_GAME_STATE_DETECTOR_H
