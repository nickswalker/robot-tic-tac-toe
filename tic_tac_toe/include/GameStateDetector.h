#ifndef TIC_TAC_TOE_GAME_STATE_DETECTOR_H
#define TIC_TAC_TOE_GAME_STATE_DETECTOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

typedef struct Piece;
using namespace cv;
using namespace std;

class GameStateDetector {
    SimpleBlobDetector blobDetector;
    bool showDetections;;

public:

    bool lastObservedWasValid;
    int lastObservedState[9];

    GameStateDetector(bool showDetections = false): showDetections(showDetections), lastObservedWasValid(false),
                                                    lastObservedState() {
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

        blobDetector = SimpleBlobDetector(params);
    }

    void detect(const Mat &img);


    void assignPiecesToCells(vector<Piece> pieces, vector<RotatedRect> cells, int *dest);
};


#endif //TIC_TAC_TOE_GAME_STATE_DETECTOR_H
