#ifndef TIC_TAC_TOE_IMAGE_PROCESSING_H
#define TIC_TAC_TOE_IMAGE_PROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

void filterChannel(Mat &src, Mat &dest) {
    int erosion_size = 3;
    Mat element = getStructuringElement(MORPH_ELLIPSE,
                                        Size(2 * erosion_size + 1, 2 * erosion_size + 1));

    erode(src, dest, element);
    dilate(dest, dest, element);
}

std::vector<KeyPoint> detectPieces(Mat &img) {

}

std::vector<Vec4i> detectGrid(Mat &img) {

}

bool linesClose(Vec2f &first, Vec2f &second) {
    return fabs(first[0] - second[0]) < 20 && fabs(first[1] - second[1]) < CV_PI * 10 / 180;
}

bool pointsClose(Point &p1, Point &p2, float thresh=64.f) {
    return powf(p1.x - p2.x, 2.f) +
           powf(p1.y - p2.y, 2.f) < powf(thresh,2.f);
}

bool pointsClose(Point &l1p1, Point &l1p2, Point &l2p1, Point &l2p2, float thresh=64.f) {
    // Check if the endpoints are within
    return pointsClose(l1p1, l2p1, thresh) && pointsClose(l1p2, l2p2, thresh);
}

bool isAlmostVerticalOrHorizontal(Vec2f &line) {
    float theta = line[1];
    float eps = 20.f * CV_PI / 180.f;
    // Are we close to 0?
    bool almostHorizontal = fabs(theta) < eps;
    // Are we close to 90?
    bool almostVertical = fabs(theta - CV_PI / 2.f) < eps;
    return almostVertical || almostHorizontal;
}

bool notVerticalHorizontal(Vec2f &line) {
    return !isAlmostVerticalOrHorizontal(line);
}


void lineToPoints(Vec2f &line, Mat &img, Point &p1, Point &p2) {
    float y = line[0];
    float theta1 = line[1];
    if (theta1 > CV_PI * 45 / 180 && theta1 < CV_PI * 135 / 180) {
        p1.x = 0;
        p1.y = y / sin(theta1);

        p2.x = img.size().width;
        p2.y = -p2.x / tan(theta1) + y / sin(theta1);
    } else {
        p1.y = 0;
        p1.x = y / cos(theta1);

        p2.y = img.size().height;
        p2.x = -p2.y * tan(theta1) + y / cos(theta1);

    }
}


void mergeRelatedLines(std::vector<Vec2f> &lines, Mat &img) {
    vector<Vec2f>::iterator current;
    for (current = lines.begin(); current != lines.end(); ++current) {
        Vec2f &line = *current;
        if (line[0] == 0 && line[1] == -100) continue;
        Point l1p1, l1p2;
        lineToPoints(line, img, l1p1, l1p2);


        vector<Vec2f>::iterator other;
        for (other = lines.begin(); other != lines.end(); ++other) {
            Vec2f &other_line = *other;
            if (current == other) continue;

            Point l2p1, l2p2;
            lineToPoints(other_line, img, l2p1, l2p2);

            if (pointsClose(l1p1,l1p2, l2p1, l2p2, 32)) {
                // Merge the two
                line[0] = (line[0] + other_line[0]) / 2;
                line[1] = (line[1] + other_line[1]) / 2;

                //other_line[0] = 0;
                //other_line[1] = -100;
            }

        }
    }
}

#endif //TIC_TAC_TOE_IMAGE_PROCESSING_H
