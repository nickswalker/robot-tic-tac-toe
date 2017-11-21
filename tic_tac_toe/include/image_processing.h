#ifndef TIC_TAC_TOE_IMAGE_PROCESSING_H
#define TIC_TAC_TOE_IMAGE_PROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <piece.h>
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

vector<Piece> extractPieces() {

}

vector<Point> extractGridContour(vector<vector<Point> > contours, vector<Vec4i> hierarchy) {
    // Get the moments
    vector<Moments> mu(contours.size());
    for(int i = 0; i < contours.size(); i++) {
        mu[i] = moments(contours[i], false);
    }

    // Get the areas
    vector<double> areas(contours.size());
    vector<double> lengths(contours.size());
    for(int i = 0; i < contours.size(); i++ ) {
        areas[i] = contourArea(contours[i], false);
        lengths[i] = arcLength(contours[i], false);
    }

    //  Get the mass centers:
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00 , mu[i].m01 / mu[i].m00);
    }

    vector<Point> result;
    double largest_yet = 0;
    for(int i = 0; i < contours.size(); i++) {
        double area = areas[i];
        if (area < 1500. || area > 4000) continue;
        if (area > largest_yet) {
            result = contours[i];
            largest_yet = area;
        }
    }
    return result;
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

void drawContours(Mat &img, vector<vector<Point> > &contours,  vector<Vec4i> &hierarchy, vector<Point2f> *centers = NULL) {
    Scalar color(255, 0, 0);
    // Draw contours
    for(int i = 0; i < contours.size(); i++) {
        drawContours( img, contours, i, color, 2, 8, hierarchy, 0, Point() );
        if (centers != NULL) {
            circle(img, (*centers)[i], 4, color, -1, 8, 0);
        }
    }

}

void drawCorners(Mat &img, Mat &cornerness) {
    // Drawing a circle around corners
    for( int j = 0; j < cornerness.rows ; j++ ) {
        for( int i = 0; i < cornerness.cols; i++ ) {
            if( (int) cornerness.at<float>(j,i) > 200 ) {
                circle(img, Point(i, j), 5,  Scalar(0), 2, 8, 0);
            }
        }
    }
}

void drawLines(Mat &img, vector<Vec2f> &lines) {
    for (size_t i = 0; i < lines.size(); i++) {
      float r = lines[i][0], t = lines[i][1];
      double cos_t = cos(t), sin_t = sin(t);
      double x0 = r * cos_t, y0 = r * sin_t;
      double alpha = 1000;

      Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
      Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
      line(img, pt1, pt2, Scalar(0, 255, 0), 3);
    }
}

void detectCorners(Mat &img, vector<Vec2f> &lines){
    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.08;
    Mat corner, corner_norm;
    /// Detecting corners
    cornerHarris(img, corner, blockSize, apertureSize, k, BORDER_DEFAULT);
    //kmeans();
    /// Normalizing
    double min, max;
    cv::minMaxLoc(corner, &min, &max);
    normalize(corner, corner_norm, 0, 255, NORM_MINMAX, CV_32FC1);
    convertScaleAbs(corner_norm, corner);
}

void inspectContours(Mat &img, vector<Vec2f> &contours){
    for (int i = 0; i < contours.size(); i++) {
        Mat detections_img;
        img.copyTo(detections_img);
        polylines(detections_img, contours[i], false, Scalar(0,255,0));
        imshow("detections_img", detections_img);
        waitKey(3);
    }
}

Mat rotate(double theta, Vec2f point) {
    Mat rotation(2,2, CV_64F);
    rotation.at<double>(0,0) = cos(theta);
    rotation.at<double>(0,1) = -sin(theta);
    rotation.at<double>(1,0) = sin(theta);
    rotation.at<double>(1,1) = cos(theta);

    Mat p(2, 1, CV_64F);
    p.at<double>(0, 0) = point[0];
    p.at<double>(0, 1) = point[1];
    return rotation * p;
}

Mat pointToMat(Point2f point) {
    Mat mat(2, 1, CV_64F);
    mat.at<double>(0,0) = point.x;
    mat.at<double>(1,0) = point.y;
    return mat;
}

Point2f matToPoint(Mat mat) {
    return Point2f(mat.at<double>(0), mat.at<double>(1));
}

vector<RotatedRect> produceGrid(RotatedRect &boundingBox) {
    double theta = boundingBox.angle * CV_PI / 180.f;
    if (theta < -CV_PI / 4) {
        boundingBox.size = Size(boundingBox.size.height, boundingBox.size.width);
        boundingBox.angle = boundingBox.angle + 90;
        theta = boundingBox.angle * CV_PI / 180.f;
    }
    Mat toCorner = rotate(theta,Vec2f(boundingBox.size.width / 2, boundingBox.size.height / 2));
    Mat origin = pointToMat(boundingBox.center);
    Mat topLeft = origin - toCorner;

    RotatedRect cell = boundingBox;
    double cellWidth = boundingBox.size.width / 3;
    double cellHeight = boundingBox.size.height / 3;
    cell.size = Size(cellWidth, cellHeight);

    // Vectors going off the top left corner along the principle axes
    Mat halfDown = rotate(theta, Vec2f(0, cellHeight / 2));
    Mat halfRight = rotate(theta, Vec2f(cellWidth / 2, 0));

    vector<RotatedRect> result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Mat newCenter = topLeft + (halfRight * 2 * j) + (halfDown * 2 * i) + halfRight + halfDown;
            cell.center = matToPoint(newCenter);
            RotatedRect newCell = cell;
            result.push_back(newCell);
        }
    }
    return result;
}

void drawGrid(Mat &img, vector<RotatedRect> &cells){
    for (vector<RotatedRect>::const_iterator i = cells.begin(); i < cells.end(); ++i) {
        const RotatedRect &cell = *i;
        Point2f vertices[4];
        cell.points(vertices);
        for (int v = 0; v < 4; v++)
            line(img, vertices[v], vertices[(v+1) % 4], Scalar(0,255,0));
    }
}

#endif //TIC_TAC_TOE_IMAGE_PROCESSING_H
