#ifndef TIC_TAC_TOE_DRAWING_HELPERS_H
#define TIC_TAC_TOE_DRAWING_HELPERS_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;


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

void drawGrid(Mat &img, vector<RotatedRect> &cells){
    for (vector<RotatedRect>::const_iterator i = cells.begin(); i < cells.end(); ++i) {
        const RotatedRect &cell = *i;
        Point2f vertices[4];
        cell.points(vertices);
        for (int v = 0; v < 4; v++)
            line(img, vertices[v], vertices[(v+1) % 4], Scalar(0,255,0));
    }
}

void drawPieces(Mat &img, vector<Piece> pieces) {
    for(vector<Piece>::const_iterator i = pieces.begin(); i < pieces.end(); ++i) {
        Scalar color = i->team == RED ? Scalar(255, 0, 255) : Scalar(255, 255, 0);
        circle(img, i->center, 5, color, 2);
    }
}

void inspectContours(Mat &img, const vector<Vec2f> &contours){
    for (int i = 0; i < contours.size(); i++) {
        Mat detections_img;
        img.copyTo(detections_img);
        polylines(detections_img, contours[i], false, Scalar(0,255,0));
        imshow("detections_img", detections_img);
        waitKey(3);
    }
}

#endif //TIC_TAC_TOE_DRAWING_HELPERS_H
