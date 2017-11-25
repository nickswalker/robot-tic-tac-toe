#ifndef TIC_TAC_TOE_PIECE_H
#define TIC_TAC_TOE_PIECE_H
#include <opencv2/imgproc/imgproc.hpp>

enum Team {
    RED = 1, BLUE = 2
};

struct Piece {
    Team team;
    cv::Point2f center;
    Piece(Team team, cv::Point2f center): team(team), center(center){};
};

#endif //TIC_TAC_TOE_PIECE_H
