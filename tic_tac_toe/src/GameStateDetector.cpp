#include <tic_tac_toe/piece.h>
#include <tic_tac_toe/GameStateDetector.h>
#include <tic_tac_toe/image_processing.h>
#include <tic_tac_toe/drawing_helpers.h>

using namespace cv;
using namespace std;

void GameStateDetector::assignPiecesToCells(vector<Piece> pieces, vector<RotatedRect> cells, int *dest) {
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

void GameStateDetector::detect(const Mat &img) {
    vector<Piece> pieces = extractPieces(img, blobDetector);

    vector<RotatedRect> gridCells = extractGrid(img);

    assignPiecesToCells(pieces, gridCells, lastObservedState);
    lastObservedWasValid = true;
    if (showDetections) {
        Mat detections_img = img;
        drawPieces(detections_img, pieces);
        drawGrid(detections_img, gridCells);
        imshow("detections_img", detections_img);
        waitKey(3);
    }
}
