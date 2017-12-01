#ifndef TIC_TAC_TOE_MICOIDLEBEHAVIOR_H
#define TIC_TAC_TOE_MICOIDLEBEHAVIOR_H

#include <ros/node_handle.h>
class MicoManager;
class MicoIdleBehavior {
    MicoManager *mico;
public:
    MicoIdleBehavior();
    MicoIdleBehavior(ros::NodeHandle n, MicoManager* manager);
    void scratch_chin();
    void move_incremental(int dest);
    void move_exaggerated();
    void tap_fingers();
    void point(uint8_t grid_square);
    void game_over();
};

#endif
