#ifndef TIC_TAC_TOE_MICOIDLEBEHAVIOR_H
#define TIC_TAC_TOE_MICOIDLEBEHAVIOR_H

#include <ros/node_handle.h>
#include <segbot_arm_manipulation/Mico.h>
class MicoIdleBehavior {
    segbot_arm_manipulation::Mico *mico;
public:
    MicoIdleBehavior(segbot_arm_manipulation::Mico* manager):mico(manager){};
    void scratch_chin(int target_duration);
    void move_incremental(int dest, int target_duration);
    void move_exaggerated(int target_duration);
    void tap_fingers(int target_duration);
    void point(uint8_t grid_square, int target_duration);
    void game_over();
};

#endif
