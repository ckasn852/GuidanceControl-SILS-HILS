#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H

// Rx 수신 데이터 (언리얼 -> Zynq)
typedef struct {
    float pitch_rate;
    float yaw_rate;
    int img_x;
    int img_y;
    float distance;
    float pitch_wing_deq;
    float yaw_wing_deq;
    int sim_state;
    float p;
    float y;
} control_data;

#endif
