// control_data.h
#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H

typedef struct {
    int img_x;
    int img_y;
    float distance;
    float yaw_rate; 		// 동체 각속도
    float pitch_rate;
    float yaw_wing_deq;		// 날개 각속도
    float pitch_wing_deq;
    int sim_state;			// 시뮬레이션 상태(진행중 or 종료)
    float p;
    float y;
} control_data;

#endif
