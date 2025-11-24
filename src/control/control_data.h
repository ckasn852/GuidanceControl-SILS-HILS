#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H

#include "xtime_l.h" // [Changed] 시간 자료형

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

    // Rx -> Control 로 넘길 시간 정보 추가
    float req_period_us;    // Req 주기
    float rtt_us;           // RTT (Req -> Rx)
    XTime rx_finish_time;   // Rx 수신 완료 시각 (Rx->Control 지연 계산용)
} control_data;

#endif
