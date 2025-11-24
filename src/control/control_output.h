#ifndef CONTROL_OUTPUT_H
#define CONTROL_OUTPUT_H

#include "xtime_l.h"

typedef struct {
    float pitch_fin_deg;
    float yaw_fin_deg;
    float target_pitch;
    float target_yaw;
    int sim_state;
    float rx_to_ctrl_delay_us;  // 3. Rx -> Control 큐 지연 시간
    XTime ctrl_start_time;      // 4. Control 시작 시간 (Tx에서 종료 시간과 비교하여 지연 계산용)
} control_output;

#endif
