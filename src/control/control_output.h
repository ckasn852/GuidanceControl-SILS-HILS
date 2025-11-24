#ifndef CONTROL_OUTPUT_H
#define CONTROL_OUTPUT_H

#include "xtime_l.h"

typedef struct {
    float pitch_fin_deg;
    float yaw_fin_deg;
    float target_pitch;
    float target_yaw;
    int sim_state;

    // Control -> Tx 로 넘길 시간 정보 추가
    float req_period_us;           // Req 주기
    float rtt_us;                  // RTT
    float ctrl_proc_us;            // [MOD] Control 처리 시간 (Rx 데이터 처리 시작 ~ Tx로 넘기기 전까지)
    XTime ctrl_finish_time;        // [MOD] Control 종료 시각 (Control->Tx 지연 계산용 기준 시각)
} control_output;

#endif
