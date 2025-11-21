// pid_tuning_mode.h
#ifndef PID_TUNING_MODE_H
#define PID_TUNING_MODE_H

#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int   tuning_active;    // 0=튜닝 적용중, 1=미적용중
    float Kp, Ki, Kd;       // 수신된 PID 게인
    int   has_params;       // 1이면 P/I/D 동봉
} PIDUpdate_t;

extern QueueHandle_t xPidUpdateQueue; // [MOD]

#ifdef __cplusplus
}
#endif

#endif // PID_TUNING_MODE_H
