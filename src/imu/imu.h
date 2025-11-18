#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

// ===== IMU Data Structure =====
typedef struct {
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float angle_roll_deg;   // X axis
    float angle_pitch_deg;  // Y axis
    float angle_yaw_deg;    // Z axis (computed)
} IMUData_t;


// ===== Public API =====
void IMU_Init(void);
void IMU_StartTask(void);
void IMUTask(void *pv);
QueueHandle_t IMU_GetQueue(void);

#endif
