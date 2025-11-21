#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

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
// [수정] 태스크 대신 주기적 업데이트 함수로 변경
void IMU_Update(float dt);
// [추가] 최신 데이터 반환 함수
IMUData_t IMU_GetData(void);

#endif
