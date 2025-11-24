#ifndef CONTROL_OUT_HW_H
#define CONTROL_OUT_HW_H

// 실제 하드웨어 센서 데이터 (Hardware Task -> Tx Task)
// 모터 피드백과 IMU 데이터를 한 번에 담아서 보냅니다.
typedef struct {
    float real_motor_angle[6]; // 실제 모터 6개의 각도 (사용 안 하더라도 구조체 유지)
    float real_imu_pitch;      // 실제 IMU Pitch
    float real_imu_yaw;        // 실제 IMU Yaw
} real_sensor_data;

#endif
