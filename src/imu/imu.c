#include "imu.h"
#include "xiicps.h"
#include "math.h"
#include "xil_printf.h"

/* 하드웨어 연결 여부 스위치
 * 0: 센서 없음 (가상 모드 - 멈추지 않음)
 * 1: 실제 센서 연결됨 (I2C 통신 수행)
 */
#define USE_REAL_IMU_HW     1

/* I2C Config */
#define IMU_ADDR            0x68
#define IIC_DEVICE_ID       XPAR_XIICPS_0_DEVICE_ID
#define IIC_SCLK_RATE       100000

/* Registers */
#define REG_ACCEL_XOUT_H    0x2D
#define REG_GYRO_XOUT_H     0x33
#define IMU_SLAVE_ADDR      0x68

#define REG_BANK_SEL        0x7F
#define GYRO_SMPLRT_DIV     0x00
#define USER_BANK_0         0x00
#define USER_BANK_1         0x10
#define USER_BANK_2         0x20
#define USER_BANK_3         0x30
#define UB2_GYRO_CONFIG_1   0x01
#define UB2_ACCEL_CONFIG    0x14
#define UB0_PWR_MGMT_1      0x06
#define UB0_PWR_MGMT_2      0x07

/* Filter Settings */
#define ALPHA               0.95f

/* Scaling */
#define GYRO_SCALE          131.072f
#define ACC_SCALE           16384.0f
#define CALIBRATION_SAMPLES 500
#define ANGLE_SETTLE_SAMPLES 500

/* ---- Adaptive Bias Filter ---- */
#define BETA_BIAS           0.002f
#define GYRO_STILL_THRESH   1.5f
#define ACC_STILL_TOL       0.08f

static XIicPs Iic;

/* Filter State */
static float roll = 0, pitch = 0, yaw = 0;
static float gyro_bias_z = 0.0f;
static float roll_offset = 0, pitch_offset = 0, yaw_offset = 0;
static bool angle_calibrated = false;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float calibrated_gyro_x, calibrated_gyro_y, calibrated_gyro_z;

// 데이터 저장용 전역 변수
static IMUData_t current_data = {0};

/* --- Helper I2C Functions --- */
static inline void WriteReg(uint8_t reg, uint8_t val)
{
#if USE_REAL_IMU_HW
    uint8_t buf[2] = {reg, val};
    while (XIicPs_BusIsBusy(&Iic));
    XIicPs_ClearOptions(&Iic, XIICPS_REP_START_OPTION);
    XIicPs_MasterSendPolled(&Iic, buf, 2, IMU_SLAVE_ADDR);
    while (XIicPs_BusIsBusy(&Iic));
#endif
}

static void ReadReg(uint8_t reg, uint8_t *out, uint8_t len)
{
#if USE_REAL_IMU_HW
    XIicPs_SetOptions(&Iic, XIICPS_REP_START_OPTION);
    XIicPs_MasterSendPolled(&Iic, &reg, 1, IMU_ADDR);
    XIicPs_MasterRecvPolled(&Iic, out, len, IMU_ADDR);
    XIicPs_ClearOptions(&Iic, XIICPS_REP_START_OPTION);
#else
    // 가상 모드에서는 버퍼를 0으로 초기화 (안전장치)
    for(int i=0; i<len; i++) out[i] = 0;
#endif
}

/* ---- Stationary Detection ---- */
static bool is_stationary(float gx, float gy, float gz, float ax, float ay, float az)
{
    float gyro_norm = fabs(gx) + fabs(gy) + fabs(gz);
    float acc_norm  = sqrtf(ax*ax + ay*ay + az*az);
    return (gyro_norm < GYRO_STILL_THRESH) && (fabs(acc_norm - 1.0f) < ACC_STILL_TOL);
}

/* ---- Adaptive Yaw Drift Correction ---- */
static float update_yaw_filtered(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    bool still = is_stationary(gx, gy, gz, ax, ay, az);
    if(still)
        gyro_bias_z = gyro_bias_z * (1.0f - BETA_BIAS) + (gz * BETA_BIAS);
    return gz - gyro_bias_z;
}

void IMU_Calibrate(void);

/* ---- IMU Update Function (Called by Hardware Task) ---- */
void IMU_Update(float dt)
{
#if USE_REAL_IMU_HW
    // [실제 하드웨어 모드]
    uint8_t buffer[12];
    ReadReg(REG_ACCEL_XOUT_H, buffer, 12);

    int16_t ax_raw = (buffer[0] << 8) | buffer[1];
    int16_t ay_raw = (buffer[2] << 8) | buffer[3];
    int16_t az_raw = (buffer[4] << 8) | buffer[5];

    int16_t gx_raw = (buffer[6] << 8) | buffer[7];
    int16_t gy_raw = (buffer[8] << 8) | buffer[9];
    int16_t gz_raw = (buffer[10] << 8) | buffer[11];

    calibrated_gyro_x = (float)gx_raw - gyro_offset_x;
    calibrated_gyro_y = (float)gy_raw - gyro_offset_y;
    calibrated_gyro_z = (float)gz_raw - gyro_offset_z;

    float ax = ax_raw / ACC_SCALE;
    float ay = ay_raw / ACC_SCALE;
    float az = az_raw / ACC_SCALE;

    float gx = calibrated_gyro_x / GYRO_SCALE;
    float gy = calibrated_gyro_y / GYRO_SCALE;
    float gz = calibrated_gyro_z / GYRO_SCALE;

    /* Filter yaw with bias compensation */
    float gz_filtered = update_yaw_filtered(gx, gy, gz, ax, ay, az, dt);

    /* Complementary Fusion */
    roll  = ALPHA * (roll  + gx * dt) + (1 - ALPHA) * (-atan2f(ay, az) * 57.3f) ;
    pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * ( atan2f(ax, sqrtf(ay*ay + az*az)) * 57.3f);
    yaw  += gz_filtered * dt;

    float roll_out  = -(roll  - roll_offset);
    float pitch_out = pitch - pitch_offset;
    float yaw_out   = yaw   - yaw_offset;

    /* Output Update */
    current_data.angle_roll_deg  = roll_out;
    current_data.angle_pitch_deg = pitch_out;
    current_data.angle_yaw_deg   = yaw_out;
    current_data.gyro_x_dps = gx;
    current_data.gyro_y_dps = gy;
    current_data.gyro_z_dps = gz;

#else
    // [가상 모드] 센서가 없으므로 0 또는 테스트 값 유지
    // 멈추지 않고 넘어가게 하는 것이 목적
    current_data.angle_roll_deg  = 0.0f;
    current_data.angle_pitch_deg = 0.0f;
    current_data.angle_yaw_deg   = 0.0f;
    current_data.gyro_x_dps = 0.0f;
    current_data.gyro_y_dps = 0.0f;
    current_data.gyro_z_dps = 0.0f;
#endif
}

IMUData_t IMU_GetData(void)
{
    return current_data;
}

/* ---- Public API ---- */
void IMU_Init(void)
{
#if USE_REAL_IMU_HW
    XIicPs_Config *cfg = XIicPs_LookupConfig(IIC_DEVICE_ID);
    XIicPs_CfgInitialize(&Iic, cfg, cfg->BaseAddress);
    XIicPs_SelfTest(&Iic);
    XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);

    uint8_t who = 0;

    WriteReg(REG_BANK_SEL, USER_BANK_0);
    ReadReg(0x00, &who, 1);

    if(who != 0xEA) {
        xil_printf("[ICM-20948] WHO_AM_I FAIL (0x%02X)\r\n", who);
    } else {
        xil_printf("[ICM-20948] WHO_AM_I OK (0xEA)\r\n");
    }

    WriteReg(UB0_PWR_MGMT_1, 0x01);
    WriteReg(UB0_PWR_MGMT_2, 0x00);

    WriteReg(REG_BANK_SEL, USER_BANK_2);

    WriteReg(UB2_GYRO_CONFIG_1, 0x21);
    WriteReg(UB2_ACCEL_CONFIG, 0x21);
    WriteReg(REG_BANK_SEL, USER_BANK_0);
    WriteReg(GYRO_SMPLRT_DIV, 0x09);

    IMU_Calibrate();
    xil_printf("[IMU] Init Complete.\r\n");
#else
    xil_printf("[IMU] Simulated Mode (No Hardware). Init Skipped.\r\n");
#endif
}

void IMU_Calibrate(void)
{
#if USE_REAL_IMU_HW
    uint8_t buffer[12];
    xil_printf("\n[IMU] Calibration Start... Keep sensor still!\r\n");

    /* 1) Gyro Offset */
    gyro_offset_x = gyro_offset_y = gyro_offset_z = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        ReadReg(REG_GYRO_XOUT_H, buffer, 6);
        int16_t gx_raw = (buffer[0] << 8) | buffer[1];
        int16_t gy_raw = (buffer[2] << 8) | buffer[3];
        int16_t gz_raw = (buffer[4] << 8) | buffer[5];
        gyro_offset_x += gx_raw;
        gyro_offset_y += gy_raw;
        gyro_offset_z += gz_raw;

        // OS 딜레이 대신 busy-wait 사용 (초기화 단계이므로 가능)
        for(volatile int k=0; k<100000; k++);
    }
    gyro_offset_x /= CALIBRATION_SAMPLES;
    gyro_offset_y /= CALIBRATION_SAMPLES;
    gyro_offset_z /= CALIBRATION_SAMPLES;

    /* 2) Angle Offset */
    float roll_acc = 0, pitch_acc = 0;
    for(int i = 0; i < ANGLE_SETTLE_SAMPLES; i++) {
        ReadReg(REG_ACCEL_XOUT_H, buffer, 6);
        int16_t ax_raw = (buffer[0] << 8) | buffer[1];
        int16_t ay_raw = (buffer[2] << 8) | buffer[3];
        int16_t az_raw = (buffer[4] << 8) | buffer[5];
        float ax = ax_raw / ACC_SCALE;
        float ay = ay_raw / ACC_SCALE;
        float az = az_raw / ACC_SCALE;
        float r = -atan2f(ay, az) * 57.3f;
        float p = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.3f;
        roll_acc  += r;
        pitch_acc += p;
        for(volatile int k=0; k<100000; k++);
    }
    roll_offset  = roll_acc  / ANGLE_SETTLE_SAMPLES;
    pitch_offset = pitch_acc / ANGLE_SETTLE_SAMPLES;
    yaw_offset   = yaw;
    angle_calibrated = true;
    xil_printf("[IMU] Calibration Completed.\n\n");
#endif
}
