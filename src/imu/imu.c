#include "imu.h"
#include "xiicps.h"
#include "math.h"
#include "task.h"
#include "xil_printf.h"

/* I2C Config */
#define IMU_ADDR            0x68
#define IIC_DEVICE_ID       XPAR_XIICPS_0_DEVICE_ID
#define IIC_SCLK_RATE       100000

/* Registers */
#define REG_ACCEL_XOUT_H    0x2D
#define REG_GYRO_XOUT_H     0x33
#define IMU_SLAVE_ADDR		0x68


#define REG_BANK_SEL		0x7F
#define GYRO_SMPLRT_DIV		0x00
#define USER_BANK_0			0x00
#define USER_BANK_1			0x10
#define USER_BANK_2			0x20
#define USER_BANK_3			0x30
#define UB2_GYRO_CONFIG_1   0x01
#define UB2_ACCEL_CONFIG    0x14
#define UB0_PWR_MGMT_1		0x06
#define UB0_PWR_MGMT_2		0x07

/* Task Settings */
#define IMU_TASK_RATE_MS    10
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
static QueueHandle_t imuQueue = NULL;

/* Filter State */
static float roll = 0, pitch = 0, yaw = 0;
static float gyro_bias_z = 0.0f;
static float roll_offset = 0, pitch_offset = 0, yaw_offset = 0;
static bool angle_calibrated = false;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float calibrated_gyro_x, calibrated_gyro_y, calibrated_gyro_z;
/* --- Helper I2C Functions --- */
static inline void WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};

    while (XIicPs_BusIsBusy(&Iic));
    XIicPs_ClearOptions(&Iic, XIICPS_REP_START_OPTION);
    XIicPs_MasterSendPolled(&Iic, buf, 2, IMU_SLAVE_ADDR);
    while (XIicPs_BusIsBusy(&Iic));
}

static void ReadReg(uint8_t reg, uint8_t *out, uint8_t len)
{
    XIicPs_SetOptions(&Iic, XIICPS_REP_START_OPTION);
    XIicPs_MasterSendPolled(&Iic, &reg, 1, IMU_ADDR);
    XIicPs_MasterRecvPolled(&Iic, out, len, IMU_ADDR);
    XIicPs_ClearOptions(&Iic, XIICPS_REP_START_OPTION);
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

/* ---- IMU FreeRTOS Task ---- */
void IMUTask(void *pv)
{
    TickType_t last = xTaskGetTickCount();
    uint8_t buffer[12];

    while(1)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(IMU_TASK_RATE_MS));

        ReadReg(REG_ACCEL_XOUT_H, buffer, 12);

        int16_t ax_raw = (buffer[0] << 8) | buffer[1];
        int16_t ay_raw = (buffer[2] << 8) | buffer[3];
        int16_t az_raw = (buffer[4] << 8) | buffer[5];

        int16_t gx_raw = (buffer[6] << 8) | buffer[7];
        int16_t gy_raw = (buffer[8] << 8) | buffer[9];
        int16_t gz_raw = (buffer[10] << 8) | buffer[11];

        //xil_printf("raw data (X,Y,Z): %d, %d, %d  \r\n", ax_raw, ay_raw, az_raw);
        calibrated_gyro_x = (float)gx_raw - gyro_offset_x;
		calibrated_gyro_y = (float)gy_raw - gyro_offset_y;
		calibrated_gyro_z = (float)gz_raw - gyro_offset_z;

        float ax = ax_raw / ACC_SCALE;
        float ay = ay_raw / ACC_SCALE;
        float az = az_raw / ACC_SCALE;

        float gx = calibrated_gyro_x / GYRO_SCALE;
        float gy = calibrated_gyro_y / GYRO_SCALE;
        float gz = calibrated_gyro_z / GYRO_SCALE;

        /* --- Filter yaw with bias compensation --- */
        float dt = (float)IMU_TASK_RATE_MS / 1000.0f;
        float gz_filtered = update_yaw_filtered(gx, gy, gz, ax, ay, az, dt);

        /* --- Complementary Fusion --- */
        roll  = ALPHA * (roll  + gx * dt) + (1 - ALPHA) * (-atan2f(ay, az) * 57.3f) ;
        pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * ( atan2f(ax, sqrtf(ay*ay + az*az)) * 57.3f);
        yaw  += gz_filtered * dt;

        float roll_out  = -(roll  - roll_offset);
		float pitch_out = pitch - pitch_offset;
		float yaw_out   = yaw   - yaw_offset;




		//센서가 부착된 방향 때문에 실제로는 x 가 pitch 값, y가 roll, z가 yaw 값 입니다.
		//roll(우리 케이스에서의 pitch), yaw 값만 출력
		char msg[100];
		snprintf(msg, sizeof(msg),
		         "Angle(pitch,yaw): %.2f, %.2f\r\n", roll_out, yaw_out);

		xil_printf("%s\r", msg);



		/* Output Structure 수정 가능*/
		IMUData_t data = {
			.angle_roll_deg = roll_out,
			.angle_yaw_deg = yaw_out
		};

		//어떤 테스크에서 보낼지에 따라 이부분 지우고 작성하시면 될 것 같습니다.
        xQueueOverwrite(imuQueue, &data);
    }
}

/* ---- Public API ---- */
void IMU_Init(void)
{
    XIicPs_Config *cfg = XIicPs_LookupConfig(IIC_DEVICE_ID);
    XIicPs_CfgInitialize(&Iic, cfg, cfg->BaseAddress);
    XIicPs_SelfTest(&Iic);
    XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);

    uint8_t who = 0;
    uint8_t full_scale = 0;

    /* ---- WHO AM I Check ---- */
    WriteReg(REG_BANK_SEL, USER_BANK_0);    // BANK0 이동
	ReadReg(0x00, &who, 1);  // REG WHO_AM_I = 0x00

	if(who != 0xEA)
	{
		xil_printf("[ICM-20948] WHO_AM_I FAIL (0x%02X) Expected: 0xEA\r\n", who);
		return;
	}
	xil_printf("[ICM-20948] WHO_AM_I OK (0xEA)\r\n");

	//초기화
	WriteReg(UB0_PWR_MGMT_1, 0x01);
	WriteReg(UB0_PWR_MGMT_2, 0x00);
	//vTaskDelay(pdMS_TO_TICKS(20));
	xil_printf("Chapter 1 Okay \r\n");


	WriteReg(REG_BANK_SEL, USER_BANK_2);    // BANK2 이동
	//vTaskDelay(pdMS_TO_TICKS(20));

	WriteReg(UB2_GYRO_CONFIG_1, 0x21);    //내부 LPF 4 / Full Scale 0 (+-250) /FCHOICE 1 (Enable)
	WriteReg(UB2_ACCEL_CONFIG, 0x21);    //내부 LPF 4 / Full Scale 0 (+-2g) /FCHOICE 1 (Enable)
	WriteReg(REG_BANK_SEL, USER_BANK_0);
	WriteReg(GYRO_SMPLRT_DIV, 0x09); 	 //1125 /( 1 + x) hZ 로 ODR 설정
	//vTaskDelay(pdMS_TO_TICKS(20));
	xil_printf("Chapter 2 Okay \r\n");

	ReadReg(UB2_GYRO_CONFIG_1, &full_scale, 1);
	u8 fs_sel = (full_scale >> 1) & 0x03;
	xil_printf("%d\r\n", full_scale);

    imuQueue = xQueueCreate(1, sizeof(IMUData_t));
    IMU_Calibrate();

    xil_printf("[IMU] Init Complete.\r\n");
}

void IMU_StartTask(void)
{
    xTaskCreate(IMUTask, "IMU", 2048, NULL, 5, NULL);
}

QueueHandle_t IMU_GetQueue(void)
{
    return imuQueue;
}


void IMU_Calibrate(void)
{
    uint8_t buffer[12];
    xil_printf("\n[IMU] Calibration Start... Keep sensor still!\r\n");

    /* ------------------------ 1) Gyro Offset Calibration ------------------------ */
    gyro_offset_x = gyro_offset_y = gyro_offset_z = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        ReadReg(REG_GYRO_XOUT_H, buffer, 6);
        int16_t gx_raw = (buffer[0] << 8) | buffer[1];
        int16_t gy_raw = (buffer[2] << 8) | buffer[3];
        int16_t gz_raw = (buffer[4] << 8) | buffer[5];

        gyro_offset_x += gx_raw;
        gyro_offset_y += gy_raw;
        gyro_offset_z += gz_raw;

        usleep(10000); // 5ms delay
    }

    gyro_offset_x /= CALIBRATION_SAMPLES;
    gyro_offset_y /= CALIBRATION_SAMPLES;
    gyro_offset_z /= CALIBRATION_SAMPLES;

    /* ------------------------ 2) Angle Offset Calibration ------------------------ */

    float roll_acc = 0, pitch_acc = 0, yaw_acc = 0;

    for(int i = 0; i < ANGLE_SETTLE_SAMPLES; i++)
    {
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

        usleep(10000);
    }

    roll_offset  = roll_acc  / ANGLE_SETTLE_SAMPLES;
    pitch_offset = pitch_acc / ANGLE_SETTLE_SAMPLES;
    yaw_offset   = yaw; // yaw는 처음엔 보통 0

    angle_calibrated = true;

    xil_printf("[IMU] Angle Offset: roll0=%.2f pitch0=%.2f yaw0=%.2f\r\n",
               roll_offset, pitch_offset, yaw_offset);

    xil_printf("[IMU] Calibration Completed.\n\n");
}
