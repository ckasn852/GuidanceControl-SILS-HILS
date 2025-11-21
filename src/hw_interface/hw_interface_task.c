#include "hw_interface_task.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include "../control/control_data.h"
#include "../control/control_output_hw.h"
#include "../imu/imu.h"

// 태스크 시간 측정
#include "xtime_l.h"
#include "../sys_stat/system_stats.h"

extern SemaphoreHandle_t printMutex;
extern QueueHandle_t xRealDataQueue;

// 6개의 타이머 베이스 주소를 배열로 정의
const u32 TIMER_BASE_ADDRS[6] = {
    XPAR_AXI_TIMER_0_BASEADDR,
    XPAR_AXI_TIMER_1_BASEADDR,
    XPAR_AXI_TIMER_2_BASEADDR,
    XPAR_AXI_TIMER_3_BASEADDR,
    XPAR_AXI_TIMER_4_BASEADDR,
    XPAR_AXI_TIMER_5_BASEADDR
};

#define TCSR0_OFFSET        0x00
#define TLR0_OFFSET         0x04
#define TCSR1_OFFSET        0x10
#define TLR1_OFFSET         0x14

#define PWMA_BIT            9
#define GENT_BIT            2
#define UDT_BIT             1
#define ARHT_BIT            4
#define ENT_BIT             7

#define SET_BIT(val, bit) ((val) | (1 << (bit)))

#define CPU_FREQ_HZ         50000000
#define PWM_PERIOD_US       20000

// 하드웨어 읽기용 스텁 함수
static float read_real_motor_angle(int motor_id) { return 0.0f; }

static u32 angle_to_pulse_counts(float angle)
{
    // 1. 입력 범위를 -90.0 ~ +90.0으로 제한
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;

    // 2. 입력된 각도를 0 ~ 180 범위로 '반대로' 변환
    float mapped_angle = 90.0f - angle;

    // 3. 변환된 값을 사용하여 펄스 폭(500us ~ 2500us) 계산
    float pulse_us = 500.0f + (mapped_angle * 2000.0f) / 180.0f;

    return (u32)((float)CPU_FREQ_HZ / 1000000.0f * pulse_us);
}

void vHwInterfaceTask(void *pvParameters)
{
	// 태스크 수행시간 측정용 변수
    XTime tStart, tEnd;

    u32 tcsr0_val = 0;
    u32 tcsr1_val = 0;

    tcsr0_val = SET_BIT(tcsr0_val, PWMA_BIT) | SET_BIT(tcsr0_val, GENT_BIT) |
                SET_BIT(tcsr0_val, UDT_BIT)  | SET_BIT(tcsr0_val, ARHT_BIT) |
                SET_BIT(tcsr0_val, ENT_BIT);
    tcsr1_val = tcsr0_val;

    u32 period_counts = (CPU_FREQ_HZ / 1000000) * PWM_PERIOD_US;

    // 6개의 타이머 모두 0도(중앙)로 초기화
    for (int i = 0; i < 6; i++) {
        Xil_Out32(TIMER_BASE_ADDRS[i] + TLR0_OFFSET, period_counts);
        Xil_Out32(TIMER_BASE_ADDRS[i] + TCSR0_OFFSET, tcsr0_val);

        u32 initial_pulse_counts = angle_to_pulse_counts(0.0f);
        Xil_Out32(TIMER_BASE_ADDRS[i] + TLR1_OFFSET, initial_pulse_counts);
        Xil_Out32(TIMER_BASE_ADDRS[i] + TCSR1_OFFSET, tcsr1_val);
    }

    ServoCommand_t received_cmd;
    static real_sensor_data current_real_data = {0};
    IMUData_t imu_temp;

    while(1)
    {
        // 큐에서 (모터 ID + 각도)가 담긴 구조체를 수신
        if (xQueueReceive(xAngleQueue, &received_cmd, portMAX_DELAY) == pdPASS)
        {
        	/* 시간 측정 시작 */
            XTime_GetTime(&tStart);

            // 유효한 motor_id인지 확인 (0~5)
            if (received_cmd.motor_id >= 0 && received_cmd.motor_id < 6)
            {
                u32 pulse_counts = angle_to_pulse_counts(received_cmd.angle);

                // 해당 ID의 모터(타이머) 레지스터에만 값을 씀
                Xil_Out32(TIMER_BASE_ADDRS[received_cmd.motor_id] + TLR1_OFFSET, pulse_counts);

                // 실제 모터 피드백 읽기
                current_real_data.real_motor_angle[received_cmd.motor_id] =
                                read_real_motor_angle(received_cmd.motor_id);

                // IMU 센서 읽기 (마지막 모터 ID 처리 시 수행)
                if (received_cmd.motor_id == 5) {
                    IMU_Update(0.02f);
                    imu_temp = IMU_GetData();

                    // [수정] Roll 데이터를 Pitch로 매핑 (센서 부착 방향 반영)
                    current_real_data.real_imu_pitch = imu_temp.angle_roll_deg;
                    current_real_data.real_imu_yaw   = imu_temp.angle_yaw_deg;

                    // Tx Task로 모든 하드웨어 데이터 전송 (최신 데이터 유지를 위해 Overwrite)
                    if (xRealDataQueue) {
                        xQueueOverwrite(xRealDataQueue, &current_real_data);
                    }
                }

                /* 시간 측정 종료 */
				XTime_GetTime(&tEnd);

				// 매번 혹은 5번 모터일 때만 업데이트
				g_time_hw_us = (float)((double)(tEnd - tStart) / (COUNTS_PER_SECOND / 1000000.0));
            }
        }
    }
}
