// File: motor/motor_driver_task.c
// (기존 motor/control_task.c)

#include "motor_driver_task.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "task.h"

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

static u32 angle_to_pulse_counts(float angle)
{
    // 1. 입력 범위를 -90.0 ~ +90.0으로 제한
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;

    // 2. 입력된 각도를 0 ~ 180 범위로 '반대로' 변환합니다.
    float mapped_angle = 90.0f - angle;

    // 3. 변환된 값을 사용하여 펄스 폭(500us ~ 2500us)을 계산합니다.
    float pulse_us = 500.0f + (mapped_angle * 2000.0f) / 180.0f;

    return (u32)((float)CPU_FREQ_HZ / 1000000.0f * pulse_us);
}

void vMotorDriverTask(void *pvParameters)
{

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

    while(1)
    {
        // 큐에서 (모터 ID + 각도)가 담긴 구조체를 수신
        if (xQueueReceive(xAngleQueue, &received_cmd, portMAX_DELAY) == pdPASS)
        {
            // 유효한 motor_id인지 확인 (0~5)
            if (received_cmd.motor_id >= 0 && received_cmd.motor_id < 6)
            {
                u32 pulse_counts = angle_to_pulse_counts(received_cmd.angle);

                // 해당 ID의 모터(타이머) 레지스터에만 값을 씀
                Xil_Out32(TIMER_BASE_ADDRS[received_cmd.motor_id] + TLR1_OFFSET, pulse_counts);

            }
        }
    }
}
