#include <sleep.h>
#include "netif/xadapter.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#include "rx/rx_task.h"
#include "tx/tx_task.h"
#include "imu/imu.h"
#include "control/control_task.h"
#include "network_init/network_bootstrap.h"

// 하드 타이머/GIC 헤더
#include "xparameters.h"
#include "xparameters_ps.h"
#include "xscugic.h"
#include "xttcps.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "control/control_queue.h"

// 모터/튜닝 헤더 추가
#include "motor/motor_driver_task.h"
#include "control/pid_tuning_mode.h"

// 태스크 스택 사이즈
#define NETINIT_STACKSIZE  1024
#define RX_STACKSIZE       1024
#define CONTROL_STACKSIZE  1024
#define TX_STACKSIZE       1024
#define MOTOR_STACKSIZE    1024
#define IMU_STACKSIZE       1024

#define PRIO_NETINIT (tskIDLE_PRIORITY+4)
#define PRIO_RX      (tskIDLE_PRIORITY+2)
#define PRIO_CONTROL (tskIDLE_PRIORITY+5)
#define PRIO_TX      (tskIDLE_PRIORITY+2)
#define PRIO_MOTOR   (tskIDLE_PRIORITY+2)
#define PRIO_IMU      (tskIDLE_PRIORITY+3) //상황에 맞춰 수정

// IRQ ID/디바이스 ID
#define TTC_DEV_ID      XPAR_XTTCPS_0_DEVICE_ID
#define TTC_IRQ_ID      XPAR_XTTCPS_0_INTR

SemaphoreHandle_t networkInitMutex;
SemaphoreHandle_t printMutex;

// 외부에서 참조하는 큐 전역 생성
QueueHandle_t xAngleQueue   = NULL;
QueueHandle_t xAngleFbQueue = NULL;
QueueHandle_t xPidUpdateQueue = NULL;

// 전역 인스턴스/핸들
static XTtcPs  g_Ttc;
TaskHandle_t gControlTaskHandle = NULL;
static volatile uint32_t g_isr_cnt = 0;

// FreeRTOS 인터럽트 설치 API 프로토타입
BaseType_t xPortInstallInterruptHandler(uint8_t ucInterruptID, Xil_InterruptHandler pxHandler, void *pvCallBackRef);
void vPortEnableInterrupt(uint8_t ucInterruptID);


// TTC ISR: 20ms마다 Control Task 깨움
static void Ttc_Isr(void *cb)
{
    XTtcPs *Ttc = (XTtcPs*)cb;
    u32 stat = XTtcPs_GetInterruptStatus(Ttc);
    XTtcPs_ClearInterruptStatus(Ttc, stat);
    g_isr_cnt++;

    BaseType_t xHigherWoken = pdFALSE;

    // Control Task에게 알림(Notify) 전송 -> Task 깨움
    vTaskNotifyGiveFromISR(gControlTaskHandle, &xHigherWoken);

    // Context Switching 요청 (Task 전환)
    portYIELD_FROM_ISR(xHigherWoken);
}

// TTC 50Hz 초기화/시작 함수
static int ttc_start_50hz(void)
{
    int Status;
    XTtcPs_Config *Cfg = XTtcPs_LookupConfig(TTC_DEV_ID);
    if (!Cfg) return -1;

    Status = XTtcPs_CfgInitialize(&g_Ttc, Cfg, Cfg->BaseAddress);
    if (Status != XST_SUCCESS) return -2;

    XTtcPs_DisableInterrupts(&g_Ttc, XTTCPS_IXR_ALL_MASK);
    XTtcPs_ClearInterruptStatus(&g_Ttc, XTTCPS_IXR_ALL_MASK);

    // Interval Mode 설정
    XTtcPs_SetOptions(&g_Ttc, XTTCPS_OPTION_INTERVAL_MODE | XTTCPS_OPTION_WAVE_POLARITY);

    XInterval interval;
    u8        prescaler;

    // 50Hz (20ms) 주기 계산
    XTtcPs_CalcIntervalFromFreq(&g_Ttc, 50, &interval, &prescaler);
    XTtcPs_SetPrescaler(&g_Ttc, prescaler);
    XTtcPs_SetInterval(&g_Ttc, interval);

    // FreeRTOS가 관리하는 GIC 인스턴스에 핸들러 연결
    Status = xPortInstallInterruptHandler(TTC_IRQ_ID, (Xil_InterruptHandler)Ttc_Isr, &g_Ttc);
    if (Status != pdPASS) {
        xil_printf("[ERR] TTC Interrupt Install Failed\r\n");
        return -5;
    }

    // FreeRTOS API를 통해 인터럽트 활성화
    vPortEnableInterrupt(TTC_IRQ_ID);
    XTtcPs_EnableInterrupts(&g_Ttc, XTTCPS_IXR_INTERVAL_MASK);
    XTtcPs_Start(&g_Ttc);

    return 0;
}

static void network_init_task(void *arg)
{
    (void)arg; // 안 쓰면 경고 방지

    if (xSemaphoreTake(networkInitMutex, portMAX_DELAY) != pdTRUE) {
        xil_printf("network_init_task: mutex take fail\r\n");
        vTaskDelete(NULL);
        return;
    }

    // Zynq 보드 네트워크 설정 초기화
    network_bootstrap();

    // 초기화 완료
    xSemaphoreGive(networkInitMutex);
    // 여기서 부터 rx_task 같은 태스크가 네트워크 사용 가능

    // 설정 초기화 되면 태스크 삭제
    vTaskDelete(NULL);
}

int main()
{
    networkInitMutex = xSemaphoreCreateMutex();     // 네트워크 초기화 뮤텍스 생성
    printMutex = xSemaphoreCreateMutex();           // 프린트 뮤텍스 생성

    IMU_Init();

    // 제어 데이터 큐 초기화 (최신 데이터만 사용하기 위해 길이 1 설정)
    control_queue_init();

    // 큐 생성
    xAngleQueue     = xQueueCreate(1, sizeof(ServoCommand_t));
    xAngleFbQueue   = xQueueCreate(1, sizeof(ServoFeedback_t));
    xPidUpdateQueue = xQueueCreate(4, sizeof(PIDUpdate_t));

    // 큐 생성 결과 출력
    if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
        printf("[main] xAngleQueue=%p xAngleFbQueue=%p xPidUpdateQueue=%p\r\n",
               (void*)xAngleQueue, (void*)xAngleFbQueue, (void*)xPidUpdateQueue);
        xSemaphoreGive(printMutex);
    }

    // 네트워크 초기화 태스크 생성
    xTaskCreate(network_init_task, "network_init_task", NETINIT_STACKSIZE, NULL, PRIO_NETINIT, NULL);

    // 수신 태스크 생성
    xTaskCreate(rx_task, "rx_task", RX_STACKSIZE, NULL, PRIO_RX, NULL);

    // 제어 태스크 생성 (핸들 저장 -> TTC ISR에서 사용)
    xTaskCreate(control_task, "control_task", CONTROL_STACKSIZE, NULL, PRIO_CONTROL, &gControlTaskHandle);

    // 모터 드라이버 태스크 생성
    xTaskCreate(vMotorDriverTask, "motor_driver_task", MOTOR_STACKSIZE, NULL, PRIO_MOTOR, NULL);

    // 송신 태스크 생성
    xTaskCreate(tx_task, "tx_task", TX_STACKSIZE, NULL, PRIO_TX, NULL);

    //IMU 태스크 생성
    xTaskCreate(IMUTask, "IMU", 2048, IMU_STACKSIZE, PRIO_IMU, NULL);

    // 하드 타이머(50Hz) 시작은 스케줄러 시작 '직전'에 수행
    // FreeRTOS 핸들러 등록을 위해 task 생성 후, scheduler 시작 전에 호출
    int rc = ttc_start_50hz();
    if (rc != 0) {
        if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
            printf("[main][ERR] TTC start failed rc=%d\r\n", rc);
            xSemaphoreGive(printMutex);
        }
    } else {
        printf("[main] TTC 50Hz Started Successfully.\r\n");
    }

    // scheduling 시작
    vTaskStartScheduler();

    while(1);
    return 0;
}
