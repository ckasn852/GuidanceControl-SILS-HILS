/*
 * Copyright (C) 2017 - 2022 Xilinx, Inc.
 * Copyright (C) 2022 - 2023 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

#include <sleep.h>
#include "netif/xadapter.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#include "rx/rx_task.h"
#include "tx/tx_task.h"
#include "control/control_task.h"
#include "network_init/network_bootstrap.h"
#include "control/pid_tuning_mode.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "control/control_queue.h"

// 태스크 스택/우선순위 상향 및 정렬 (주소 관련 없음)
#define NETINIT_STACKSIZE  1024
#define RX_STACKSIZE       1024
#define CONTROL_STACKSIZE  1024
#define TX_STACKSIZE       1024

#define PRIO_NETINIT (tskIDLE_PRIORITY+4)
#define PRIO_RX      (tskIDLE_PRIORITY+1)
#define PRIO_CONTROL (tskIDLE_PRIORITY+2)
#define PRIO_TX      (tskIDLE_PRIORITY+3)

SemaphoreHandle_t networkInitMutex;
SemaphoreHandle_t printMutex;
QueueHandle_t xPidUpdateQueue = NULL;

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

    // 초기화 완료 -> 이제 rx_task 같은 애들이 네트워크 사용해도 됨
    xSemaphoreGive(networkInitMutex);

    // 설정 초기화 되면 태스크 삭제
    vTaskDelete(NULL);
}

int main()
{
    networkInitMutex = xSemaphoreCreateMutex();     // 네트워크 초기화 뮤텍스 생성
    printMutex = xSemaphoreCreateMutex();           // 프린트 뮤텍스 생성

    // 제어 데이터 큐 초기화 (최신 데이터만 사용하기 위해 길이 1 설정)
    control_queue_init();

    // PID 게인 튜닝 큐 초기화
    xPidUpdateQueue = xQueueCreate(4, sizeof(PIDUpdate_t));

    // 네트워크 초기화 태스크 생성
    xTaskCreate(network_init_task, "network_init_task", NETINIT_STACKSIZE, NULL, PRIO_NETINIT, NULL);

    // 수신 태스크 생성
    xTaskCreate(rx_task, "rx_task", RX_STACKSIZE, NULL, PRIO_RX, NULL);

    // 제어 태스크 생성
    xTaskCreate(control_task, "control_task", CONTROL_STACKSIZE, NULL, PRIO_CONTROL, NULL);

    // 송신 태스크 생성
    xTaskCreate(tx_task, "tx_task", TX_STACKSIZE, NULL, PRIO_TX, NULL);

    //scheduling 시작
    vTaskStartScheduler();
    while(1);
    return 0;
}
