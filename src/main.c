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

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "control/control_queue.h"

#define TASK_STACKSIZE  1024

SemaphoreHandle_t networkInitMutex;
SemaphoreHandle_t printMutex;

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
    xil_printf("\n\r\n\r");
    xil_printf("Zynq board Boot Successful\r\n");
    xil_printf("-----lwIP Socket Mode UDP Client Application------\r\n");

    networkInitMutex = xSemaphoreCreateMutex();		// 네트워크 초기화 뮤텍스 생성
    printMutex = xSemaphoreCreateMutex();			// 프린트 뮤텍스 생성

    // 제어 데이터 큐 초기화
    control_queue_init();

    // 네트워크 초기화 태스크 생성
    xTaskCreate(network_init_task,
				"network_init_task",
				TASK_STACKSIZE,
				NULL,
				tskIDLE_PRIORITY+4,
				NULL);

    // 제어 태스크 생성
	xTaskCreate(control_task,
				"control_task",
				TASK_STACKSIZE,
				NULL,
				tskIDLE_PRIORITY+3,
				NULL);

    // 수신 태스크 생성
	xTaskCreate(rx_task,
				"rx_task",
				TASK_STACKSIZE,
				NULL,
				tskIDLE_PRIORITY+2,
				NULL);


    // 송신 태스크 생성
    xTaskCreate(tx_task,
				"tx_task",
				TASK_STACKSIZE,
				NULL,
				tskIDLE_PRIORITY+1,
				NULL);


	//scheduling 시작
	vTaskStartScheduler();
	while(1);
	return 0;
}
