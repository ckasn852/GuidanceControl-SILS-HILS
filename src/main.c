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
#include "platform_config.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "lwip/inet.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "udp_task.h"
#include "servo_task.h"

#ifdef XPS_BOARD_ZCU102
#if defined(XPAR_XIICPS_0_DEVICE_ID) || defined(XPAR_XIICPS_0_BASEADDR)
int IicPhyReset(void);
#endif
#endif

void print_app_header();
void start_application();
int udp_thread();

#define THREAD_STACKSIZE 1024


#define UDP_TASK_PRIORITY   ( tskIDLE_PRIORITY + 2 )
#define SERVO_TASK_PRIORITY ( tskIDLE_PRIORITY + 3 )


QueueHandle_t xAngleQueue = NULL;


int main()
{
	xil_printf("--- System Booting ---\r\n");

	/* ================================================================== */
	/* 1. 큐(Queue) 생성 */
	/* ================================================================== */
	// ServoCommand_t 타입의 데이터를 10개까지 저장할 수 있는 큐 생성
	xAngleQueue = xQueueCreate(10, sizeof(ServoCommand_t));

	if (xAngleQueue == NULL) {
		xil_printf("Fatal Error: Failed to create xAngleQueue!\r\n");
		while(1);
	}
	xil_printf("xAngleQueue created successfully.\r\n");


	/* ================================================================== */
	/* 2. 태스크(Task) 생성 */
	/* ================================================================== */

	// 2-1. 서보 모터 제어 태스크 생성
	xTaskCreate(vServoControlTask,           /* 태스크 함수 포인터 */
				"ServoTask",                 /* 태스크 이름 (디버깅용) */
				configMINIMAL_STACK_SIZE,    /* 스택 크기 (필요시 증가) */
				NULL,                        /* 태스크 파라미터 */
				SERVO_TASK_PRIORITY,         /* 태스크 우선순위 */
				NULL);                       /* 태스크 핸들 (필요 없음) */

	xil_printf("vServoControlTask created.\r\n");


	// 2-2. UDP 통신 태스크 생성 (기존 main_thread의 내용)
	sys_thread_new("udp_thread",             /* 태스크 이름 */
				   (void(*)(void*))udp_thread, /* 태스크 함수 */
				   NULL,                     /* 파라미터 */
				   THREAD_STACKSIZE,         /* 스택 크기 */
				   UDP_TASK_PRIORITY);       /* 태스크 우선순위 */

	xil_printf("udp_thread created.\r\n");


	/* ================================================================== */
	/* 3. FreeRTOS 스케줄러 시작 */
	/* ================================================================== */
	xil_printf("Starting FreeRTOS scheduler...\r\n");

	// scheduling 시작
	vTaskStartScheduler();

	while(1);
	return 0;
}
