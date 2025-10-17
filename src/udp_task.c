/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * All rights reserved.
 * ... (라이선스 헤더는 동일) ...
 */




#include "udp_task.h" // 실제 파일명에 맞게 유지

#include <stdio.h>  // sscanf, snprintf 를 위해 추가
#include <string.h> // memset, strlen 를 위해 추가
#include <sleep.h>
#include "netif/xadapter.h"
#include "platform_config.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#include "pid.h" 			//PID 헤더파일
#include "ibvs_calculate.h" //IBVS 헤더파일
#include "xtime_l.h"		//시간 측정 라이브러리

#define DEFAULT_IP_ADDRESS "192.168.1.10"
#define DEFAULT_IP_MASK "255.255.255.0"
#define DEFAULT_GW_ADDRESS "192.168.1.1"
#define MAX_BUFFER_LEN 256
extern struct netif server_netif;

// 수신 및 송신을 위한 버퍼의 최대 크기를 정의
struct netif server_netif;
static int complete_nw_thread;
#define THREAD_STACKSIZE 1024

//IBVS 관련 상수
float tx_image_coords[2];	// 이미지 좌표
float velocity_out[2];		// IBVS 계산 값 저장 배열
int ibvs_e;

//PID 생성
PID_Config_t pid_set;
PID_t pid_yaw;
PID_t pid_pitch;
float yaw_deg;
float pitch_deg;

//시간 측정용 변수 설정
XTime t_start, t_end;
double elapsed_us;

//디버깅용
char buf[100];	//제어 알고리즘 디버깅용 버퍼



#ifdef XPS_BOARD_ZCU102
#if defined(XPAR_XIICPS_0_DEVICE_ID) || defined(XPAR_XIICPS_0_BASEADDR)
int IicPhyReset(void);
#endif
#endif


static void print_ip(char *msg, ip_addr_t *ip)
{
	xil_printf(msg);
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
				ip4_addr3(ip), ip4_addr4(ip));
}

static void print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	print_ip("Board IP:       ", ip);
	print_ip("Netmask :       ", mask);
	print_ip("Gateway :       ", gw);
}

static void assign_default_ip(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	int err;

	xil_printf("Configuring default IP %s \r\n", DEFAULT_IP_ADDRESS);

	err = inet_aton(DEFAULT_IP_ADDRESS, ip);
	if(!err)
		xil_printf("Invalid default IP address: %d\r\n", err);

	err = inet_aton(DEFAULT_IP_MASK, mask);
	if(!err)
		xil_printf("Invalid default IP MASK: %d\r\n", err);

	err = inet_aton(DEFAULT_GW_ADDRESS, gw);
	if(!err)
		xil_printf("Invalid default gateway address: %d\r\n", err);
}


void network_thread(void *p)
{
	/* the mac address of the board. this should be unique per board */
	u8_t mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(&server_netif, NULL, NULL, NULL, mac_ethernet_address,
		PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\r\n");
		return;
	}

	netif_set_default(&server_netif);

	/* specify that the network if is up */
	netif_set_up(&server_netif);

	/* start packet receive thread - required for lwIP operation */
	sys_thread_new("xemacif_input_thread",
			(void(*)(void*))xemacif_input_thread, &server_netif,
			THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	complete_nw_thread = 1;


	vTaskDelete(NULL);
}


int udp_thread()
{

#ifdef XPS_BOARD_ZCU102
	IicPhyReset();
#endif
	xil_printf("\n\r\n\r");
	xil_printf("Zynq board Booting successfully\r\n");
	xil_printf("-----lwIP Socket Mode UDP Client Application------\r\n");

	/* initialize lwIP before calling sys_thread_new */
	lwip_init();
	HILS_init();	//HILS관련 초기화 함수

	/* any thread using lwIP should be created using sys_thread_new */
	sys_thread_new("nw_thread", network_thread, NULL,
			THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	while(!complete_nw_thread)
		usleep(50);


	assign_default_ip(&(server_netif.ip_addr), &(server_netif.netmask),
				&(server_netif.gw));

	print_ip_settings(&(server_netif.ip_addr), &(server_netif.netmask),
				&(server_netif.gw));
	xil_printf("\r\n");

	/* print all application headers */
	print_app_header();
	xil_printf("\r\n");

	/* start the application*/
	start_application();

	vTaskDelete(NULL);
	return 0;
}


//PID초기화 함수
void HILS_init(void){
	//PID 게인 설정
	pid_config_init(&pid_set, 1.0, 0.0, 0.00, 35, -35);

	//pid_init(PID, pid_set);
	pid_init(&pid_pitch, &pid_set);
	pid_init(&pid_yaw,&pid_set);
}

void print_app_header(void)
{
    xil_printf("------------------------------------------------------\r\n");
    xil_printf("         UDP String Parser/Echo Application\r\n");
    xil_printf("------------------------------------------------------\r\n");
    xil_printf("Listening on port %d for formatted string data\r\n", UDP_CONN_PORT);
    xil_printf("Waiting for data from client...\r\n");
}

void start_application(void)
{
    int sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    int n;

    // 수신/송신을 위한 버퍼 선언
    char recv_buffer[MAX_BUFFER_LEN];
    char send_buffer[MAX_BUFFER_LEN];

    // 1. UDP 소켓 생성
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        xil_printf("Error: Could not create socket\r\n");
        return;
    }

    // 2. 서버 주소 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_CONN_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    // 3. 소켓에 주소 할당 (바인딩)
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        xil_printf("Error: Failed to bind socket\r\n");
        close(sock);
        return;
    }

    TickType_t xLastWakeTime;

    // 4. 무한 루프를 돌며 데이터 수신 및 응답
    while (1) {
    	xLastWakeTime = xTaskGetTickCount();
        n = recvfrom(sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
                     (struct sockaddr *)&client_addr, &client_addr_len);

        if (n < 0) {
            xil_printf("Error: recvfrom failed\r\n");
            continue;
        }

        // 수신된 데이터의 끝에 Null 문자를 추가하여 C-스타일 문자열로 만듭니다.
        recv_buffer[n] = '\0';
        xil_printf("\nReceived raw string: \"%s\"\r\n", recv_buffer);

        // 파싱된 데이터를 저장할 변수들
        float p_val, y_val ;
        float x_coord, y_coord;


        // sscanf를 사용하여 특정 형식의 문자열을 파싱합니다.
        // 성공적으로 4개의 항목을 모두 읽으면 4를 반환합니다.
        int items_scanned = sscanf(recv_buffer, "P: %f Y: %f x: %f y: %f",
                                   &p_val, &y_val, &x_coord, &y_coord);

        // 이미지 좌표
        tx_image_coords[0] = x_coord;
        tx_image_coords[1] = y_coord;


        // [핵심 로직] 파싱이 성공했는지 확인
        if (items_scanned == 4) {
            // 파싱 성공!
            xil_printf("Parsing successful.\r\n");

            XTime_GetTime(&t_start);
            //제어 알고리즘 구현 부분
            //IBVS 실행
            ibvs_e = IBVS_calculation(velocity_out, x_coord, y_coord);
            if(ibvs_e < 0 )continue;

            snprintf(buf, sizeof(buf), "omega_pitch: %.4f    omega_yaw: %.4f ",
                                        velocity_out[0],velocity_out[1]);
            xil_printf("After IBVS, %s\r\n", buf);

            //PID 구현
            pitch_deg = pid_calculation(&pid_pitch, velocity_out[0], p_val);
            yaw_deg = pid_calculation(&pid_yaw, velocity_out[1], y_val);

            //snprintf(buf, sizeof(buf), "p_error:%.4f  Y_error:%.4f P_deg: %.4f Y_deg: %.4f "
            //							,pid_pitch.pre_e, pid_yaw.pre_e, pitch_deg, yaw_deg);

            matrix_scalar_mul((float)(t_end - t_start) / (float)COUNTS_PER_SECOND, velocity_out, 2, 1);
            snprintf(buf, sizeof(buf), " P_deg: %.4f Y_deg: %.4f "
                 							 , pitch_deg, yaw_deg);
            xil_printf("IBVS-> deg : %s\r\n", buf);
            // snprintf를 사용하여 응답 문자열을 다시 만듭니다.
            // %.1f는 소수점 첫째 자리까지만 표시하도록 합니다.
            snprintf(send_buffer, sizeof(send_buffer), "P: %.5f Y: %.5f x: %.5f y: %.5f",
            		pitch_deg, yaw_deg, x_coord, y_coord);

            // 처리된 문자열을 클라이언트에게 다시 보냅니다.
            sendto(sock, send_buffer, strlen(send_buffer), 0,
                   (struct sockaddr *)&client_addr, client_addr_len);

            xil_printf("Sent back modified string: \"%s\"\r\n", send_buffer);

        } else {
            // 파싱 실패! 수신된 문자열의 형식이 예상과 다릅니다.
            xil_printf("Warning: Received string does not match expected format.\r\n");

            // 에러 메시지를 클라이언트에게 보낼 수도 있습니다.
            const char *error_msg = "Error: Invalid string format received by Zynq.";
            sendto(sock, error_msg, strlen(error_msg), 0,
                   (struct sockaddr *)&client_addr, client_addr_len);
        }
        XTime_GetTime(&t_end);

        //시간 계산
        elapsed_us = 1e6 * (double)(t_end - t_start) / (double)COUNTS_PER_SECOND;
        u32 elapsed_int_us = (u32)elapsed_us;
        u32 elapsed_int_ms = elapsed_int_us / 1000;

        xil_printf("Processing+Send time: %u us (~%u ms)\r\n",elapsed_int_us, elapsed_int_ms);

        //vTaskDelayUntil(&xLastWakeTime, 20);
    }

    close(sock);
}
