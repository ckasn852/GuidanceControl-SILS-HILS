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

#define DEFAULT_IP_ADDRESS "192.168.1.10"
#define DEFAULT_IP_MASK "255.255.255.0"
#define DEFAULT_GW_ADDRESS "192.168.1.1"
#define MAX_BUFFER_LEN 256
extern struct netif server_netif;
// 수신 및 송신을 위한 버퍼의 최대 크기를 정의

struct netif server_netif;
static int complete_nw_thread;
#define THREAD_STACKSIZE 1024

//PID 생성
PID_t pid_yaw;
PID_t pid_pitch;

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

//여기까지

//PID초기화 함수
void HILS_init(void){
	//pid_init(PID, Kp, Ki, Kd, out_max, out_min);
	pid_init(&pid_yaw,0.1, 0.0, 0.0, 20, -20);
	pid_init(&pid_pitch,0.1, 0.0, 0.0, 20, -20);
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

    // 4. 무한 루프를 돌며 데이터 수신 및 응답
    while (1) {
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
        float p_val, y_val;
        int x_coord, y_coord;

        //디버깅용 타겟 속도
        float target_p = -18;
        float target_y = 10;

        // sscanf를 사용하여 특정 형식의 문자열을 파싱합니다.
        // 성공적으로 4개의 항목을 모두 읽으면 4를 반환합니다.
        int items_scanned = sscanf(recv_buffer, "P: %f Y: %f x: %d y: %d",
                                   &p_val, &y_val, &x_coord, &y_coord);

        // [핵심 로직] 파싱이 성공했는지 확인
        if (items_scanned == 4) {
            // 파싱 성공!
            xil_printf("Parsing successful. Original Y value: %.1f\r\n", y_val);
            //제어 알고리즘 구현 부분
            //PID 구현
            y_val = pid_calculation(&pid_yaw, target_y, y_val);  //target*1.5 =3
            p_val = pid_calculation(&pid_pitch, target_p, p_val);
            char buf[32];
            snprintf(buf, sizeof(buf), "P: %.1f Y: %.1f ",
                                 p_val,y_val);
            xil_printf("After PID : %s\r\n", buf);
            
            // Y 값에 1.0을 더합니다.
            y_val += 1.0;

            // snprintf를 사용하여 응답 문자열을 다시 만듭니다.
            // %.1f는 소수점 첫째 자리까지만 표시하도록 합니다.
            snprintf(send_buffer, sizeof(send_buffer), "P: %.1f Y: %.1f x: %d y: %d",
                     p_val, y_val, x_coord, y_coord);

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
    }

    close(sock);
}
