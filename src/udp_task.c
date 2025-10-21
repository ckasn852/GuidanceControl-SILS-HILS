/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * All rights reserved.
 * ... (�씪�씠�꽑�뒪 �뿤�뜑�뒗 �룞�씪) ...
 */




#include "udp_task.h" // �떎�젣 �뙆�씪紐낆뿉 留욊쾶 �쑀吏�

#include <stdio.h>  // sscanf, snprintf 瑜� �쐞�빐 異붽�
#include <string.h> // memset, strlen 瑜� �쐞�빐 異붽�
#include <sleep.h>
#include "netif/xadapter.h"
#include "platform_config.h"
#include "xil_printf.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#include "pid.h" 			//PID �뿤�뜑�뙆�씪
#include "ibvs_calculate.h" //IBVS �뿤�뜑�뙆�씪
#include "xtime_l.h"		//�떆媛� 痢≪젙 �씪�씠釉뚮윭由�


#include "FreeRTOS.h"
#include "queue.h"
#include "servo_task.h"

#define DEFAULT_IP_ADDRESS "192.168.1.10"
#define DEFAULT_IP_MASK "255.255.255.0"
#define DEFAULT_GW_ADDRESS "192.168.1.1"
#define MAX_BUFFER_LEN 256
extern struct netif server_netif;

// �닔�떊 諛� �넚�떊�쓣 �쐞�븳 踰꾪띁�쓽 理쒕� �겕湲곕�� �젙�쓽
struct netif server_netif;
static int complete_nw_thread;
#define THREAD_STACKSIZE 1024

//IBVS 愿��젴 �긽�닔
float tx_image_coords[2];	// �씠誘몄� 醫뚰몴
float velocity_out[2];		// IBVS 怨꾩궛 媛� ���옣 諛곗뿴
int ibvs_e;

//PID �깮�꽦
PID_Config_t pid_set;
PID_t pid_yaw;
PID_t pid_pitch;
float yaw_deg;
float pitch_deg;

//�떆媛� 痢≪젙�슜 蹂��닔 �꽕�젙
XTime t_start, t_end;
double elapsed_us;

//�뵒踰꾧퉭�슜
char buf[100];	//�젣�뼱 �븣怨좊━利� �뵒踰꾧퉭�슜 踰꾪띁



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
	HILS_init();	//HILS愿��젴 珥덇린�솕 �븿�닔

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


//PID珥덇린�솕 �븿�닔
void HILS_init(void){
	//PID 寃뚯씤 �꽕�젙
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

    // �닔�떊/�넚�떊�쓣 �쐞�븳 踰꾪띁 �꽑�뼵
    char recv_buffer[MAX_BUFFER_LEN];
    char send_buffer[MAX_BUFFER_LEN];

    // 1. UDP �냼耳� �깮�꽦
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        xil_printf("Error: Could not create socket\r\n");
        return;
    }

    // 2. �꽌踰� 二쇱냼 �꽕�젙
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_CONN_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    // 3. �냼耳볦뿉 二쇱냼 �븷�떦 (諛붿씤�뵫)
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        xil_printf("Error: Failed to bind socket\r\n");
        close(sock);
        return;
    }

    TickType_t xLastWakeTime;

    // 4. 臾댄븳 猷⑦봽瑜� �룎硫� �뜲�씠�꽣 �닔�떊 諛� �쓳�떟
    while (1) {
    	xLastWakeTime = xTaskGetTickCount();
        n = recvfrom(sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
                     (struct sockaddr *)&client_addr, &client_addr_len);

        if (n < 0) {
            xil_printf("Error: recvfrom failed\r\n");
            continue;
        }

        // �닔�떊�맂 �뜲�씠�꽣�쓽 �걹�뿉 Null 臾몄옄瑜� 異붽��븯�뿬 C-�뒪���씪 臾몄옄�뿴濡� 留뚮벊�땲�떎.
        recv_buffer[n] = '\0';
        xil_printf("\nReceived raw string: \"%s\"\r\n", recv_buffer);

        // �뙆�떛�맂 �뜲�씠�꽣瑜� ���옣�븷 蹂��닔�뱾
        float p_val, y_val ;
        float x_coord, y_coord;


        // sscanf瑜� �궗�슜�븯�뿬 �듅�젙 �삎�떇�쓽 臾몄옄�뿴�쓣 �뙆�떛�빀�땲�떎.
        // �꽦怨듭쟻�쑝濡� 4媛쒖쓽 �빆紐⑹쓣 紐⑤몢 �씫�쑝硫� 4瑜� 諛섑솚�빀�땲�떎.
        int items_scanned = sscanf(recv_buffer, "P: %f Y: %f x: %f y: %f",
                                   &p_val, &y_val, &x_coord, &y_coord);

        // �씠誘몄� 醫뚰몴
        tx_image_coords[0] = x_coord;
        tx_image_coords[1] = y_coord;


        // [�빑�떖 濡쒖쭅] �뙆�떛�씠 �꽦怨듯뻽�뒗吏� �솗�씤
        if (items_scanned == 4) {
            // �뙆�떛 �꽦怨�!
            xil_printf("Parsing successful.\r\n");

            XTime_GetTime(&t_start);
            //�젣�뼱 �븣怨좊━利� 援ы쁽 遺�遺�
            //IBVS �떎�뻾
            ibvs_e = IBVS_calculation(velocity_out, x_coord, y_coord);
            if(ibvs_e < 0 )continue;

            snprintf(buf, sizeof(buf), "omega_pitch: %.4f    omega_yaw: %.4f ",
                                        velocity_out[0],velocity_out[1]);
            xil_printf("After IBVS, %s\r\n", buf);

            //PID 援ы쁽
            pitch_deg = pid_calculation(&pid_pitch, velocity_out[0], p_val);
            yaw_deg = pid_calculation(&pid_yaw, velocity_out[1], y_val);

            //snprintf(buf, sizeof(buf), "p_error:%.4f  Y_error:%.4f P_deg: %.4f Y_deg: %.4f "
            //							,pid_pitch.pre_e, pid_yaw.pre_e, pitch_deg, yaw_deg);


            if (xAngleQueue != NULL)
                        {
                            ServoCommand_t cmd_m1, cmd_m2, cmd_m3, cmd_m4;

                            // 1. p_deg 값으로 0번, 1번 모터 제어
                            cmd_m1.motor_id = 0;
                            cmd_m1.angle = pitch_deg;

                            cmd_m2.motor_id = 1;
                            cmd_m2.angle = pitch_deg;

                            // 2. y_deg 값으로 2번, 3번 모터 제어
                            cmd_m3.motor_id = 2;
                            cmd_m3.angle = yaw_deg;

                            cmd_m4.motor_id = 3;
                            cmd_m4.angle = yaw_deg;

                            // 3. 4개의 명령을 큐로 전송 (Non-blocking)
                            xQueueSend(xAngleQueue, &cmd_m1, (TickType_t)0);
                            xQueueSend(xAngleQueue, &cmd_m2, (TickType_t)0);
                            xQueueSend(xAngleQueue, &cmd_m3, (TickType_t)0);
                            xQueueSend(xAngleQueue, &cmd_m4, (TickType_t)0);
                        }


            matrix_scalar_mul((float)(t_end - t_start) / (float)COUNTS_PER_SECOND, velocity_out, 2, 1);
            snprintf(buf, sizeof(buf), " P_deg: %.4f Y_deg: %.4f "
                 							 , pitch_deg, yaw_deg);
            xil_printf("IBVS-> deg : %s\r\n", buf);
            // snprintf瑜� �궗�슜�븯�뿬 �쓳�떟 臾몄옄�뿴�쓣 �떎�떆 留뚮벊�땲�떎.
            // %.1f�뒗 �냼�닔�젏 泥レ㎏ �옄由ш퉴吏�留� �몴�떆�븯�룄濡� �빀�땲�떎.
            snprintf(send_buffer, sizeof(send_buffer), "P: %.5f Y: %.5f x: %.5f y: %.5f",
            		pitch_deg, yaw_deg, x_coord, y_coord);

            // 泥섎━�맂 臾몄옄�뿴�쓣 �겢�씪�씠�뼵�듃�뿉寃� �떎�떆 蹂대깄�땲�떎.
            sendto(sock, send_buffer, strlen(send_buffer), 0,
                   (struct sockaddr *)&client_addr, client_addr_len);

            xil_printf("Sent back modified string: \"%s\"\r\n", send_buffer);

        } else {
            // �뙆�떛 �떎�뙣! �닔�떊�맂 臾몄옄�뿴�쓽 �삎�떇�씠 �삁�긽怨� �떎由낅땲�떎.
            xil_printf("Warning: Received string does not match expected format.\r\n");

            // �뿉�윭 硫붿떆吏�瑜� �겢�씪�씠�뼵�듃�뿉寃� 蹂대궪 �닔�룄 �엳�뒿�땲�떎.
            const char *error_msg = "Error: Invalid string format received by Zynq.";
            sendto(sock, error_msg, strlen(error_msg), 0,
                   (struct sockaddr *)&client_addr, client_addr_len);
        }
        XTime_GetTime(&t_end);

        //�떆媛� 怨꾩궛
        elapsed_us = 1e6 * (double)(t_end - t_start) / (double)COUNTS_PER_SECOND;
        u32 elapsed_int_us = (u32)elapsed_us;
        u32 elapsed_int_ms = elapsed_int_us / 1000;

        xil_printf("Processing+Send time: %u us (~%u ms)\r\n",elapsed_int_us, elapsed_int_ms);

        //vTaskDelayUntil(&xLastWakeTime, 20);
    }

    close(sock);
}
