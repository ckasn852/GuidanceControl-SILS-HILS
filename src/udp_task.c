/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * All rights reserved.
 * ... (���̼��� ����� ����) ...
 */




#include "udp_task.h" // ���� ���ϸ� �°� ����

#include <stdio.h>  // sscanf, snprintf �� ���� �߰�
#include <string.h> // memset, strlen �� ���� �߰�
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
// ���� �� �۽��� ���� ������ �ִ� ũ�⸦ ����

struct netif server_netif;
static int complete_nw_thread;
#define THREAD_STACKSIZE 1024

//PID ����
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

//�������

//PID�ʱ�ȭ �Լ�
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

    // ����/�۽��� ���� ���� ����
    char recv_buffer[MAX_BUFFER_LEN];
    char send_buffer[MAX_BUFFER_LEN];

    // 1. UDP ���� ����
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        xil_printf("Error: Could not create socket\r\n");
        return;
    }

    // 2. ���� �ּ� ����
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_CONN_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    // 3. ���Ͽ� �ּ� �Ҵ� (���ε�)
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        xil_printf("Error: Failed to bind socket\r\n");
        close(sock);
        return;
    }

    // 4. ���� ������ ���� ������ ���� �� ����
    while (1) {
        n = recvfrom(sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
                     (struct sockaddr *)&client_addr, &client_addr_len);

        if (n < 0) {
            xil_printf("Error: recvfrom failed\r\n");
            continue;
        }

        // ���ŵ� �������� ���� Null ���ڸ� �߰��Ͽ� C-��Ÿ�� ���ڿ��� ����ϴ�.
        recv_buffer[n] = '\0';
        xil_printf("\nReceived raw string: \"%s\"\r\n", recv_buffer);

        // �Ľ̵� �����͸� ������ ������
        float p_val, y_val;
        int x_coord, y_coord;

        //������ Ÿ�� �ӵ�
        float target_p = -18;
        float target_y = 10;

        // sscanf�� ����Ͽ� Ư�� ������ ���ڿ��� �Ľ��մϴ�.
        // ���������� 4���� �׸��� ��� ������ 4�� ��ȯ�մϴ�.
        int items_scanned = sscanf(recv_buffer, "P: %f Y: %f x: %d y: %d",
                                   &p_val, &y_val, &x_coord, &y_coord);

        // [�ٽ� ����] �Ľ��� �����ߴ��� Ȯ��
        if (items_scanned == 4) {
            // �Ľ� ����!
            xil_printf("Parsing successful. Original Y value: %.1f\r\n", y_val);
            //���� �˰��� ���� �κ�
            //PID ����
            y_val = pid_calculation(&pid_yaw, target_y, y_val);  //target*1.5 =3
            p_val = pid_calculation(&pid_pitch, target_p, p_val);
            char buf[32];
            snprintf(buf, sizeof(buf), "P: %.1f Y: %.1f ",
                                 p_val,y_val);
            xil_printf("After PID : %s\r\n", buf);
            
            // Y ���� 1.0�� ���մϴ�.
            y_val += 1.0;

            // snprintf�� ����Ͽ� ���� ���ڿ��� �ٽ� ����ϴ�.
            // %.1f�� �Ҽ��� ù° �ڸ������� ǥ���ϵ��� �մϴ�.
            snprintf(send_buffer, sizeof(send_buffer), "P: %.1f Y: %.1f x: %d y: %d",
                     p_val, y_val, x_coord, y_coord);

            // ó���� ���ڿ��� Ŭ���̾�Ʈ���� �ٽ� �����ϴ�.
            sendto(sock, send_buffer, strlen(send_buffer), 0,
                   (struct sockaddr *)&client_addr, client_addr_len);

            xil_printf("Sent back modified string: \"%s\"\r\n", send_buffer);

        } else {
            // �Ľ� ����! ���ŵ� ���ڿ��� ������ ����� �ٸ��ϴ�.
            xil_printf("Warning: Received string does not match expected format.\r\n");

            // ���� �޽����� Ŭ���̾�Ʈ���� ���� ���� �ֽ��ϴ�.
            const char *error_msg = "Error: Invalid string format received by Zynq.";
            sendto(sock, error_msg, strlen(error_msg), 0,
                   (struct sockaddr *)&client_addr, client_addr_len);
        }
    }

    close(sock);
}
