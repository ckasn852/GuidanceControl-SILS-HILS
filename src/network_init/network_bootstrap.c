#include <stdio.h>
#include <string.h>
#include <sleep.h>
#include "xil_printf.h"
#include "network_bootstrap.h"
#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/init.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "netif/xadapter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

struct netif server_netif;
int complete_nw_thread = 0; // 0으로 초기화
struct sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);

// rx_task 에서 사용할 UDP 전역 소켓
int udp_sock = -1;

SemaphoreHandle_t gNetworkInitMutex = NULL;

// socat -v udp4-recvfrom:7777,bind=127.0.0.1,fork udp4-sendto:192.168.1.10:7777

// UDP 서버 초기화 함수
int udp_server_init(UdpServerContext* ctx, int port)
{
    // 소켓 생성
    ctx->sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (ctx->sock < 0) {
        xil_printf("Error: Could not create socket\r\n");
        return -1;
    }

    // 주소 바인드
    memset(&ctx->server_addr, 0, sizeof(ctx->server_addr));
    ctx->server_addr.sin_family = AF_INET;
    ctx->server_addr.sin_port = htons(port);
    ctx->server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(ctx->sock, (struct sockaddr*)&ctx->server_addr, sizeof(ctx->server_addr)) < 0) {
        xil_printf("Error: Failed to bind socket\r\n");
        close(ctx->sock);
        return -1;
    }

    xil_printf("UDP Server initialized.\n");
    return 0; // 성공
}

// 3. UDP 서버 정리 함수
void udp_server_cleanup(UdpServerContext* ctx)
{
    if (ctx->sock >= 0) {
        close(ctx->sock);
        ctx->sock = -1;
        xil_printf("UDP Server cleaned up.\n");
    }
}


/* 네트워크 초기화 스레드: netif 추가, up, 수신 스레드 시작 */
void network_thread(void* p)
{
    /* 보드의 MAC 주소 (보드별 고유) */
    u8_t mac_ethernet_address[] = { 0x00, 0x18, 0x3E, 0x04, 0x51, 0x96 };

    if (!xemac_add(&server_netif, NULL, NULL, NULL, mac_ethernet_address,
        PLATFORM_EMAC_BASEADDR)) {
        xil_printf("Error adding N/W interface\r\n");
        vTaskDelete(NULL);
        return;
    }

    netif_set_default(&server_netif);
    netif_set_up(&server_netif);

    /* 수신 스레드 (lwIP 동작에 필요) */
    sys_thread_new("xemacif_input_thread",
        (void(*)(void*))xemacif_input_thread, &server_netif,
        THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

    complete_nw_thread = 1;
    vTaskDelete(NULL);
}

/* 보드가 DHCP 미사용일 때 기본 IP 설정 */
void print_ip(const char* msg, ip_addr_t* ip)
{
    xil_printf("%s", msg);
    xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
        ip4_addr3(ip), ip4_addr4(ip));
}

void print_ip_settings(ip_addr_t* ip, ip_addr_t* mask, ip_addr_t* gw)
{
    print_ip("Board IP:       ", ip);
    print_ip("Netmask :       ", mask);
    print_ip("Gateway :       ", gw);
}

void print_app_header(void)
{
    xil_printf("------------------------------------------------------\r\n");
    xil_printf("         UDP String Parser/Echo Application\r\n");
    xil_printf("------------------------------------------------------\r\n");
    xil_printf("Listening on port %d for formatted string data\r\n", UDP_CONN_PORT);
}

void assign_default_ip(ip_addr_t* ip, ip_addr_t* mask, ip_addr_t* gw)
{
    int err;
    xil_printf("Configuring default IP %s \r\n", DEFAULT_IP_ADDRESS);

    err = inet_aton(DEFAULT_IP_ADDRESS, ip);
    if (!err) xil_printf("Invalid default IP address: %d\r\n", err);

    err = inet_aton(DEFAULT_IP_MASK, mask);
    if (!err) xil_printf("Invalid default IP MASK: %d\r\n", err);

    err = inet_aton(DEFAULT_GW_ADDRESS, gw);
    if (!err) xil_printf("Invalid default gateway address: %d\r\n", err);
}


void network_bootstrap(){

    xil_printf("\n\r\n\r");
    xil_printf("Zynq board Boot Successful\r\n");
    xil_printf("-----lwIP Socket Mode UDP Client Application------\r\n");

    /* initialize lwIP before calling sys_thread_new */
    lwip_init();
    xil_printf("lwip_init() called\r\n");

    /* any thread using lwIP should be created using sys_thread_new */
    sys_thread_new("nw_thread", network_thread, NULL,
        THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

    while (!complete_nw_thread){
    	vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 징크보드 ip 할당
    assign_default_ip(&(server_netif.ip_addr), &(server_netif.netmask),
        &(server_netif.gw));

    // 징크보드 ip 세팅 결과 출력
    print_ip_settings(&(server_netif.ip_addr), &(server_netif.netmask),
        &(server_netif.gw));
    xil_printf("\r\n");

    /* print all application headers */
    print_app_header();
    xil_printf("\r\n");

    // 1. 서버 컨텍스트 변수 선언
    UdpServerContext server_context;

    // 2. 서버 초기화
    if (udp_server_init(&server_context, UDP_CONN_PORT) != 0) {
        xil_printf("Server initialization failed. Halting.\r\n");
        return;
    }

    udp_sock = server_context.sock;
    xil_printf("Waiting for data from client...\r\n");

}
