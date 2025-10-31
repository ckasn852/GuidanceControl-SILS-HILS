#ifndef NETWORK_BOOTSTRAP_H
#define NETWORK_BOOTSTRAP_H

#define PLATFORM_EMAC_BASEADDR XPAR_XEMACPS_0_BASEADDR
#define PLATFORM_ZYNQ

// Zynq 보드가 수신 대기할 UDP 포트 번호
#define UDP_CONN_PORT 7777		/*언리얼 엔진에서 이 포트로 데이터를 보내야 합니다.*/
#define DEFAULT_IP_ADDRESS  "192.168.1.10"
#define DEFAULT_IP_MASK     "255.255.255.0"
#define DEFAULT_GW_ADDRESS  "192.168.1.1"
#define THREAD_STACKSIZE 1024 /* 스레드 스택 크기 (lwIP 소켓/파싱 사용 시 4KB 권장) */

//#include <stdio.h>         // sscanf, snprintf
//#include <string.h>        // memset, strlen
//#include <sleep.h>
//#include "xil_printf.h"
//#include "lwipopts.h"
//#include "lwip/ip_addr.h"
//#include "lwip/err.h"
//#include "lwip/udp.h"
//#include "lwip/init.h"
//#include "lwip/inet.h"
#include "lwip/sockets.h"
//#include "lwip/sys.h"
//#include "netif/xadapter.h"
//#include "platform_config.h"

/* 외부/전역 네트워크 인터페이스 */
extern struct netif server_netif;
extern int complete_nw_thread;
extern struct sockaddr_in client_addr;
extern socklen_t client_addr_len;

// 서버의 상태를 관리하기 위한 구조체
typedef struct {
    int sock; // 소켓 파일 디스크립터
    struct sockaddr_in server_addr;
} UdpServerContext;

// UDP 소켓
extern int udp_sock;

int udp_server_init(UdpServerContext* ctx, int port);
void udp_server_cleanup(UdpServerContext* ctx);
void assign_default_ip(ip_addr_t* ip, ip_addr_t* mask, ip_addr_t* gw);
void print_ip(const char* msg, ip_addr_t* ip);
void print_ip_settings(ip_addr_t* ip, ip_addr_t* mask, ip_addr_t* gw);
void print_app_header(void);
void network_thread(void* p);
void network_bootstrap();
void HILS_init(void);


#endif /* NETWORK_BOOTSTRAP_H */
