/*
 * Copyright (C) 2017 - 2019 Xilinx, Inc.
 * All rights reserved.
 * ... (라이선스 헤더는 동일) ...
 */

#ifndef __UDP_ECHO_SERVER_H_
#define __UDP_ECHO_SERVER_H_

#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "xil_printf.h"

// Zynq 보드가 수신 대기할 UDP 포트 번호
// 언리얼 엔진에서 이 포트로 데이터를 보내야 합니다.
#define UDP_CONN_PORT 7777

#endif /* __UDP_ECHO_SERVER_H_ */
