//
// Created by 32827 on 2026/4/6.
//

#ifndef MSPM0_UART_PACKAGE_CONNECT_H
#define MSPM0_UART_PACKAGE_CONNECT_H
#include "inttypes.h"

#define UART_CONNECT_PACKAGE_SIZE 256

typedef enum
{
    IDLE = 0,
    STX_1,
    RECV,
    ETX_1
} UartConnectStatus;
/**
 * UART连接包
 * 包头[4]：0xee,0xff,0x55,0xaa
 * 包ID[1]: 0x__ 每次包+1
 */
typedef struct
{
    UartConnectStatus status;
    uint8_t rx_buf[UART_CONNECT_PACKAGE_SIZE];
    uint8_t is_new_package;
    uint8_t stx_index;              //接收数据长度，以及接收时的写入指针
    uint8_t package_id;
    void (*package_handler)(void *connect);
} UartConnect;

void package_check(UartConnect *connect);
void package_put(UartConnect *connect,uint8_t data);
#endif //MSPM0_UART_PACKAGE_CONNECT_H