//
// Created by 32827 on 2026/4/6.
//

#include "uart_package_connect.h"
#include <string.h>
#include "yzf.h"

static const uint8_t uart_connect_head[2] = {0xee,0xff};
static const uint8_t uart_connect_tail[2] = {0x55,0xaa};
extern YZF_BOOL uart0_rx_buf_act();
void package_check(UartConnect *connect)
{
    while (uart0_rx_buf_act()){}
    if (connect->is_new_package)
    {
        connect->is_new_package = 0;
        connect->package_handler(connect);
    }
}

void package_put(UartConnect *connect,uint8_t data)
{
    switch (connect->status)
    {
        case IDLE:
            if (data == uart_connect_head[0])
                connect->status = STX_1;
            break;
        case STX_1:
            if (data == uart_connect_head[1])
            {
                connect->status = RECV;
                connect->stx_index = 0;
            }
            else
            {
                connect->status = IDLE;
            }
            break;
        case RECV:
            if (data == uart_connect_tail[0])
            {
                connect->status = ETX_1;
                connect->rx_buf[connect->stx_index++] = data;   //假设不是先接收
            }else
            {
                connect->rx_buf[connect->stx_index++] = data;
            }
            break;
        case ETX_1:
            if (data == uart_connect_tail[1])
            {
                connect->status = IDLE;
                connect->stx_index--;                           //回退长度
                connect->is_new_package = 1;
            }else
            {
                connect->status = RECV;
                connect->rx_buf[connect->stx_index++] = data;
            }
            break;
        default:
            connect->status = IDLE;
    }
}