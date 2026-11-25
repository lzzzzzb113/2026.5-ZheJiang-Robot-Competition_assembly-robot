//
// Created by 32827 on 2023/12/28.
//
#include "yz_log.h"

char yz_log_transmit_buf[YZ_APPLICATION_TRANSMIT_BUF_SIZE];
__attribute__((weak)) void yz_error(const char *fmt, ...){
    char *str_error="[ERROR]";
    int i;

    va_list arg;
    va_start(arg, fmt);
    memset(yz_log_transmit_buf,0,sizeof(yz_log_transmit_buf));
    i = vsprintf(yz_log_transmit_buf, fmt, arg);
    va_end(arg);

    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)str_error, strlen(str_error));
    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)yz_log_transmit_buf,i);
}
__attribute__((weak)) void yz_info(const char *fmt, ...){
    char *str_info="[INFO]";
    int i;

    va_list arg;
    va_start(arg, fmt);
    memset(yz_log_transmit_buf,0,sizeof(yz_log_transmit_buf));
    i = vsprintf(yz_log_transmit_buf, fmt, arg);
    va_end(arg);

    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)str_info, strlen(str_info));
    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)yz_log_transmit_buf,i);
}
__attribute__((weak)) void yz_debug(const char *fmt, ...){
    char *str_debug="[DEBUG]";
    int i;

    va_list arg;
    va_start(arg, fmt);
    memset(yz_log_transmit_buf,0,sizeof(yz_log_transmit_buf));
    i = vsprintf(yz_log_transmit_buf, fmt, arg);
    va_end(arg);

    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)str_debug, strlen(str_debug));
    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)yz_log_transmit_buf,i);
}
__attribute__((weak)) void yz_ok(const char *fmt, ...){
    char *str_ok="[SUCCESS]";
    int i;

    va_list arg;
    va_start(arg, fmt);
    memset(yz_log_transmit_buf,0,sizeof(yz_log_transmit_buf));
    i = vsprintf(yz_log_transmit_buf, fmt, arg);
    va_end(arg);

    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)str_ok, strlen(str_ok));
    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)yz_log_transmit_buf,i);
}
__attribute__((weak)) void yz_printf(const char *fmt, ...){
    int i;

    va_list arg;
    va_start(arg, fmt);
    memset(yz_log_transmit_buf,0,sizeof(yz_log_transmit_buf));
    i = vsprintf(yz_log_transmit_buf, fmt, arg);
    va_end(arg);

    yz_shell_send(RUNNING_LOG_FILE_PATH,(uint8_t *)yz_log_transmit_buf,i);
}
__attribute__((weak)) void yz_shell_send(void *file,uint8_t *str,  size_t len){

}