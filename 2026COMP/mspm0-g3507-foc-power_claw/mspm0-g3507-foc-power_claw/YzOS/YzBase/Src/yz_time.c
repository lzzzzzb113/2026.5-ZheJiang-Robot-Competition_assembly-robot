//
// Created by 32827 on 2023/11/23.
//
#include "yz_time.h"
volatile YZ_TIME SystemOSTime= {0};

YZF_RESULT yz_time_heart_handler(uint32_t tick){
    SystemOSTime.SYStemOSTime+=tick;
    return YZF_HANDLER_NOT_FINISH;
}

YZF_RESULT yz_time_block_init(){
    yz_heart_process_add(yz_time_heart_handler);
    return YZF_OK;
}

YZ_TIME * yz_time_get(){
    return (YZ_TIME *)&SystemOSTime;
}

void yz_time_block_delay_tick(uint32_t tick){
    uint32_t end=SystemOSTime.SYStemOSTime+tick;
    while(SystemOSTime.SYStemOSTime<end);
}