//
// Created by 32827 on 2023/11/23.
//

#ifndef YZFRAME_YZ_TIME_H
#define YZFRAME_YZ_TIME_H

#include "yz_heart.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * 一个简单的延迟模块
 * */

typedef struct {
    uint32_t SYStemOSTime;
} YZ_TIME;

YZF_RESULT yz_time_block_init();

YZ_TIME * yz_time_get();

void yz_time_block_delay_tick(uint32_t tick);

#ifdef __cplusplus
};
#endif

#endif //YZFRAME_YZ_TIME_H
