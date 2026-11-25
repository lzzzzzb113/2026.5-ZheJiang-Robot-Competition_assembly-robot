//
// Created by 32827 on 2023/11/15.
//

#ifndef YZF_H
#define YZF_H
#include <stdio.h>
#include <inttypes.h>

#include "ti_msp_dl_config.h"
#include "yzf_basic_ram.h"

#define YZF_ON 1
#define YZF_OFF 0
#define YZF_NULL 255U
#define YZF_NULL_PTR 0

#define ARM_MATH_CM0


enum YZF_BOOL{
    YZF_FALSE = 0,
    YZF_TRUE = 1
};

enum YZF_STATUS{
    YZF_OK=0,
    YZF_ERROR,          ///未知错误
    YZF_NOT_INIT,       ///成员变量未初始化
    YZF_NOT_FOUND,      ///所选参数/配置未找到
    YZF_CRC_ERROR,      ///CRC校验错误
    YZF_CLOCK_NOT_READY, ///时钟未就绪
    YZF_HANDLER_NOT_FINISH,  ///处理函数未结束
    YZF_NOTHING          ///空闲
};

typedef enum{
    FLAG_OFF,
    FLAG_ON
} YZF_FLAG;

typedef enum YZF_STATUS YZF_RESULT;
typedef enum YZF_BOOL YZF_BOOL;

void YZF_ERROR_HANDLE(YZF_RESULT error);

#define YZF_CHECK YZF_ON

#if YZF_CHECK
#define YZF_CHECK_PARAM(n) if((int)n==YZF_NULL){return YZF_NOT_INIT;};
#define YZF_CHECK_RETURN(n) if(n!=YZF_OK){YZF_ERROR_HANDLE(n);};
#else
#define YZF_CHECK_PARAM(n) ;
#define YZF_CHECK_RETURN(n) n
#endif

#define YZF_FLAG_BIT_C(Reg,mask,isClear) ((Reg & mask) != 0 ? (isClear?(Reg&= ~mask) : 0,YZF_TRUE) :YZF_FALSE)
#define YZF_FLAG_BIT(Reg,mask) ((Reg & mask) != 0 ? YZF_TRUE :YZF_FALSE)
#define YZF_ENABLE(Reg,mask) (Reg |= mask)
#define YZF_DISABLE(Reg,mask) (Reg &= ~mask)
#define YZF_MODIFY_REG(Reg,mask,value,offset) (Reg = (Reg & ~ (mask)) | ((value) << offset & (mask)))
#define YZF_PIN_CONV_FROM(n) (__builtin_popcount((n)-1))

/**
 * For M0 Version And the further version will remove peripheral code.
 * They will be release by the peripheral driver.
 * */

/**
 *  再次记录一些需要被调用(实现)的函数
 *  yz_heart_handler(1) //全局心跳更新函数，要使用延迟函数还需在初始化中添加 yz_time_block_init();
 *  yz_log.h 下与串口发送相关的函数 void yz_shell_send(void *file,uint8_t *str,  size_t len)
 *
 * */

///------------------------------------------------
#endif //YZFRAME_YZF_H
