//
// Created by 32827 on 2025/11/26.
//

#ifndef MSPM0_YZ_SYSTEM_TYPE_H
#define MSPM0_YZ_SYSTEM_TYPE_H
#include "yz.h"

#define YZ_SYSTEM_STACK_SIZE 1024

typedef enum{
    PEND_SV_NONE=0,
    PEND_SV_ENTRY=1,
    PEND_SV_CONV_FIRST=2,
    PEND_SV_CONV=3
} PendSVStatus;

typedef YZF_RESULT (*YzThread)(void *) ;

typedef struct
{
    uint32_t stack[YZ_SYSTEM_STACK_SIZE/4];
    uint32_t PROTECT_RAM[16];
} YzStack;

typedef struct
{
    uint32_t StartStackTop;
    YzStack *StartStack;
    YZF_BOOL isUsed;
    //--------------- Will COPY ----------------
    uint32_t R10;
    uint32_t R11;
    uint32_t R7;
    uint32_t R8;

    uint32_t R9;
    uint32_t R4;
    uint32_t R5;
    uint32_t R6;

    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;

    uint32_t R12;
    uint32_t LR;
    uint32_t PC;
    union {
        uint32_t EPSR;
        uint32_t IPSR;
        uint32_t APSR;
    };

    union {
        uint32_t MSP;
        uint32_t PSP;
    };
} YzThreadStartParams;

typedef struct
{
    uint32_t OSExitAddress;
    PendSVStatus OSPendSVStatus;
    YzStack *OSCurrentStack;
    YzStack *OSNextStack;
    YzThreadStartParams *OSNextThreadStartParams;
} YzOSGlobalData;


void SetPendSVPro(void);
void CLRPendSV(void);
void SetPendSV(void);

uint32_t yz_get_pc(void);

YZF_RESULT yz_thread_start_params_init(YzThreadStartParams *data,YzStack *stack,YzThread thread,void *params);

void yz_thread_os_enter(YzOSGlobalData *global,YzThreadStartParams *data,uint32_t exitAddress);

void yz_thread_os_conv(YzOSGlobalData *global,YzThreadStartParams *data);

#endif //MSPM0_YZ_SYSTEM_TYPE_H
