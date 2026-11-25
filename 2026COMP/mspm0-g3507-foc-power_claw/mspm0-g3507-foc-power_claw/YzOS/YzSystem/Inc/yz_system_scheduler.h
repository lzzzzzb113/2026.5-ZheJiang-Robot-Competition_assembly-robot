//
// Created by 32827 on 2026/1/18.
//

#ifndef MSPM0_YZ_SYSTEM_SCHEDULER_H
#define MSPM0_YZ_SYSTEM_SCHEDULER_H

#include "yz_system_type.h"
#include "yz_time.h"

#define YZ_SYSTEM_MAX_TCB_COUNT 5

typedef enum
{
    SYSTEM_RUNNING,
    SYSTEM_BLOCKED,
    SYSTEM_READY,
    SYSTEM_SUSPEND,
    SYSTEM_DELETED
} YzSystemStatus;

typedef struct
{
    YzStack *stack;
    YzThreadStartParams *stack_params;
    void *params;
    YzSystemStatus status;
    uint32_t priority;
    volatile uint32_t waketime;
} YzSystemControlBlock;

typedef struct
{
    YzOSGlobalData global_data;
    volatile YzSystemControlBlock block[YZ_SYSTEM_MAX_TCB_COUNT];
    volatile int32_t prioritySoft[YZ_SYSTEM_MAX_TCB_COUNT];
    volatile int32_t current;
    volatile YZF_BOOL hasChange;
    volatile YZF_BOOL isInit;
    volatile YZF_BOOL isStart;
}YzSCBManager;

YZF_RESULT yz_system_thread_init(YzStack *stack,YzThreadStartParams *stack_params,YzThread thread, uint32_t priority , void *params);

YZF_RESULT yz_system_soft_priority();

YZF_RESULT yz_system_start_scheduler();

YZF_RESULT yz_system_thread_wait_exit(YZF_RESULT exitCode);
#endif //MSPM0_YZ_SYSTEM_SCHEDULER_H