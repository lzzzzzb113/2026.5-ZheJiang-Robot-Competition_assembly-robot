//
// Created by 32827 on 2025/11/26.
//

#include "yz_system_type.h"

uint32_t yz_get_pc(void) {
    uint32_t pc;
    asm volatile (
            "mov %0, lr"
            : "=r" (pc)
            );
    return pc;
}


YZF_RESULT yz_thread_start_params_init(YzThreadStartParams *data,YzStack *stack,YzThread thread,void *params)
{
    data->PC = ((uint32_t) (YzThread) thread) | 0x1;            //注意PC bit0需要为1 以表示Thumb状态
    data->R0 = (uint32_t) params;
    data->R1 = 0x0;
    data->R2 = 0x0;
    data->R3 = 0x0;
    data->R12 = 0x0;
    data->APSR = 0x01000000;                                    //注意T-thumb状态为要为1
    data->PSP = (uint32_t) (stack->PROTECT_RAM);                //跳转前线程栈顶 只在第一次启动时使用
    data->StartStackTop =(uint32_t) (stack->PROTECT_RAM - 16);  //后续线程的栈顶预置
    data->StartStack = stack;
    data->isUsed = YZF_FALSE;

    data->R10 = 0x0;
    data->R11 = 0x0;
    data->R7 = 0x0;
    data->R8 = 0x0;
    data->R9 = 0x0;
    data->R4 = 0x0;
    data->R5 = 0x0;
    data->R6 = 0x0;
    return YZF_OK;
}


YzOSGlobalData *os_global_data;

void yz_thread_os_enter(YzOSGlobalData *global,YzThreadStartParams *data,uint32_t exitAddress)
{
    if (exitAddress==0)
    {
        uint32_t lr;
        asm volatile (
                "mov %0, lr"
                : "=r" (lr)
                );
        global->OSExitAddress = lr; //记录退出地址
    }else
    {
        global->OSExitAddress = exitAddress;
    }

    os_global_data = global;

    SetPendSVPro();                                     //设置 PendSV 中断优先级为最低（通常在 RTOS 中使用）
    __set_PSP(data->PSP);                               //设置 PSP 寄存器为该值
    __set_CONTROL(0x2);                                 //设置 CONTROL 寄存器：
                                                        // bit[1] = 1: 使用 PSP 作为栈指针（线程模式）
                                                        // bit[0] = 0: 线程模式使用特权级（不是用户级）
    global->OSPendSVStatus = PEND_SV_ENTRY;
    global->OSNextStack = data->StartStack;
    global->OSNextThreadStartParams = data;
    data->isUsed = YZF_TRUE;

    asm volatile("dsb");
    asm volatile("isb");

    SetPendSV();
}

void yz_thread_os_conv(YzOSGlobalData *global,YzThreadStartParams *data)
{

    global->OSNextStack = data->StartStack;
    global->OSNextThreadStartParams = data;
    if(data->isUsed == YZF_FALSE){
        global->OSPendSVStatus = PEND_SV_CONV_FIRST;
        data->isUsed = YZF_TRUE;
    }else{
        global->OSPendSVStatus = PEND_SV_CONV;
    }

    asm volatile("dsb");
    asm volatile("isb");

    SetPendSV();
}

volatile void * OS_CURRENT_STACK;
volatile void * OS_NEXT_STACK;

void PendSV_Handler(void)
{
    CLRPendSV();
    switch (os_global_data->OSPendSVStatus)
    {
        case PEND_SV_NONE:
            break;
        case PEND_SV_ENTRY:
            os_global_data->OSPendSVStatus = PEND_SV_NONE;
            //第一次进入需要为退出设置地址，在这是因为，在初始化处可能未定义ExitAddress
            os_global_data->OSNextThreadStartParams->LR = os_global_data->OSExitAddress;
            //本来应该*9的但是PSP不进行修改，因为其不会被自动压入栈内
            memcpy(&os_global_data->OSNextStack->stack[YZ_SYSTEM_STACK_SIZE / 4 - 8],
                   &os_global_data->OSNextThreadStartParams->R0, sizeof(uint32_t) * 8);
            os_global_data->OSCurrentStack = os_global_data->OSNextStack;
            asm volatile(   "LDR R0, =0xFFFFFFFD\n"     // r0 = -3 = 0xFFFFFFFD
                            "mov lr, r0 \n"             // lr = EXC_RETURN
                            "bx lr \n");                // 异常返回
            break;
        case PEND_SV_CONV_FIRST: //切换上下文
            os_global_data->OSNextThreadStartParams->LR = os_global_data->OSExitAddress;
            memcpy(&os_global_data->OSNextStack->stack[YZ_SYSTEM_STACK_SIZE / 4 - 16],      //补上了其他通用寄存器的值
                   &os_global_data->OSNextThreadStartParams->R10, sizeof(uint32_t) * 16);
            os_global_data->OSNextStack->stack[0] = os_global_data->OSNextThreadStartParams->StartStackTop;
        case PEND_SV_CONV: //切换上下文
            asm volatile("CPSID i\n");
            OS_CURRENT_STACK = os_global_data->OSCurrentStack;
            OS_NEXT_STACK = os_global_data->OSNextStack;//(0x20200e68 - (uint32_t)&os_global_data->OSNextStack->stack[0])/4
            os_global_data->OSCurrentStack = os_global_data->OSNextStack;
            asm volatile(
                         "mov r3,sp\n"
                         "mrs r0,psp\n"
                         "mov sp,r0\n"

                         "mov r0,r4\n"
                         "mov r1,r5\n"
                         "mov r2,r6\n"
                         "push {r0-r2}\n"

                         "mov r0,r7\n"
                         "mov r1,r8\n"
                         "mov r2,r9\n"
                         "push {r0-r2}\n"

                         "mov r0,r10\n"
                         "mov r1,r11\n"
                         "push {r0-r1}\n"

                         "isb\n"
                         "mov r0,sp\n"
                         "isb\n"
                         "msr psp,r0\n"             //同步PSP

                         // 保存到currentStack结构
                         "ldr r1, =OS_CURRENT_STACK \n"
                         "ldr r2, [r1] \n"         // r2 = currentStack uint8_t *
                         "str r0, [r2, #0] \n"     // 保存PSP  保存在栈的最高处

                         "ldr r1, =OS_NEXT_STACK \n"
                         "ldr r0, [r1] \n"         // r0 = nextStack uint8_t *
                         "ldr r1, [r0, #0] \n"     // r1 = nextStack + 4
                         "isb\n"
                         "mov sp, r1 \n"

                         "pop {r0-r1} \n"
                         "mov r10, r0 \n"
                         "mov r11, r1 \n"

                         "pop {r0-r2} \n"
                         "mov r7, r0 \n"
                         "mov r8, r1 \n"
                         "mov r9, r2 \n"

                         "pop {r0-r2} \n"
                         "mov r4, r0 \n"
                         "mov r5, r1 \n"
                         "mov r6, r2 \n"

                         "isb\n"
                         "mov r0, sp\n"
                         "isb\n"
                         "msr psp, r0\n"
                         "isb\n"
                         "mov sp, r3\n"
            );
            asm volatile("CPSIE i\n");
            asm volatile("dsb\n");
            asm volatile("isb\n");
            asm volatile(
                         "LDR R0, =0xFFFFFFFD\n"   // r0 = -3 = 0xFFFFFFFD
                         "mov lr, r0 \n"        // lr = EXC_RETURN
                         "bx lr \n");           // 异常返回
            break;
        default:
            break;
    }
}