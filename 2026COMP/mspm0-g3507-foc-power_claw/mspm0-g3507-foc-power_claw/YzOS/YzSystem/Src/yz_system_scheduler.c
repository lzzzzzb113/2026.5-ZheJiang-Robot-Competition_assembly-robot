//
// Created by 32827 on 2026/1/18.
//

#include "../Inc/yz_system_scheduler.h"

#include "yz_log.h"

YzSCBManager scb_manager={0};

YZF_RESULT yz_system_scheduler_heart_handler(uint32_t tick){
    asm volatile("dsb\n");
    asm volatile("isb\n");
    asm volatile("CPSID i\n");
    //需要重新调度的情况
    //1.有程序结束睡眠，并且是比当前运行的优先级高
    //2.有程序进入就绪态，并且是比当前运行的优先级高
    if (scb_manager.isStart&&yz_system_soft_priority()==YZF_OK)  //如果返回YZF_NOTHING，则说明没有需要重新调度的程序
    {
        if (scb_manager.hasChange == YZF_TRUE)
        {
            //如果需要改变就是要切换到处于就绪状态的最高优先级的事项
            for (int i=0;i<YZ_SYSTEM_MAX_TCB_COUNT;i++)
            {
                int index = scb_manager.prioritySoft[i];

                if (index==-1)
                {
                    scb_manager.hasChange = YZF_FALSE;
                    break;//已经查找到末尾
                }

                volatile YzSystemControlBlock *current,*next;
                current = &scb_manager.block[scb_manager.current];
                next = &scb_manager.block[index];

                if (next->status == SYSTEM_READY)
                {
                    if (current->status != SYSTEM_DELETED)
                    {
                        //前一个状态到Ready
                        current->status = SYSTEM_SUSPEND;
                    }
                    scb_manager.current = index;
                    next->status = SYSTEM_RUNNING;
                    scb_manager.hasChange = YZF_FALSE;
                    yz_thread_os_conv(&scb_manager.global_data,next->stack_params);
                    break;
                }
            }
        }
    }
    asm volatile("CPSIE i\n");
    return YZF_HANDLER_NOT_FINISH;
}

int yz_system_manager_get_free_index()
{
    for (int i=0;i<YZ_SYSTEM_MAX_TCB_COUNT;i++)
    {
        if (scb_manager.block[i].status == SYSTEM_DELETED)
        {
            return i;
        }
    }
    return -1;
}

/**
 * 此函数会维护SCB的状态，会将SUSPEND状态恢复为READY，同时判断最高优先级是否发生改变
 *
 * @return YZF_NOTHING表示所有进程都已结束 YZF_OK表示正常运行 YZF_NOT_FOUND表示无变化
 */
YZF_RESULT yz_system_soft_priority()
{
    //生成prioritySoft 冒泡排序 剔除 scb_manager.block[i].status == SYSTEM_DELETED;

    //先提取所有不为SYSTEM_DELETED的控制块，将索引加入prioritySoft，并记录个数
    int count = 0;
    uint32_t nowtime = yz_time_get()->SYStemOSTime;
    for (int i = 0; i < YZ_SYSTEM_MAX_TCB_COUNT; i++)
    {
        volatile YzSystemStatus* nowStatus = &scb_manager.block[i].status;
        if (*nowStatus != SYSTEM_DELETED)
        {
            //顺便更新状态，也许应该改下函数名字，改为状态维护
            //睡眠状态恢复
            if (*nowStatus == SYSTEM_SUSPEND
                && scb_manager.block[i].waketime<nowtime)
            {
                *nowStatus = SYSTEM_READY;
            }
            scb_manager.prioritySoft[count] = i;
            count++;
        }
    }
    for (int i=count; i < YZ_SYSTEM_MAX_TCB_COUNT; i++)
    {
        scb_manager.prioritySoft[i] = -1;
    }

    //对prioritySoft内的索引代表的block进行排序
    for (int i = 0; i < count; i++)
    {
        for (int j = 0; j < count - i - 1; j++)
        {
            if (scb_manager.block[scb_manager.prioritySoft[j]].priority < scb_manager.block[scb_manager.prioritySoft[j + 1]].priority)
            {
                int temp = scb_manager.prioritySoft[j];
                scb_manager.prioritySoft[j] = scb_manager.prioritySoft[j + 1];
                scb_manager.prioritySoft[j + 1] = temp;
            }
        }
    }

    if (scb_manager.prioritySoft[0] == -1)
    {
        return YZF_NOTHING;
    }

    //判断最高优先级是否有变化，如果有变化，则hasChange = YZF_TRUE
    if (scb_manager.hasChange != YZF_TRUE )
    {
        for (int i=0;i<count;i++)
        {
            if (scb_manager.current == scb_manager.prioritySoft[i])
            {
                return YZF_NOT_FOUND;
            }
            else if (scb_manager.block[scb_manager.prioritySoft[i]].status == SYSTEM_READY)
            {
                scb_manager.hasChange = YZF_TRUE;
                return YZF_OK;
            }
        }
    }

    return YZF_OK;
}

YZF_RESULT yz_system_thread_init(YzStack *stack,YzThreadStartParams *stack_params,YzThread thread, uint32_t priority , void *params)
{
    //如果scb_manager没有初始化,则先初始化
    if (scb_manager.isInit == YZF_FALSE)
    {
        scb_manager.hasChange = YZF_TRUE;
        memset((void *)&scb_manager.block, 0, sizeof(YzSystemControlBlock) * YZ_SYSTEM_MAX_TCB_COUNT);
        memset((void *)&scb_manager.prioritySoft, 0, sizeof(int32_t) * YZ_SYSTEM_MAX_TCB_COUNT);
        scb_manager.current = -1;
        for (int i = 0; i < YZ_SYSTEM_MAX_TCB_COUNT; i++)
        {
            scb_manager.block[i].status = SYSTEM_DELETED;   //默认为删除状态
        }
        scb_manager.isStart = YZF_FALSE;                    //默认还没开启调度
        scb_manager.isInit = YZF_TRUE;
    }

    //获取空闲位置，初始化scb
    int i = yz_system_manager_get_free_index();

    if (i < 0)  //没有空闲位置
    {
        return YZF_ERROR;
    }

    scb_manager.block[i].priority = priority;
    scb_manager.block[i].stack = stack;
    scb_manager.block[i].params = params;
    scb_manager.block[i].stack_params = stack_params;

    yz_thread_start_params_init(stack_params,stack,thread,params);        //初始化堆栈

    scb_manager.block[i].status = SYSTEM_READY; //默认为Ready

    return YZF_OK;
}

YZF_RESULT yz_system_start_scheduler()
{
    //更新软排序，维护SCB状态
    yz_system_soft_priority();

    //开始调度
    scb_manager.current = scb_manager.prioritySoft[0];
    volatile YzSystemControlBlock* now = &scb_manager.block[scb_manager.current];
    if (now->status!=SYSTEM_READY) //未准备好，返回
    {
        return YZF_ERROR;
    }
    now->status = SYSTEM_RUNNING;  //设置为运行状态
    scb_manager.isStart=YZF_TRUE;  //调度器启动

    yz_heart_process_add(yz_system_scheduler_heart_handler);//向心跳压入调度器

    yz_thread_os_enter(&scb_manager.global_data,now->stack_params,(uint32_t)yz_system_thread_wait_exit);//进入

    return YZF_OK; //不会到这
}

/**
 * 线程结束会来到这里，线程的返回值为 YZF_RESULT
 * @param exitCode 线程返回值
 * @return 退出方式
 */
YZF_RESULT yz_system_thread_wait_exit(YZF_RESULT exitCode)
{
    //如果其中一个线程结束会返回到这里，可能需要通过判断psp来判断是哪个线程结束，用以将状态变化为SYSTEM_DELETED，然后重新进入调度
    //不对，来到这里的必然是current
    asm volatile("CPSID i\n");
    volatile YzSystemControlBlock* cur = &scb_manager.block[scb_manager.current];

    cur->status = SYSTEM_DELETED;

    if (yz_system_soft_priority()==YZF_NOTHING) //全部已经退出
    {
        asm volatile("CPSIE i\n");
        //确认最后一个线程结束，退出
        __set_CONTROL(0x0); //切换回MSP
        scb_manager.isStart = YZF_FALSE;
        while (1)
        {
            asm volatile("nop");
        }
    }else
    {
        asm volatile("CPSIE i\n");
        while (1)   //等待调度
        {
            asm volatile("nop");
        }
    }
}