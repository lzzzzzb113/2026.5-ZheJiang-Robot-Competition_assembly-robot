//
// Created by 32827 on 2026/4/23.
//

#ifndef STM32H723_LTS_BSP_YZL_SPACE_SENSOR_ARM_ASYNC_LINE_H
#define STM32H723_LTS_BSP_YZL_SPACE_SENSOR_ARM_ASYNC_LINE_H
#include "yzl_softport.h"
#include "yz_tim.h"
#include "yzl_space_sensor_arm.hpp"
#include "yzl_space_sensor_arm_pole.hpp"
#ifdef YZSCREEN
#include "yzl_ui_widget_pp_text.hpp"
#include "yzl_ui_widget_pp_text_button.hpp"
#endif

typedef struct
{
    int RawStep;
    int Dir;
    YZF_BOOL isInit;
    YZF_BOOL isFinish;
} ArmAsyncBlock;

typedef struct
{
    YZF_BOOL isUse;
    SpaceSensorArmPolePortParam *PipeParam;
} ArmTimAsyncPipe;

typedef struct
{
    ArmTimAsyncPipe PipeCH1;
    ArmTimAsyncPipe PipeCH2;
    ArmTimAsyncPipe PipeCH3;
    ArmTimAsyncPipe PipeCH4;
    float speed;
    int coreSpeed;
    void * InstanceHandler;   //C++实例句柄
} ArmAsyncLineParams;

class SpaceSensorArmAsyncLine
{
public:
    ArmAsyncLineParams *LineParams{};
    ArmAsyncBlock PipeBlockCH1{};
    ArmAsyncBlock PipeBlockCH2{};
    ArmAsyncBlock PipeBlockCH3{};
    ArmAsyncBlock PipeBlockCH4{};
    ArmAsyncBlock* activeBlocks[4] = {nullptr, nullptr, nullptr, nullptr};
    YZF_TIM_POINTER timCount;
    int activeChannels[4] = {0};
    int activeCount = 0;
    int nowPosition[4]={0};     //当前步数(脉冲数）
    int NextStep{};
    int CurrentIndexStep{};
#ifdef YZSCREEN
    YZF_BOOL hasWidget=YZF_FALSE;
    WidgetTextButton *widgetText{};
#endif

    SpaceSensorArmAsyncLine(ArmAsyncLineParams *params,YZF_TIM_POINTER timCount);

    void InitAllCHGpio();

    YZF_RESULT ApplyNewParams(ArmAsyncLineParams *params);

    YZF_RESULT ClearALL();

    YZF_RESULT Push(int ch,ArmAsyncBlock *block);

    YZF_RESULT Resolve();

    YZF_RESULT PartSend();

    YZF_BOOL isFinish();

    YZF_BOOL isHanded();

    void act();

    //先 Push block
    //loop isHanded == True
    //  PartSend()
    //  Check isFinish()
};
#endif //STM32H723_LTS_BSP_YZL_SPACE_SENSOR_ARM_ASYNC_LINE_H