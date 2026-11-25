//
// Created by 32827 on 2026/4/23.
//

#include "yzl_space_sensor_arm_async_line.hpp"

SpaceSensorArmAsyncLine::SpaceSensorArmAsyncLine(ArmAsyncLineParams *params,YZF_TIM_POINTER timCount)
{
    ApplyNewParams(params);
    this->timCount = timCount;
}

YZF_RESULT SpaceSensorArmAsyncLine::ApplyNewParams(ArmAsyncLineParams *params)
{
    this->LineParams = params;
    this->LineParams->InstanceHandler = this;

    PipeBlockCH1.RawStep = 0;
    PipeBlockCH1.isInit = LineParams->PipeCH1.isUse;
    PipeBlockCH1.isFinish = YZF_TRUE;
    PipeBlockCH2.RawStep = 0;
    PipeBlockCH2.isInit = LineParams->PipeCH2.isUse;
    PipeBlockCH2.isFinish = YZF_TRUE;
    PipeBlockCH3.RawStep = 0;
    PipeBlockCH3.isInit = LineParams->PipeCH3.isUse;
    PipeBlockCH3.isFinish = YZF_TRUE;
    PipeBlockCH4.RawStep = 0;
    PipeBlockCH4.isInit = LineParams->PipeCH4.isUse;
    PipeBlockCH4.isFinish = YZF_TRUE;

    InitAllCHGpio();

    return YZF_OK;
}

void SpaceSensorArmAsyncLine::InitAllCHGpio()
{
    if (PipeBlockCH1.isInit)
    {
        YLGPIO(LineParams->PipeCH1.PipeParam->Dir)
        yzf_gpio_open_clock_();
        yzf_gpio_set_PP_();
        yzf_gpio_set_Pull_Up_();
        yzf_gpio_set_HIGH_();
        yzf_gpio_set_af_(0);
    }
    if (PipeBlockCH2.isInit)
    {
        YLGPIO(LineParams->PipeCH2.PipeParam->Dir)
        yzf_gpio_open_clock_();
        yzf_gpio_set_PP_();
        yzf_gpio_set_Pull_Up_();
        yzf_gpio_set_HIGH_();
        yzf_gpio_set_af_(0);
    }
    if (PipeBlockCH3.isInit)
    {
        YLGPIO(LineParams->PipeCH3.PipeParam->Dir)
        yzf_gpio_open_clock_();
        yzf_gpio_set_PP_();
        yzf_gpio_set_Pull_Up_();
        yzf_gpio_set_HIGH_();
        yzf_gpio_set_af_(0);
    }
    if (PipeBlockCH4.isInit)
    {
        YLGPIO(LineParams->PipeCH4.PipeParam->Dir)
        yzf_gpio_open_clock_();
        yzf_gpio_set_PP_();
        yzf_gpio_set_Pull_Up_();
        yzf_gpio_set_HIGH_();
        yzf_gpio_set_af_(0);
    }
}

YZF_RESULT SpaceSensorArmAsyncLine::ClearALL()
{
    if (PipeBlockCH1.isInit)
    {
        PipeBlockCH1.isFinish = YZF_TRUE;
    }
    if (PipeBlockCH2.isInit)
    {
        PipeBlockCH2.isFinish = YZF_TRUE;
    }
    if (PipeBlockCH3.isInit)
    {
        PipeBlockCH3.isFinish = YZF_TRUE;
    }
    if (PipeBlockCH4.isInit)
    {
        PipeBlockCH4.isFinish = YZF_TRUE;
    }
    return YZF_OK;
}

YZF_RESULT SpaceSensorArmAsyncLine::Push(int ch,ArmAsyncBlock *block)
{
    if(ch == 1)
    {
        if(PipeBlockCH1.isInit)
        {
            PipeBlockCH1.Dir = block->Dir;
            PipeBlockCH1.RawStep = block->RawStep;
            PipeBlockCH1.isFinish = YZF_FALSE;
        }
    }
    else if(ch == 2)
    {
        if(PipeBlockCH2.isInit)
        {
            PipeBlockCH2.Dir = block->Dir;
            PipeBlockCH2.RawStep = block->RawStep;
            PipeBlockCH2.isFinish = YZF_FALSE;
        }
    }else if (ch == 3)
    {
        if(PipeBlockCH3.isInit)
        {
            PipeBlockCH3.Dir = block->Dir;
            PipeBlockCH3.RawStep = block->RawStep;
            PipeBlockCH3.isFinish = YZF_FALSE;
        }
    }else if (ch == 4)
    {
        if(PipeBlockCH4.isInit)
        {
            PipeBlockCH4.Dir = block->Dir;
            PipeBlockCH4.RawStep = block->RawStep;
            PipeBlockCH4.isFinish = YZF_FALSE;
        }
    }
    return YZF_OK;
}

YZF_RESULT SpaceSensorArmAsyncLine::Resolve()
{
    activeCount = 0;
    if (PipeBlockCH1.isInit && !PipeBlockCH1.isFinish) {
        activeBlocks[activeCount] = &PipeBlockCH1;
        activeChannels[activeCount] = 1;
        activeCount++;
    }
    if (PipeBlockCH2.isInit && !PipeBlockCH2.isFinish) {
        activeBlocks[activeCount] = &PipeBlockCH2;
        activeChannels[activeCount] = 2;
        activeCount++;
    }
    if (PipeBlockCH3.isInit && !PipeBlockCH3.isFinish) {
        activeBlocks[activeCount] = &PipeBlockCH3;
        activeChannels[activeCount] = 3;
        activeCount++;
    }
    if (PipeBlockCH4.isInit && !PipeBlockCH4.isFinish) {
        activeBlocks[activeCount] = &PipeBlockCH4;
        activeChannels[activeCount] = 4;
        activeCount++;
    }
    if (activeCount == 0) {
        NextStep = 0;
        return YZF_OK;
    }
    int minRemaining = activeBlocks[0]->RawStep;
    for (int i = 1; i < activeCount; i++) {
        int remaining = activeBlocks[i]->RawStep;
        if (remaining < minRemaining) {
            minRemaining = remaining;
        }
    }
    NextStep = minRemaining;
    if (NextStep > 0) {
        return YZF_HANDLER_NOT_FINISH;
    }
    return YZF_OK;
}
YZF_RESULT SpaceSensorArmAsyncLine::PartSend(){
    if (!yz_tim_pwm_with_count_is_finish(timCount)){
        return YZF_ERROR;
    }
    // 构建多通道参数
    int16_t ch[4] ={-1,-1,-1,-1};
    int8_t ch_c=0;
    YZF_TIM_POINTER pwmTim=NULL_PTR;  // 示例
    // 根据活动通道设置对应的PWM通道ID
    for (int i = 0; i < activeCount; i++) {
        switch(activeChannels[i]) {
            case 1:
                ch[ch_c] = LineParams->PipeCH1.PipeParam->ch_id;  // 通道1的PWM输出通道
                if (activeBlocks[i]->Dir == 1){
                    yzf_gpio_set_1(LineParams->PipeCH1.PipeParam->Dir.port,LineParams->PipeCH1.PipeParam->Dir.pin);
                }else{
                    yzf_gpio_set_0(LineParams->PipeCH1.PipeParam->Dir.port,LineParams->PipeCH1.PipeParam->Dir.pin);
                }
                pwmTim = LineParams->PipeCH1.PipeParam->pwm_gen_tim;
                ch_c++;
                break;
            case 2:
                ch[ch_c] = LineParams->PipeCH2.PipeParam->ch_id;
                if (activeBlocks[i]->Dir == 1)
                {
                    yzf_gpio_set_1(LineParams->PipeCH2.PipeParam->Dir.port,LineParams->PipeCH2.PipeParam->Dir.pin);
                }else
                {
                    yzf_gpio_set_0(LineParams->PipeCH2.PipeParam->Dir.port,LineParams->PipeCH2.PipeParam->Dir.pin);
                }
                pwmTim = LineParams->PipeCH2.PipeParam->pwm_gen_tim;
                ch_c++;
                break;
            case 3:
                ch[ch_c] = LineParams->PipeCH3.PipeParam->ch_id;
                if (activeBlocks[i]->Dir == 1)
                {
                    yzf_gpio_set_1(LineParams->PipeCH3.PipeParam->Dir.port,LineParams->PipeCH3.PipeParam->Dir.pin);
                }else
                {
                    yzf_gpio_set_0(LineParams->PipeCH3.PipeParam->Dir.port,LineParams->PipeCH3.PipeParam->Dir.pin);
                }
                pwmTim = LineParams->PipeCH3.PipeParam->pwm_gen_tim;
                ch_c++;
                break;
            case 4:
                ch[ch_c] = LineParams->PipeCH4.PipeParam->ch_id;
                if (activeBlocks[i]->Dir == 1)
                {
                    yzf_gpio_set_1(LineParams->PipeCH4.PipeParam->Dir.port,LineParams->PipeCH4.PipeParam->Dir.pin);
                }else
                {
                    yzf_gpio_set_0(LineParams->PipeCH4.PipeParam->Dir.port,LineParams->PipeCH4.PipeParam->Dir.pin);
                }
                pwmTim = LineParams->PipeCH4.PipeParam->pwm_gen_tim;
                ch_c++;
                break;
            default:
                break;
        }
    }
    int w=(int) roundf((float)LineParams->coreSpeed/LineParams->speed);

    if(w>65535){
        w=65535;
    }

    if(w<2){
        w=2;
    }

    for (int i=0;i<ch_c;i++)
    {
        yz_tim_pwm_data_set(pwmTim,ch[i],w/2,w-1);
    }
    YZF_RESULT result = yz_tim_pwm_with_count_mult_fast(
        pwmTim,      // PWM输出定时器
        ch[0],           // 主通道ID（例如通道1的PWM输出）
        ch[1],         // 通道2的PWM输出（-1表示不使用）
        ch[2],         // 通道3的PWM输出（-1表示不使用）
        ch[3],         // 通道4的PWM输出（-1表示不使用）
        timCount,    // 计数定时器
        NextStep     // 需要发送的脉冲数量
    );

    if (result != YZF_OK) {
        return YZF_ERROR;
    }else{
        for (int i = 0; i < activeCount; i++) {
            activeBlocks[i]->RawStep -= NextStep;
            if (activeBlocks[i]->RawStep == 0) {
                activeBlocks[i]->isFinish = YZF_TRUE;
                if (activeCount == 1)
                {
                    CurrentIndexStep = 0;
                }
            }
        }
    }

    return YZF_OK;
}
YZF_BOOL SpaceSensorArmAsyncLine::isFinish()
{
    return yz_tim_pwm_with_count_is_finish(timCount);
}

YZF_BOOL SpaceSensorArmAsyncLine::isHanded()
{
    return ((Resolve() == YZF_OK)&&isFinish() )? YZF_TRUE : YZF_FALSE;
}

void SpaceSensorArmAsyncLine::act()
{
    if (Resolve()!=YZF_OK)
    {
        if (isFinish())
        {
            PartSend();
        }
    }
}