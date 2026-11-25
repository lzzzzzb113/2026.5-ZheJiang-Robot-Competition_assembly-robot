//
// Created by 32827 on 2025/5/7.
//

#include "../Inc/ArmGroup.hpp"
/********舵机参数定义********/
SpaceSensorArmPlatePortParam arm_clawR_params={
        {YZF_PIN_8,GPIOD},
        TIM4,
        1,
        19999,
        2499,
        499,
        999,
        270.0f,

        RotateTypeX
};
/********步进电机参数定义*********/
SpaceSensorArmPolePortParam pole_null_params={
        {YZF_PIN_UNKnow,NULL_PTR},
        NULL_PTR,
        0xFFFF,
        NULL_PTR,
        0,
        0,
        0,
        0,
        1000000,
        0
};
SpaceSensorArmPolePortParam arm_x_params={
        {YZF_PIN_10,GPIOE},
        TIM3,
        1,
        TIM1,
        2000,
        0,
        0,
        2000,
        1000000,
        1000,   //1cm 参数缩放用于setLength()函数计算实际距离
};
SpaceSensorArmPolePortParam arm_y_params={
        {YZF_PIN_15,GPIOB},
        TIM3,
        2,
        TIM1,
        2000,
        0,
        0,
        2000,
        1000000,
        1000,   //1cm 参数缩放用于setLength()函数计算实际距离
};
SpaceSensorArmPolePortParam arm_z_params={
        {YZF_PIN_14,GPIOB},
        TIM2,
        3,
        TIM8,
        2000,
        0,
        0,
        2000,
        1000000,
        1000,   //1cm 参数缩放用于setLength()函数计算实际距离
};
SpaceSensorArmPolePortParam arm_oustage_params={
        {YZF_PIN_12,GPIOE},
        TIM2,
        4,
        TIM8,
        1000000,
        -1000000,
        0,
        2000,
        1000000,
        1000,   //1cm 参数缩放用于setLength()函数计算实际距离
};
ArmAsyncLineParams tim1_xy_params={
    .PipeCH1 = {
        .isUse = YZF_TRUE,
        .PipeParam = &arm_x_params
    },
    .PipeCH2 = {
        .isUse = YZF_TRUE,
        .PipeParam = &arm_y_params
    },
    .PipeCH3 = {
        .isUse = YZF_FALSE,
        .PipeParam = &pole_null_params
    },
    .PipeCH4 = {
        .isUse = YZF_FALSE,
        .PipeParam = &pole_null_params
    },
    .speed = 1500,
    .coreSpeed = 1000000,
    .InstanceHandler = NULL_PTR
};

ArmAsyncLineParams tim8_oz_params={
    .PipeCH1 = {
        .isUse = YZF_TRUE,
        .PipeParam = &arm_oustage_params
    },
    .PipeCH2 = {
        .isUse = YZF_TRUE,
        .PipeParam = &arm_z_params
    },
    .PipeCH3 = {
        .isUse = YZF_FALSE,
        .PipeParam = &pole_null_params
    },
    .PipeCH4 = {
        .isUse = YZF_FALSE,
        .PipeParam = &pole_null_params
    },
    .speed = 2000,
    .coreSpeed = 1000000,
    .InstanceHandler = NULL_PTR
};

ArmGroup::ArmGroup() {

    Line_xy_Tim1 = new SpaceSensorArmAsyncLine(&tim1_xy_params,TIM1);
    Line_oz_Tim8 = new SpaceSensorArmAsyncLine(&tim8_oz_params,TIM8);

    Pose *defaultPose = new Pose();
    arm_clawR = new SpaceSensorArmPlate(&arm_clawR_params,*defaultPose,YZF_NULL_PTR,YZF_TRUE);

    arm_clawR->setAngle(124*1.5f);

    delete defaultPose;
}

ArmGroup::~ArmGroup() {

}

void ArmGroup::addParams(int index,float timeUnit) {

}

void ArmGroup::delParams(int index,float timeUnit) {

}

uint32_t ArmGroup::ProgramTick = 0;
YZF_BOOL ArmGroup::isLoadTickHandler = YZF_FALSE;

YZF_RESULT ArmGroup::ProgramTickHandler(uint32_t tick) {
    ProgramTick+=tick;
    return YZF_HANDLER_NOT_FINISH;
}

void ArmGroup::act() {
    Line_xy_Tim1->act();
    Line_oz_Tim8->act();
}

void ArmGroup::Init() {
    yz_heart_process_add(ProgramTickHandler);
    isLoadTickHandler = YZF_TRUE;
}
void ArmGroup::getEnd() {

}
void ArmGroup::OutEnable()
{
    arm_clawR->EnableOut();
}
void ArmGroup::OutDisable()
{

}

float SpaceResultY1[4];
float *ArmGroup::compile() {
    return SpaceResultY1;
}
