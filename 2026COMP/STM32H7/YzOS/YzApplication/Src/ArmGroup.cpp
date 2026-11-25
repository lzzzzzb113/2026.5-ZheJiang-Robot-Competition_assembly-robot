//
// Created by 32827 on 2025/5/7.
//

#include "../Inc/ArmGroup.hpp"

SpaceSensorArmPolePortParam arm07_params={
        {YZF_PIN_9,GPIOD},
        TIM4,
        2,
        TIM8,
        90,
        -90,
        0,

        3200,
        1000000,
        850*8,   //1cm 参数缩放用于setLength()函数计算实际距离
};

SpaceSensorArmPolePortParam arm08_params={
        {YZF_PIN_10,GPIOD},
        TIM4,
        3,
        TIM8,
        26,
        0,
        13,

        3200,
        1000000,
        520,   //1cm 参数缩放用于setLength()函数计算实际距离
};

SpaceSensorArmPolePortParam arm09_params={
        {YZF_PIN_8,GPIOD},
        TIM4,
        1,
        TIM8,
        360,
        0,
        180,

        800,
        1000000,
        210,   //1mm 参数缩放用于setLength()函数计算实际距离
};

SpaceSensorArmPolePortParam arm10_params={
    {YZF_PIN_11,GPIOD},
    TIM4,
    4,
    TIM8,
    360,
    0,
    180,

    800,
    1000000,
   16,   //1mm 参数缩放用于setLength()函数计算实际距离
};

SpaceSensorArmPlatePortParam  arm01_params={
        {YZF_PIN_2,GPIOA},
        TIM2,
        3,
        19999,
        2499,
        499,
        800,
        180.0f,

        RotateTypeZ
};

SpaceSensorArmPlatePortParam  arm02_params={
        {YZF_PIN_11,GPIOE},
        TIM1,
        2,
        19999,
        2499,
        499,
        632,
        180.0f,

        RotateTypeX
};

SpaceSensorArmPlatePortParam  arm03_params={
        {YZF_PIN_13,GPIOE},
        TIM1,
        3,
        19999,
        2499,
        499,
        999,
        180.0f,

        RotateTypeX
};

SpaceSensorArmPlatePortParam arm04_params={
        {YZF_PIN_11,GPIOB},
        TIM2,
        4,
        19999,
        2499,
        499,
        1999,
        180.0f,

        RotateTypeX
};

SpaceSensorArmPlatePortParam arm05_params={
        {YZF_PIN_6,GPIOC},
        TIM3,
        1,
        19999,
        2499,
        499,
        1776,
        180.0f,

        RotateTypeX
};

SpaceSensorArmPlatePortParam arm06_params={
        {YZF_PIN_7,GPIOC},
        TIM3,
        2,
        19999,
        2499,
        499,
        499,
        180.0f,

        RotateTypeX
};
ArmGroup::ArmGroup() {
    Position *defPos;//存储空间坐标位置（x,y,z）
    Pose *defPose;//存储空间姿态（位置 + 欧拉角 / 旋转角）
    defPos=new Position(0,0,0);
    defPose=new Pose(defPos,0,0,0);//初始的俯仰 / 偏航 / 滚转角度

    yz_info("Arm07...\n");
    arm07 = new SpaceSensorArmPole(&arm07_params,*defPose,YZF_NULL_PTR,YZF_TRUE);

    yz_info("Arm08...\n");
    arm08 = new SpaceSensorArmPole(&arm08_params,*defPose,YZF_NULL_PTR,YZF_TRUE);

    yz_info("Arm09...\n");
    arm09 = new SpaceSensorArmPole(&arm09_params,*defPose,YZF_NULL_PTR,YZF_TRUE);

    yz_info("Arm10...\n");
    arm10 = new SpaceSensorArmPole(&arm10_params,*defPose,YZF_NULL_PTR,YZF_TRUE);

    yz_info("Arm01...\n");
    arm01 = new SpaceSensorArmPlate(&arm01_params,*defPose,YZF_NULL_PTR,YZF_TRUE);

    yz_info("Arm02...\n");
    arm02 = new SpaceSensorArmPlate(&arm02_params,*defPose,arm01,YZF_FALSE);

    yz_info("Arm03...\n");
    arm03 = new SpaceSensorArmPlate(&arm03_params,*defPose,arm02,YZF_FALSE);

    yz_info("Arm04...\n");
    arm04 = new SpaceSensorArmPlate(&arm04_params,*defPose,arm03,YZF_FALSE);

    yz_info("Arm05...\n");
    arm05 = new SpaceSensorArmPlate(&arm05_params,*defPose,arm04,YZF_FALSE);

    yz_info("Arm06...\n");
    arm06 = new SpaceSensorArmPlate(&arm06_params,*defPose,arm05,YZF_FALSE);

    delete defPose; //之后不用记得删除 释放内存
    delete defPos;
}

ArmGroup::~ArmGroup() {
    delete arm06;
    delete arm05;
    delete arm04;
    delete arm03;
    delete arm02;
    delete arm01;
}

void ArmGroup::addParams(int index,float timeUnit) {
    if(index==0){
        arm01->addAngle(15.0f*timeUnit);
    }
    if(index==1){
        arm02->addAngle(15.0f*timeUnit);
    }
    if(index==2){
        arm03->addAngle(15.0f*timeUnit);
        arm04->setAngle(180.0f-arm03->getAngle());
    }
    if(index==3){
        arm04->addAngle(15.0f*timeUnit);
        arm03->setAngle(180.0f-arm04->getAngle());
    }
    if(index==4){
        arm05->addAngle(15.0f*timeUnit);
    }
    if(index==5){
        arm06->addAngle(15.0f*timeUnit);
    }
    if(index==6){
        arm07->addLength(10.0f*timeUnit);
    }
}

void ArmGroup::delParams(int index,float timeUnit) {
    if(index==0){
        arm01->delAngle(15.0f*timeUnit);
    }
    if(index==1){
        arm02->delAngle(15.0f*timeUnit);
    }
    if(index==2){
        arm03->delAngle(15.0f*timeUnit);
        arm04->setAngle(180.0f-arm03->getAngle());//arm03 与 arm04 保持互补角度，总和恒为 180°，实现机械结构的对称联动。
    }
    if(index==3){
        arm04->delAngle(15.0f*timeUnit);
        arm03->setAngle(180.0f-arm04->getAngle());
    }
    if(index==4){
        arm05->delAngle(15.0f*timeUnit);
    }
    if(index==5){
        arm06->delAngle(15.0f*timeUnit);
    }
    if(index==6){
        arm07->delLength(10*timeUnit);
    }
}

uint32_t ArmGroup::ProgramTick = 0;
YZF_BOOL ArmGroup::isLoadTickHandler = YZF_FALSE;

YZF_RESULT ArmGroup::ProgramTickHandler(uint32_t tick) {
    ProgramTick+=tick;
    return YZF_HANDLER_NOT_FINISH;
}

void ArmGroup::act() {
    arm07->act(ProgramTick);
    arm08->act(ProgramTick);
    arm09->act(ProgramTick);
    arm01->act(ProgramTick);
    arm02->act(ProgramTick);
    arm03->act(ProgramTick);
    arm04->act(ProgramTick);
    arm05->act(ProgramTick);
    arm06->act(ProgramTick);
}

void ArmGroup::Init() {
    yz_heart_process_add(ProgramTickHandler);
    isLoadTickHandler = YZF_TRUE;
}

void ArmGroup::getEnd() {

}
void ArmGroup::OutEnable()
{
    arm01->EnableOut();
    arm02->EnableOut();
    arm03->EnableOut();
    arm04->EnableOut();
    arm05->EnableOut();
    arm06->EnableOut();
}
void ArmGroup::OutDisable()
{
    arm01->DisableOut();
    arm02->DisableOut();
    arm03->DisableOut();
    arm04->DisableOut();
    arm05->DisableOut();
    arm06->DisableOut();
}

float SpaceResultY1[4];//[x, y, z, theta]（三维坐标 + 旋转角度）

float arm01_fix=8.280f;//底座 / 大臂关节固定偏移（角度或长度）
float arm01_len=25.0f;//第一段臂杆长度
float arm02_fix=1.0f;
float arm02_len=17.5f;
float arm03_fix=90.0f;
float arm04_fix=40.0f;
float arm0201_len_cos;// 缓存 arm02 与 arm01 构成三角形的 cos 值
float arm0201_len_sin;// 缓存 arm02 与 arm01 构成三角形的 sin 值
float arm0201_len; // 缓存计算出的临时边长
float arm0201_up_angle;// 缓存计算出的临时角度
float arm0201_ground_len;// 缓存地面投影长度

float *ArmGroup::compile() {
    float bottom_angle=arm01->getAngle()-arm01_fix;
    float bottom_cos=COS(bottom_angle); // 底座旋转的X轴余弦分量
    float bottom_sin=SIN(bottom_angle);// 底座旋转的Y轴正弦分量

    float up_angle=arm02->getAngle()-arm02_fix;
    float up_cos=COS(up_angle);
    float up_sin=SIN(up_angle);

    float up_dir_angle=arm03_fix-(arm03->getAngle()-180.0f);
    float up_dir_cos=COS(up_dir_angle);
    float up_dir_sin=SIN(up_dir_angle);

    arm0201_len_cos = arm01_len-arm02_len*up_dir_cos;
    arm0201_len_sin = arm02_len*up_dir_sin;
    arm0201_len=sqrt(arm0201_len_cos*arm0201_len_cos+arm0201_len_sin*arm0201_len_sin);
    arm0201_up_angle=up_angle-yzl_atan2_tab_calc(arm0201_len_sin,arm0201_len_cos);//TODO : need div PI

    SpaceResultY1[2] = arm0201_len * SIN(arm0201_up_angle); // Y
    arm0201_ground_len = arm0201_len * COS(arm0201_up_angle); // 2D Len 地映射
    SpaceResultY1[0] = arm0201_ground_len * bottom_cos;
    SpaceResultY1[1] = arm0201_ground_len * bottom_sin;
    SpaceResultY1[3] = 220.0f-arm03->getAngle()+up_angle;
    return SpaceResultY1;
}
