//
// Created by 32827 on 2025/4/24.
//

#include "yzl_space_sensor_arm_plate.hpp"
#include "yz_tim.h"
SpaceSensorArmPlate::SpaceSensorArmPlate(SpaceSensorArm *parent, YZF_TIM_POINTER tim, uint16_t ch_id, Pose defaultPose,
                                         SpaceSensorArmRotateType type,int pwm_auto_reload, int maxParam, int minParam,
                                         float maxAngle,int defaultParam,YZF_BOOL isRoot) {
    parentArm = parent;
    ArmPoseBack = defaultPose;
    this->tim = tim;
    this->ch_id = ch_id;
    minLimit = minParam;
    maxLimit = maxParam;
    nowParam = defaultParam;
    this->isRoot = isRoot;
    rotateType = type;
    this->maxAngle = maxAngle;
    this->pwm_auto_reload = pwm_auto_reload;

    yz_tim_pwm_data_set(tim,ch_id,nowParam,pwm_auto_reload);

    ArmPose=ArmPoseBack;

    switch (rotateType) {       //更新姿态数据
        case RotateTypeX:
            ArmPose.RotateX(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        case RotateTypeY:
            ArmPose.RotateY(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        case RotateTypeZ:
            ArmPose.RotateZ(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        default:
        break;
    }
}

float SpaceSensorArmPlate::setAngle(float angle) {
    int param = minLimit+(int)((angle/maxAngle)*(float )(maxLimit-minLimit));
    if(param<minLimit){
        param = minLimit;
    }else if(param>maxLimit){
        param = maxLimit;
    }
    nowParam = param;
    yz_tim_pwm_data_set(tim,ch_id,nowParam,pwm_auto_reload);
#if YZSCREEN
    if(hasWidget){
        memset(transmit_buf,0,64);
        sprintf(transmit_buf,"%.3f",getAngle());
        widgetText->setText((utf8 *)transmit_buf);
    }
#endif

    ArmPose=ArmPoseBack;
    switch (rotateType) {
        case RotateTypeX:
            ArmPose.RotateX(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        case RotateTypeY:
            ArmPose.RotateY(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        case RotateTypeZ:
            ArmPose.RotateZ(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        default:
            break;
    }

    return ((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle;
}

Position SpaceSensorArmPlate::getPosition(SpaceSensorArm *ArmEnd) {
    //TODO: MAY RAM LEAK
    // 可能存在内存泄露，暂时不适用
    Position temp(0,0,0);
    SpaceSensorArm *tempArm=this;
    if(ArmEnd==this){
        return *(ArmPose * temp);
    }
    while(1){
        if(tempArm->isRoot==YZF_TRUE){
            return temp;
        }else if(parentArm!=YZF_NULL_PTR){
            tempArm=parentArm;
            Position *p=(tempArm->ArmPose * temp);
            temp = *p;
            delete p;
        }else{
            return temp;
        }
    }
}

float SpaceSensorArmPlate::getAngle() {
    return ((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle;
}

void SpaceSensorArmPlate::addAction(SpaceSensorAction *action) {
    if(action->getType()==ACTION_TYPE_ANGLE) {
        SpaceSensorArm::addAction(action);
    }else{
        delete action;
    }
}

void SpaceSensorArmPlate::EnableOut()
{
    yz_tim_pwm_enable_ch(this->tim,this->ch_id);
}

void SpaceSensorArmPlate::DisableOut()
{
    yz_tim_pwm_disable_ch(this->tim,this->ch_id);
}

#if YZSCREEN
void SpaceSensorArmPlate::LinkTextUI(WidgetTextButton *pWidgetText) {
    this->widgetText = pWidgetText;
    hasWidget = YZF_TRUE;
    memset(transmit_buf,0,64);
    sprintf(transmit_buf,"%.3f",getAngle());
    widgetText->setText((utf8 *)transmit_buf);
}

void SpaceSensorArmPlate::CancelLink() {
    hasWidget = YZF_FALSE;
}

SpaceSensorArmPlate::SpaceSensorArmPlate(SpaceSensorArmPlatePortParam *param, Pose defaultPose, SpaceSensorArm *parent,
                                         YZF_BOOL isRoot) {
    parentArm = parent;
    ArmPoseBack = defaultPose;
    this->isRoot = isRoot;

    tim = param->pwm_gen_tim;
    ch_id = param->pwm_ch_id;
    minLimit = param->pwm_compare_min;
    maxLimit = param->pwm_compare_max;
    nowParam = param->pwm_compare_default;
    rotateType = param->type;
    maxAngle = param->maxAngle;
    pwm_auto_reload = param->pwm_auto_reload;

    yz_tim_pwm_data_set(tim,ch_id,nowParam,pwm_auto_reload);

    ArmPose=ArmPoseBack;

    switch (rotateType) {
        case RotateTypeX:
            ArmPose.RotateX(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        case RotateTypeY:
            ArmPose.RotateY(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        case RotateTypeZ:
            ArmPose.RotateZ(((float )(nowParam-minLimit)/(float )(maxLimit-minLimit))*maxAngle);
            break;
        default:
            break;
    }
}

#endif


