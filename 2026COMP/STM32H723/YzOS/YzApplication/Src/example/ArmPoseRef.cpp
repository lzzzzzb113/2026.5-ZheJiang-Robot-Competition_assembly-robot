//
// Created by 32827 on 2025/5/12.
//

#include "../../Inc/example/ArmPoseRef.hpp"

ArmPoseRef::ArmPoseRef(ArmGroup *armGroup) {
    this->armGroup=armGroup;
}

ArmPoseRef::~ArmPoseRef() {

}

void ArmPoseRef::Capture() {
    arm_data = armGroup->arm_clawR_params->getAngle();
}

void ArmPoseRef::Restore(int time) {
    armGroup->arm_clawR_params->addAction(new SpaceSensorActionAngle(armGroup->ProgramTick,time,armGroup->arm01->getAngle(),arm_data[0]));
}

YZF_BOOL ArmPoseRef::IsRestore() {
    if(armGroup->arm_clawR_params->isActionFinish()){
        return YZF_TRUE;
    }else{
        return YZF_FALSE;
    }
}

void ArmPoseRef::act() {
    isFinish = YZF_TRUE;
}

ArmPoseRef::ArmPoseRef(float *arm_data, ArmGroup *armGroup) {
    memcpy(this->arm_data,arm_data,sizeof(float)*7);
    this->armGroup = armGroup;
}
float arm_deta[7]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
void ArmPoseRef::SetData(float *pArm_data) {
    for (int i = 0; i < 7; ++i) {
        arm_data[i]=pArm_data[i]-arm_deta[i];
    }
    //TODO: Need Remove
    arm_data[3]=180.0f-arm_data[2];
}
