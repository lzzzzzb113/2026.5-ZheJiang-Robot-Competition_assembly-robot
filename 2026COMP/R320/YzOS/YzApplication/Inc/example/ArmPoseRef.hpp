//
// Created by 32827 on 2025/5/12.
//

#ifndef STM32H723_LTS_BSP_ARMPOSEREF_HPP
#define STM32H723_LTS_BSP_ARMPOSEREF_HPP

#include "../ArmGroup.hpp"
#include "yzl_space_sensor_action_angle.hpp"


/**
 * 机械臂位置参考类，实现机械臂的快照功能
 */
class ArmPoseRef {
    public:
        float arm_data[7];
        ArmGroup *armGroup;
        ArmPoseRef(ArmGroup *armGroup);
        ArmPoseRef(float arm_data[7], ArmGroup *armGroup);
        ~ArmPoseRef();

        YZF_BOOL isFinish = YZF_FALSE;
        void Capture();
        void Restore(int time);
        void SetData(float *pArm_data);
        void act();
        YZF_BOOL IsRestore();
};
#endif //STM32H723_LTS_BSP_ARMPOSEREF_HPP
