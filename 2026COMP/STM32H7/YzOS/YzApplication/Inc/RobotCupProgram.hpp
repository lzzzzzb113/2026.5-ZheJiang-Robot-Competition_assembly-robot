//
// Created by 32827 on 2026/3/20.
//

#ifndef STM32H723_LTS_BSP_ROBOTCUPPROGRAM_H
#define STM32H723_LTS_BSP_ROBOTCUPPROGRAM_H

#include "yzx_base.h"
#include "ArmPoseRef.hpp"

extern "C" YZF_RESULT robot_main_program_thread();
extern "C" YZF_RESULT robot_reset_program_thread();
extern ArmGroup *armGroup;

#endif //STM32H723_LTS_BSP_ROBOTCUPPROGRAM_H