#ifndef R320_MOTORCLASS_H
#define R320_MOTORCLASS_H
#include "yz_uart.h"
#include "yz_interrupt.h"
#include "yz_log.h"
#include "../Inc/ArmGroup.hpp"
#include "MainUiView.hpp"
#include "yzl_space_sensor_arm_async_line.hpp"
#include "yzl_space_sensor_arm_plate.hpp"
typedef enum{
    Status_arm_running=666,
    Status_arm_finish=520,
    Status_FINISH=888,
    Status_BEGIN=0,
    Status_ERROR=555,
    Status_arm_init=333,
    Status_arm_waiting=999,
    Status_arm_error=444
}Status;
typedef enum{
    TOO_HIGH=1,
    TOO_LOW=2,
    TOO_CLOSE=3,
    TOO_FAR=4,
    POSITION_CURRENT=5,
    INPUT_ERROR=6,
    FEEDBACK_ON=7,
    FEEDBACK_OFF=8
}limit_sensor;
limit_sensor arm_limit_higher(char HLCF[]);
limit_sensor IR_sensor(char CO[]);
YZF_BOOL conveyor_addLength(uint32_t Length);
void motor_Stepper_setLength(SpaceSensorArmAsyncLine *Line_id_TimX,int length0,int length1);
void motor_Stepper_XYZO_setLength(int lengthX, int lengthY,int lengthZ,int lengthO);
void motor_r_controller_angle(float a);
void motor_r_controller(int32_t length);
void motor_claw_controller(float length);
void motor_Stepper_controller(SpaceSensorArmAsyncLine *Line_id_TimX,int length0,int length1);
YZF_BOOL check_motions_finished();
#endif //R320_MOTORCLASS_H
