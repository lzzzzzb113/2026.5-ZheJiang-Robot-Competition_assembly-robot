//
// Created by 32827 on 2026/3/20.
//
#include "RobotCupProgram.hpp"

#include "../../YzSoftPort/Inc/yzl_softport.h"
#include "../../YzSoftPort/Inc/yzl_softport.h"
#include "../Inc/ArmGroup.hpp"
#include "../Inc/Motorclass.hpp"

int Status_CT=0;
uint32_t tick_check=0;
YZF_BOOL isPut = YZF_FALSE;
int Status_PUT = 0;
extern "C" YZF_RESULT robot_main_test() {
    if(Status_PUT==Status_BEGIN&& tick_check + 5000 < armGroup->ProgramTick) {
        tick_check = armGroup->ProgramTick;
        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(90), THE_PUT_Y_DEFAULT(-100),CAMERA_SEE_POINT_Z,0);
        Status_PUT++;
    }else if(Status_PUT==1 && tick_check + 3000 < armGroup->ProgramTick) {
        tick_check = armGroup->ProgramTick;
        if (claw_type==YUAN) {
            motor_r_controller_angle(176);
        }else if (claw_type==SI){
            motor_r_controller_angle(180);
        }else {
            motor_r_controller_angle(180);
        }
        Status_PUT++;
    }else if(Status_PUT==2 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
        tick_check = armGroup->ProgramTick;
        if (claw_type==TI||claw_type==P_TI) {
            armGroup->arm_clawR->setAngle(270);//124
        }
        if (claw_type==SAN||claw_type==P_SAN) {
            armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle()-30.0);
        }
        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(-100),CAMERA_SEE_POINT_Z,0);
        Status_PUT++;
    }else if(Status_PUT==3 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
        if (claw_type==SI) {
            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(3),CAMERA_SEE_POINT_Z,0);
        }
        if (claw_type==SAN||claw_type==TI||claw_type==YUAN) {
            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(0),CAMERA_SEE_POINT_Z,0);
        }
        if (claw_type==P_SAN||claw_type==P_TI) {
            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(0),CAMERA_SEE_POINT_Z,0);
        }
        tick_check = armGroup->ProgramTick;
        Status_PUT++;
    }else if(Status_PUT==4 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
        tick_check = armGroup->ProgramTick;
        if (claw_type==SI) {
            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(3),THE_PUT_Z_DEFAULT(0),0);
        }
        if (claw_type==SAN||claw_type==TI||claw_type==YUAN) {
            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(0),THE_PUT_Z_DEFAULT(0),0);
        }
        if (claw_type==P_SAN||claw_type==P_TI) {
            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(0),THE_PUT_Z_DEFAULT(0),0);
        }
        Status_PUT++;
    }else if(Status_PUT==5 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
        motor_claw_controller(-0.9);
        tick_check = armGroup->ProgramTick;
        Status_PUT++;
    }else if(Status_PUT==6 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(0),CAMERA_SEE_POINT_Z,0);
        tick_check = armGroup->ProgramTick;
        Status_PUT++;
    }else if(Status_PUT==7 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
        if (claw_type==YUAN) motor_r_controller_angle(90);
        if (claw_type==SI) motor_r_controller_angle(270);
        if (claw_type==SAN) motor_r_controller_angle(270);
        if (claw_type == TI) motor_r_controller_angle(270);
        if (claw_type == P_TI) motor_r_controller_angle(90);
        if (claw_type == P_SAN) motor_r_controller_angle(90);

        tick_check = armGroup->ProgramTick;
        Status_PUT++;
    }else if(Status_PUT==8 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
        motor_r_controller(0);
        tick_check = armGroup->ProgramTick;
        Status_PUT=Status_FINISH;
    }
    // }else if(Status_PUT==50 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
    //     tick_check = armGroup->ProgramTick;
    //     motor_r_controller_angle(0);
    //     Status_PUT++;
    // }else if(Status_PUT==51 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
    //     tick_check = armGroup->ProgramTick;
    //     if (claw_type==P_TI) motor_r_controller_angle(291);
    //     if (claw_type==P_SAN) motor_r_controller_angle(240);
    //     Status_PUT++;
    // }else if(Status_PUT==52 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
    //     tick_check = armGroup->ProgramTick;
    //     motor_Stepper_XYZO_setLength(85*ARM_X_2_MM, 2*ARM_Y_2_MM,200*ARM_Z_2_MM,0);
    //     Status_PUT++;
    // }else if(Status_PUT==53 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
    //     tick_check = armGroup->ProgramTick;
    //     motor_Stepper_XYZO_setLength(85*ARM_X_2_MM, 2*ARM_Y_2_MM,73*ARM_Z_2_MM,0);
    //     Status_PUT++;
    // }else if(Status_PUT==53 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
    //     tick_check = armGroup->ProgramTick;
    //     motor_claw_controller(-0.9);
    //     Status_PUT=Status_FINISH;
    // }

    if(Status_PUT==Status_FINISH){
        Status_PUT = Status_BEGIN;
        isPut =YZF_TRUE;
        return YZF_OK;
    }
    return YZF_HANDLER_NOT_FINISH;
}

    int PutCount = 6;
    extern "C" YZF_RESULT robot_main_program_thread()//爪子的延时函数，setlength的调试
    {
        if(Status_CT==Status_BEGIN){
            if (PutCount % 2 == 0) {
                if (conveyor_addLength(1000)) {
                    return YZF_HANDLER_NOT_FINISH;
                }
            }
            motor_claw_controller(-0.9);
            camera_finish = YZF_FALSE;
            yzf_global_handler_add(robot_main_camera_thread);
            armGroup->arm_clawR->setAngle(124*1.5);
            Status_CT++;
        }//0.8是抓，-0.8是松
        if (Status_CT==1&&camera_finish==YZF_TRUE) {        //等待摄像头处理完成
            tick_check = armGroup->ProgramTick;
            armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle()-camera_check_claw_R);
            motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X, CAMERA_Y_OFFSET_SAN,CAMERA_SEE_POINT_Z,0);
            Status_CT++;
        }
        if (Status_CT==2&&check_motions_finished()) {
            tick_check = armGroup->ProgramTick;
            if (claw_type==P_TI||claw_type==P_SAN) {
                motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X, CAMERA_Y_OFFSET_SAN,10*ARM_Z_2_MM,0);

            }else {
                motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X, CAMERA_Y_OFFSET_SAN,83*ARM_Z_2_MM,0);
            }
            Status_CT++;
        }
        if (Status_CT==3&&check_motions_finished()) {
            tick_check = armGroup->ProgramTick;
            motor_claw_controller(0.7);
            if (camera_check_second) Status_CT=50;
            else Status_CT++;
        }
        if(Status_CT==4&&check_motions_finished() && tick_check + 5000 < armGroup->ProgramTick){
            tick_check = armGroup->ProgramTick;
            motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X,CAMERA_Y_OFFSET_SAN,CAMERA_SEE_POINT_Z,0);//x73
            Status_CT++;
        }else if(Status_CT==5&& check_motions_finished()){
            tick_check = armGroup->ProgramTick;
            if (claw_type== TI)   motor_Stepper_XYZO_setLength(84.5*ARM_X_2_MM, 2*ARM_Y_2_MM,CAMERA_SEE_POINT_Z,0);
            if (claw_type== SAN) motor_Stepper_XYZO_setLength(114.5*ARM_X_2_MM, 10*ARM_Y_2_MM,CAMERA_SEE_POINT_Z,0);
            if (claw_type== YUAN)motor_Stepper_XYZO_setLength(98*ARM_X_2_MM, 58*ARM_Y_2_MM,CAMERA_SEE_POINT_Z,0);
            if (claw_type== SI) motor_Stepper_XYZO_setLength(90*ARM_X_2_MM, 58*ARM_Y_2_MM,CAMERA_SEE_POINT_Z,0);
            if (claw_type== P_TI)motor_Stepper_XYZO_setLength(95*ARM_X_2_MM, 0*ARM_Y_2_MM,CAMERA_SEE_POINT_Z,0);
            if (claw_type== P_SAN)motor_Stepper_XYZO_setLength(95*ARM_X_2_MM, 0*ARM_Y_2_MM,CAMERA_SEE_POINT_Z,0);
            armGroup->arm_clawR->setAngle(124*1.5);
            Status_CT ++;
        }else if(Status_CT==6&& check_motions_finished()&&tick_check + 3000 < armGroup->ProgramTick){
            tick_check = armGroup->ProgramTick;
            if (claw_type == YUAN) motor_r_controller_angle(90);
            if (claw_type == SI) motor_r_controller_angle(270);
            if (claw_type == SAN) motor_r_controller_angle(240);
            if (claw_type == TI) motor_r_controller_angle(296);
            if (claw_type == P_SAN) motor_r_controller_angle(62);
            if (claw_type == P_TI) motor_r_controller_angle(116);
            Status_CT++;
        }else if(Status_CT==7&&check_motions_finished()&&tick_check + 3000 < armGroup->ProgramTick){
            tick_check = armGroup->ProgramTick;
            if (claw_type == TI) motor_Stepper_XYZO_setLength(84.5*ARM_X_2_MM, 0*ARM_Y_2_MM,60*ARM_Z_2_MM,0);
            if (claw_type== SAN) motor_Stepper_XYZO_setLength(106.5*ARM_X_2_MM, 10*ARM_Y_2_MM,60*ARM_Z_2_MM,0);
            if (claw_type== YUAN) motor_Stepper_XYZO_setLength(98*ARM_X_2_MM, 58*ARM_Y_2_MM,63*ARM_Z_2_MM,0);//三角形盖子y为0z为0,正方形y为53,Z为63，
            if (claw_type== SI) motor_Stepper_XYZO_setLength(90*ARM_X_2_MM, 58*ARM_Y_2_MM,63*ARM_Z_2_MM,0);//三角形盖子y为0z为0,正方形y为53,Z为63
            if (claw_type == P_TI) motor_Stepper_XYZO_setLength(95*ARM_X_2_MM, 0*ARM_Y_2_MM,0*ARM_Z_2_MM,0);
            if (claw_type == P_SAN) motor_Stepper_XYZO_setLength(90*ARM_X_2_MM, 0*ARM_Y_2_MM,0*ARM_Z_2_MM,0);
            Status_CT++;
        }else if (Status_CT==8&&check_motions_finished()&&tick_check + 4000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            motor_claw_controller(-0.9);
            Status_CT++;
        }else if (Status_CT==9&&check_motions_finished()&&tick_check + 4000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            motor_claw_controller(0.7);
            isPut =YZF_FALSE;
            yzf_global_handler_add(robot_main_test);
            Status_CT++;

        }else if (Status_CT==10&&check_motions_finished()&&isPut) {
            PutCount--;
            Status_CT = Status_BEGIN;
            if (PutCount==0) {
                Status_CT=Status_FINISH;
            }
            return YZF_HANDLER_NOT_FINISH;
        }else if (Status_CT==50&&tick_check + 4000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;

            motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X,CAMERA_Y_OFFSET_SAN,CAMERA_SEE_POINT_Z,0);

            Status_CT++;

        }else if (Status_CT==51&&tick_check + 4000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            armGroup->arm_clawR->setAngle(270);
            Status_CT++;

        }else if (Status_CT==52&&tick_check + 2000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            if (claw_type==TI) motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X, CAMERA_Y_OFFSET_SAN,83*ARM_Z_2_MM,0);
            if (claw_type==P_TI) motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X, CAMERA_Y_OFFSET_SAN,10*ARM_Z_2_MM,0);
            Status_CT++;

        }else if (Status_CT==53&&tick_check + 2000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            motor_claw_controller(-0.9);
            Status_CT++;

        }else if (Status_CT==54&&tick_check + 2000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            if (claw_type==P_TI) motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X, CAMERA_Y_OFFSET_SAN,83*ARM_Z_2_MM,0);
            Status_CT++;
        }else if (Status_CT==55&&tick_check + 2000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            armGroup->arm_clawR->setAngle(90);
            Status_CT++;
        }else if (Status_CT==56&&tick_check + 2000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            if (claw_type==P_TI) motor_Stepper_XYZO_setLength(CAMERA_SEE_POINT_X, CAMERA_Y_OFFSET_SAN,10*ARM_Z_2_MM,0);
            Status_CT++;
        }else if (Status_CT==57&&tick_check + 2000 < armGroup->ProgramTick) {
            tick_check = armGroup->ProgramTick;
            motor_claw_controller(0.7);
            Status_CT=4;
        }
        if(Status_CT==Status_FINISH){
            Status_CT = Status_BEGIN;
            return YZF_OK;
        }
        return YZF_HANDLER_NOT_FINISH;
    }
#define STAGE_MOVE_2_GRID 7963*8+100

extern "C" YZF_RESULT robot_reset_program_thread() {
        // motor_Stepper_XYZO_setLength(0,0,0,1000);
        if (Status_CT == Status_BEGIN) {
           // motor_Stepper_controller(armGroup->Line_oz_Tim8,STAGE_MOVE_2_GRID,0);//2075一格
            if (!conveyor_addLength(1000)) {
                Status_CT=Status_FINISH;
            }
        }
        if (Status_CT == Status_FINISH) {
            Status_CT = Status_BEGIN;
            return YZF_OK;
        }
        return YZF_HANDLER_NOT_FINISH;

    }