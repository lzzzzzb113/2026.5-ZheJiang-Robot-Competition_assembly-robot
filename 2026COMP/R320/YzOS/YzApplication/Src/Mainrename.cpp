//
// Created by 18744 on 2026/5/19.
//

#include "../Inc/Mainrename.hpp"
///*=========内部物料台到外部物料台的夹取====================*/
//extern "C" YZF_RESULT robot_first_In_test() {
//    if (Status_PUT == Status_BEGIN && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_RELEASE);
//        motor_r_controller_angle(89+r_correct_putin_stage[yuan]); //YUAN
//        Status_PUT++;
//    } else if (Status_PUT == 1 && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((100+x_correct_putin_stage[yuan])*ARM_X_2_MM, (65+y_correct_putin_stage[yuan])*ARM_Y_2_MM,camera_see_point_z,RED_YUAN);//YUAN
//        Status_PUT++;
//    } else if (Status_PUT == 2 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(
//                (100+x_correct_putin_stage[yuan])*ARM_X_2_MM, (65+y_correct_putin_stage[yuan])*ARM_Y_2_MM, (78+y_correct_putin_stage[yuan] )* ARM_Z_2_MM, RED_YUAN); //YUAN
//        Status_PUT++;
//    } else if (Status_PUT == 3 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_CATCH);
//        Status_PUT++;
//    } else if (Status_PUT == 4 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((100+x_correct_putin_stage[yuan]) * ARM_X_2_MM, (65+y_correct_putin_stage[yuan]) * ARM_Y_2_MM, camera_see_point_z, RED_YUAN); //YUAN
//        Status_PUT++;
//    } else if (Status_PUT == 5 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        motor_r_controller_angle(179+r_correct_putin_wheel[yuan]);
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 6 && check_motions_finished() && tick_check + 1000 < armGroup->ProgramTick) {
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[yuan]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[yuan]), camera_see_point_z, RED_YUAN); //YUAN
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 7 && check_motions_finished() && tick_check + 1000 < armGroup->ProgramTick) {
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[yuan]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[yuan]),THE_PUT_Z_DEFAULT(z_correct_putin_wheel[yuan]), RED_YUAN);
//        //YUAN
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 8 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        arm_put_shake(179+r_correct_putin_wheel[yuan],5,THE_PUT_Z_DEFAULT(z_correct_putin_wheel[yuan]), 0, RED_YUAN, 400);
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 9 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_RELEASE);
//        Status_PUT++;
//    } else if (Status_PUT == 10 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[yuan]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[yuan]), camera_see_point_z, RED_YUAN); //YUAN
//        Status_PUT++;
//    }
//        //方形夹取
//    else if (Status_PUT == 11 && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_r_controller_angle(269+r_correct_putin_stage[si]);
//        Status_PUT++;
//    } else if (Status_PUT == 12 && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((74+x_correct_putin_stage[si]) * ARM_X_2_MM, (55+y_correct_putin_stage[si]) * ARM_Y_2_MM, camera_see_point_z, BLUE_SI); //SI
//        Status_PUT++;
//    } else if (Status_PUT == 13 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((74+x_correct_putin_stage[si]) * ARM_X_2_MM, (55+y_correct_putin_stage[si]) * ARM_Y_2_MM, (90+z_correct_putin_stage[si]) * ARM_Z_2_MM, BLUE_SI); //SI
//        Status_PUT++;
//    } else if (Status_PUT == 14 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_CATCH);
//        Status_PUT++;
//    } else if (Status_PUT == 15 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((74+x_correct_putin_stage[si]) * ARM_X_2_MM, (55+y_correct_putin_stage[si])* ARM_Y_2_MM, camera_see_point_z, BLUE_SI); //SI
//        Status_PUT++;
//    } else if (Status_PUT == 16 && check_motions_finished() && tick_check + 1000 < armGroup->ProgramTick) {
//        motor_r_controller_angle(181+r_correct_putin_wheel[si]);
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 17 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[si]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[si]), camera_see_point_z, BLUE_SI); //SI
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 18 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[si]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[si]),THE_PUT_Z_DEFAULT(z_correct_putin_wheel[si]), BLUE_SI); //SI
//        Status_PUT++;
//    } else if (Status_PUT == 19 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        arm_put_shake(181+r_correct_putin_wheel[si],6,THE_PUT_Z_DEFAULT(z_correct_putin_wheel[si]), 0, BLUE_SI, 400);
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_RELEASE);
//        Status_PUT++;
//    } else if (Status_PUT == 20 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[si]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[si]), camera_see_point_z, BLUE_SI); //SI
//        Status_PUT++;
//    } else if (Status_PUT == 21 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_r_controller_angle(239+r_correct_putin_stage[san]); //SI
//        Status_PUT++;
//    }
//        //三角形夹取
//    else if (Status_PUT == 22 && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((95.5+x_correct_putin_stage[san]) * ARM_X_2_MM, (6+y_correct_putin_stage[san]) * ARM_Y_2_MM, camera_see_point_z, RED_SAN); //SAN
//        Status_PUT++;
//    } else if (Status_PUT == 23 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((95.5+x_correct_putin_stage[san]) * ARM_X_2_MM, (6+y_correct_putin_stage[san]) * ARM_Y_2_MM, (90+z_correct_putin_stage[san]) * ARM_Z_2_MM, RED_SAN); //SAN
//        Status_PUT++;
//    } else if (Status_PUT == 24 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_CATCH);
//        Status_PUT++;
//    } else if (Status_PUT == 25 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((95.5+x_correct_putin_stage[san]), (6+y_correct_putin_stage[san]) * ARM_Y_2_MM, camera_see_point_z, RED_SAN); //SAN
//        Status_PUT++;
//    } else if (Status_PUT == 26 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        motor_r_controller_angle(178.0+r_correct_putin_wheel[san]);
//        tick_check = armGroup->ProgramTick;
//        Status_PUT = 28;
//    } else if (Status_PUT == 28 && check_motions_finished() && tick_check + 1000 < armGroup->ProgramTick) {
//        armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle() + 90.0);
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 29 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[san]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[san]), camera_see_point_z, RED_SAN); //SAN
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 30 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[san]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[san]),THE_PUT_Z_DEFAULT(z_correct_putin_wheel[san]), RED_SAN); //SI
//        Status_PUT++;
//    } else if (Status_PUT == 31 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        arm_put_shake(178.0+r_correct_putin_wheel[san], 8,THE_PUT_Z_DEFAULT(z_correct_putin_wheel[san]), 5, RED_SAN, 400);
//        Status_PUT++;
//    } else if (Status_PUT == 32 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_RELEASE);
//        Status_PUT++;
//    } else if (Status_PUT == 33 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle() - 90.0);
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[san]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[san]), camera_see_point_z, RED_SAN); //SAN
//        Status_PUT++;
//    }
//    //三角形盖子
//    if (isPut_PS) {
//        if (Status_PUT == 34 && tick_check + 3000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_r_controller_angle(62+r_correct_putin_stage[ps]);
//            Status_PUT = 36;
//        } else if (Status_PUT == 36 && tick_check + 1500 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle() - 30.0);
//            motor_Stepper_XYZO_setLength((80+x_correct_putin_stage[ps]) * ARM_X_2_MM, y_correct_putin_stage[ps] * ARM_Y_2_MM, camera_see_point_z, RED_SAN); //P_SAN
//            Status_PUT++;
//        } else if (Status_PUT == 37 && check_motions_finished() && tick_check + 1500 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength((80+x_correct_putin_stage[ps]) * ARM_X_2_MM, y_correct_putin_stage[ps] * ARM_Y_2_MM, (55+z_correct_putin_stage[ps]) * ARM_Z_2_MM, RED_SAN); //P_SAN
//            Status_PUT++;
//        } else if (Status_PUT == 38 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_claw_controller(CLAW_CATCH);
//            Status_PUT++;
//        } else if (Status_PUT == 39 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength((80+x_correct_putin_stage[ps]) * ARM_X_2_MM, y_correct_putin_stage[ps] * ARM_Y_2_MM, camera_see_point_z, RED_SAN); //P_SAN
//            Status_PUT++;
//        } else if (Status_PUT == 40 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            motor_r_controller_angle(178+r_correct_putin_wheel[ps]);
//            tick_check = armGroup->ProgramTick;
//            Status_PUT++;
//        } else if (Status_PUT == 41 && check_motions_finished()) {
//            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[ps]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[ps]), camera_see_point_z, RED_SAN);
//            //P_SAN
//            tick_check = armGroup->ProgramTick;
//            Status_PUT++;
//        } else if (Status_PUT == 42 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[ps]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[ps]),THE_PUT_Z_DEFAULT(50+z_correct_putin_wheel[ps]),
//                                         RED_SAN); //P_TI
//            Status_PUT++;
//        } else if (Status_PUT == 43 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            arm_put_shake(178+r_correct_putin_wheel[ps], 4,THE_PUT_Z_DEFAULT(y_correct_putin_wheel[ps]), 5, RED_SAN, 400);
//            Status_PUT++;
//        } else if (Status_PUT == 44 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_claw_controller(CLAW_RELEASE);
//            Status_PUT++;
//        } else if (Status_PUT == 45 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle() + 30.0);
//            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[ps]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[ps]), camera_see_point_z, RED_SAN);
//            //P_SAN
//            Status_PUT = 47;
//        }
//    } else if (Status_PUT == 34 && tick_check + 3000 < armGroup->ProgramTick) {
//        Status_PUT = 47;
//    }
//
//    //梯形夹取
//    if (Status_PUT == 47 && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_r_controller_angle(293+r_correct_putin_stage[ti]); //TI
//        Status_PUT++;
//    } else if (Status_PUT == 48 && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((78.0+x_correct_putin_stage[ti]) * ARM_X_2_MM, (1+y_correct_putin_stage[ti]) * ARM_Y_2_MM, camera_see_point_z, RED_TI); //TI
//        Status_PUT++;
//    } else if (Status_PUT == 49 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((78.0+x_correct_putin_stage[ti]) * ARM_X_2_MM, (1+y_correct_putin_stage[ti]) * ARM_Y_2_MM, (90+z_correct_putin_stage[si]) * ARM_Z_2_MM, RED_TI); //TI
//        Status_PUT++;
//    } else if (Status_PUT == 50 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_CATCH);
//        Status_PUT++;
//    } else if (Status_PUT == 51 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength((78.0+x_correct_putin_stage[ti]) * ARM_X_2_MM, (1+y_correct_putin_stage[ti]) * ARM_Y_2_MM, camera_see_point_z, RED_TI); //TI
//        Status_PUT++;
//    } else if (Status_PUT == 52 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//
//        motor_r_controller_angle(180+r_correct_putin_wheel[ti]);
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 53 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        armGroup->arm_clawR->setAngle(269);
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 54 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[ti]), THE_PUT_Y_DEFAULT(1+y_correct_putin_wheel[ti]), camera_see_point_z, RED_TI); //TI
//        tick_check = armGroup->ProgramTick;
//        Status_PUT++;
//    } else if (Status_PUT == 55 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[ti]), THE_PUT_Y_DEFAULT(1+y_correct_putin_wheel[ti]),THE_PUT_Z_DEFAULT(20+z_correct_putin_wheel[ti]), RED_TI); //TI
//        Status_PUT++;
//    } else if (Status_PUT == 56 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        arm_put_shake(180+r_correct_putin_wheel[ti], 8,THE_PUT_Z_DEFAULT(z_correct_putin_wheel[ti]) , 5, RED_TI, 400);
//        Status_PUT++;
//    } else if (Status_PUT == 57 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_RELEASE);
//        Status_PUT++;
//    } else if (Status_PUT == 58 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_wheel[ti]), THE_PUT_Y_DEFAULT(y_correct_putin_wheel[ti]), camera_see_point_z, RED_TI); //TI
//        Status_PUT = 61;
//    }
//    //梯形盖子
//    if (isPut_PT) {
//        if (Status_PUT == 61 && tick_check + 3000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_r_controller_angle(116+r_correct_putin_stage[pt]);
//            armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle() - 90.0);
//            Status_PUT++;
//        } else if (Status_PUT == 62 && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength((90+x_correct_putin_stage[pt]) * ARM_X_2_MM, (14+y_correct_putin_stage[pt]) * ARM_Y_2_MM, camera_see_point_z, RED_TI);
//            Status_PUT++;
//        } else if (Status_PUT == 63 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength((90+x_correct_putin_stage[pt]) * ARM_X_2_MM, (14+y_correct_putin_stage[pt]) * ARM_Y_2_MM, (55+z_correct_putin_stage[pt]) * ARM_Z_2_MM, RED_TI);
//            Status_PUT++;
//        } else if (Status_PUT == 64 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_claw_controller(CLAW_CATCH);
//            Status_PUT++;
//        } else if (Status_PUT == 65 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength((90+x_correct_putin_stage[pt]) * ARM_X_2_MM, (14+y_correct_putin_stage[pt]) * ARM_Y_2_MM, camera_see_point_z, RED_TI);
//            Status_PUT++;
//        } else if (Status_PUT == 66 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            motor_r_controller_angle(178+r_correct_putin_wheel[pt]);
//            tick_check = armGroup->ProgramTick;
//            Status_PUT++;
//        } else if (Status_PUT == 67 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle() + 90.0);
//            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_stage[pt]), THE_PUT_Y_DEFAULT(y_correct_putin_stage[pt]+4), camera_see_point_z, RED_TI); //P_TI
//            tick_check = armGroup->ProgramTick;
//            Status_PUT++;
//        } else if (Status_PUT == 68 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_stage[pt]), THE_PUT_Y_DEFAULT(y_correct_putin_stage[pt]+4),THE_PUT_Z_DEFAULT(50+z_correct_putin_stage[pt]), RED_TI);
//            //P_TI
//            Status_PUT++;
//        } else if (Status_PUT == 69 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            arm_put_shake(178+r_correct_putin_wheel[pt], 4,THE_PUT_Z_DEFAULT(z_correct_putin_wheel[pt]), 5, RED_TI, 400);
//            Status_PUT++;
//        } else if (Status_PUT == 70 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_claw_controller(CLAW_RELEASE);
//            Status_PUT++;
//        } else if (Status_PUT == 71 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//            tick_check = armGroup->ProgramTick;
//            motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(x_correct_putin_stage[pt]), THE_PUT_Y_DEFAULT(y_correct_putin_stage[pt]+4), camera_see_point_z, RED_TI);
//            //P_TI
//            Status_PUT++;
//        } else if (Status_PUT == 72 && check_motions_finished()) {
//            motor_r_controller_angle(90); //P_TI
//            Status_PUT++;
//        }
//    } else if (Status_PUT == 61 && tick_check + 3000 < armGroup->ProgramTick) {
//        Status_PUT = 73;
//    }
//    if (Status_PUT == 73 && check_motions_finished()) {
//        motor_r_controller_angle(0); //P_TI
//        Status_PUT = Status_FINISH;
//    }
//    if (Status_PUT == Status_FINISH) {
//        Status_PUT = Status_BEGIN;
//        isPut = YZF_TRUE;
//        return YZF_OK;
//    }
//    return YZF_HANDLER_NOT_FINISH;
//}


















//
///***************物块放置内部物料台进程****************/
//int PutCount = 6;
//YZF_BOOL flag_error = YZF_TRUE;
//extern MainView *mainMenu;
//extern "C" YZF_RESULT robot_main_program_thread() //爪子的延时函数，setlength的调试
//{
//    if (Status_CT == Status_BEGIN) {
//         if (PutCount % 2 == 0 && flag_error) {
//             while (1) {
//                 if (yzf_gpio_get(GPIOF, YZF_PIN_12)) {
//                     yzf_gpio_set_1(GPIOE,YZF_PIN_13);
//                 }else {
//                     HAL_Delay(500);
//                     yzf_gpio_set_0(GPIOE,YZF_PIN_13);
//                     break;
//                 }
//             }
//         }
//        flag_error = YZF_TRUE;
//        motor_claw_controller(CLAW_RELEASE);
//        camera_finish = YZF_FALSE;
//        yzf_global_handler_add(robot_main_camera_thread);
//        armGroup->arm_clawR->setAngle(121 * 1.5 - 2.0f);
//        Status_CT = 100;
//    } //0.7是抓，-0.7是松
//    if (Status_CT == 100 && camera_finish == YZF_TRUE && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == ERROR_CLAW)
//        {
//            flag_error = YZF_FALSE;
//            Status_CT = Status_BEGIN;
//            yzf_gpio_set_1(GPIOE,YZF_PIN_13);
//            HAL_Delay(500);
//            yzf_gpio_set_0(GPIOE,YZF_PIN_13);
//            return YZF_HANDLER_NOT_FINISH;
//        }
//        else
//        {
//            Status_CT = 1;
//        }
////        motor_Stepper_XYZO_setLength(camera_check_x - 20, CAMERA_Y_OFFSET_SAN, camera_see_point_z, 0);
////        claw_type=claw_type_choose_by_height();
//    }
//    if (Status_CT == 1 && tick_check + 2000 < armGroup->ProgramTick) {
//        //等待摄像头处理完成
//        tick_check = armGroup->ProgramTick;
//        armGroup->arm_clawR->setAngle(armGroup->arm_clawR->getAngle() - camera_check_claw_R);
//        motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, camera_see_point_z, 0);
//        Status_CT++;
//    }
//    if (Status_CT == 2 && check_motions_finished()) {
//        //物料 开始夹取
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, Z_SENSOR_BEGIN * ARM_Z_2_MM, 0);
//        Status_CT=1000;
//    }
//    if(Status_CT==1000 && check_motions_finished()&&tick_check + 2000 < armGroup->ProgramTick){
//        tick_check=armGroup->ProgramTick;
//        if(mainMenu->dist_data>0&&mainMenu->dist_data<300){
//            if(claw_type==TI||claw_type==P_TI) {
//                if(mainMenu->dist_data<=Z_BLOCK_SHELL_LIMIT){
//                    claw_type= TI;
//                }else claw_type=P_TI;
//            }else if(claw_type==SAN||claw_type==P_SAN) {
//                if(mainMenu->dist_data<=Z_BLOCK_SHELL_LIMIT){
//                    claw_type= SAN;
//                }
//                else{
//                    claw_type=P_SAN;
//                }
//            }
//        }
//        if(claw_type==P_TI||claw_type==P_SAN){
//            motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, (53+z_correct_catch_conveyor[shell])*ARM_Z_2_MM, 0);
//        }else motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, (93+z_correct_catch_conveyor[block])*ARM_Z_2_MM, 0);
//        Status_CT=3;
//    }
//    if (Status_CT == 3 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_CATCH); //夹
//        if (camera_check_second)Status_CT = 50;
//        else Status_CT++;
//    }
//    if (Status_CT == 4 && check_motions_finished() && tick_check + 5000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick; //夹
//        motor_Stepper_XYZO_setLength(camera_check_x,CAMERA_Y_OFFSET_SAN, camera_see_point_z, 0); //x73
//        Status_CT++;
//    } else if (Status_CT == 5 && check_motions_finished()) {
//        //传送带抬升
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == TI)motor_Stepper_XYZO_setLength(78 * ARM_X_2_MM, 1 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == SAN) motor_Stepper_XYZO_setLength(95.5 * ARM_X_2_MM, 6 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == YUAN)motor_Stepper_XYZO_setLength(100 * ARM_X_2_MM, 65 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == SI) motor_Stepper_XYZO_setLength(74 * ARM_X_2_MM, 55 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == P_TI && stage_in[TI] == 0)motor_Stepper_XYZO_setLength(
//            90 * ARM_X_2_MM, 14 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == P_TI && stage_in[TI] == 1) {
//            motor_Stepper_XYZO_setLength(78 * ARM_X_2_MM, 1 * ARM_Y_2_MM, camera_see_point_z, 0);
//            isPut_PT = YZF_FALSE;
//        }
//        if (claw_type == P_SAN && stage_in[SAN] == 0) {
//            motor_Stepper_XYZO_setLength(80 * ARM_X_2_MM, 0 * ARM_Y_2_MM, camera_see_point_z, 0);
//
//        }
//        if (claw_type == P_SAN && stage_in[SAN] == 1){motor_Stepper_XYZO_setLength(
//            95.5 * ARM_X_2_MM, 6 * ARM_Y_2_MM, camera_see_point_z, 0);
//            isPut_PS = YZF_FALSE;}
//        armGroup->arm_clawR->setAngle(121 * 1.5 - 2.0f);
//        Status_CT++;
//    } else if (Status_CT == 6 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//        //转动
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == YUAN) motor_r_controller_angle(89);
//        if (claw_type == SI) motor_r_controller_angle(269);
//        if (claw_type == SAN) motor_r_controller_angle(239);
//        if (claw_type == TI) motor_r_controller_angle(296);
//        if (claw_type == P_TI && stage_in[TI] == 0)motor_r_controller_angle(116);
//        if (claw_type == P_TI && stage_in[TI] == 1)motor_r_controller_angle(296);
//        if (claw_type == P_SAN && stage_in[SAN] == 0)motor_r_controller_angle(62);
//        if (claw_type == P_SAN && stage_in[SAN] == 1)motor_r_controller_angle(239);
//        Status_CT++;
//    } else if (Status_CT == 7 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//        //内部放置
//        tick_check = armGroup->ProgramTick; //Z轴统一加10
//        if (claw_type == TI) motor_Stepper_XYZO_setLength(
//                (78+x_correct_putin_stage[ti])*ARM_X_2_MM, (1+y_correct_putin_stage[ti]) * ARM_Y_2_MM, (65+z_correct_catch_conveyor[ti])* ARM_Z_2_MM, 0);
//        if (claw_type == SAN) motor_Stepper_XYZO_setLength((95.5+x_correct_putin_stage[san]) * ARM_X_2_MM, (6+y_correct_putin_stage[san]) * ARM_Y_2_MM, (65+z_correct_putin_stage[san]) * ARM_Z_2_MM, 0);
//        if (claw_type == YUAN) motor_Stepper_XYZO_setLength((100+x_correct_putin_stage[yuan]) * ARM_X_2_MM, (65+y_correct_putin_stage[yuan]) * ARM_Y_2_MM, (68+z_correct_putin_stage[yuan]) * ARM_Z_2_MM, 0);
//        //三角形盖子y为0z为0,正方形y为53,Z为63，
//        if (claw_type == SI) motor_Stepper_XYZO_setLength((74+x_correct_putin_stage[si]) * ARM_X_2_MM, (55+y_correct_putin_stage[si]) * ARM_Y_2_MM, (68+z_correct_putin_stage[si]) * ARM_Z_2_MM, 0);
//        //三角形盖子y为0z为0,正方形y为53,Z为63
//        if (claw_type == P_TI && stage_in[TI] == 0)motor_Stepper_XYZO_setLength(
//                    (90+x_correct_putin_stage[pt]) * ARM_X_2_MM, (14+y_correct_putin_stage[pt]) * ARM_Y_2_MM, (58+z_correct_putin_stage[pt]) * ARM_Z_2_MM, 0);
//        if (claw_type == P_TI && stage_in[TI] == 1)motor_Stepper_XYZO_setLength(
//                    (78+x_correct_putin_stage[ti])*ARM_X_2_MM, (1+y_correct_putin_stage[ti]) * ARM_Y_2_MM, (150+z_correct_putin_stage[ti]) * ARM_Z_2_MM, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 0)motor_Stepper_XYZO_setLength(
//                    (80+x_correct_putin_stage[ps]) * ARM_X_2_MM, (0+y_correct_putin_stage[ps])* ARM_Y_2_MM, (55+z_correct_putin_stage[ps])* ARM_Z_2_MM, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 1)motor_Stepper_XYZO_setLength(
//                    (95.5+x_correct_putin_stage[san]) * ARM_X_2_MM, (6+y_correct_putin_stage[san]) * ARM_Y_2_MM, (145+z_correct_putin_stage[san]) * ARM_Z_2_MM, 0);
//        Status_CT++;
//    } else if (Status_CT == 8 && check_motions_finished() && tick_check + 4000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_RELEASE); //松
//        if (claw_type == P_TI || claw_type == P_SAN) Status_CT++;
//        else Status_CT = 12;
//    } else if (Status_CT == 9 && check_motions_finished() && tick_check + 4000 < armGroup->ProgramTick) {
//        //判定盖子
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == P_TI && stage_in[TI] == 0)motor_Stepper_XYZO_setLength(
//            92 * ARM_X_2_MM, 14 * ARM_Y_2_MM, 90 * ARM_Z_2_MM, 0);
//        if (claw_type == P_TI && stage_in[TI] == 1)motor_Stepper_XYZO_setLength(
//            78 * ARM_X_2_MM, 1 * ARM_Y_2_MM, 160 * ARM_Z_2_MM, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 0)motor_Stepper_XYZO_setLength(
//            80 * ARM_X_2_MM, 0 * ARM_Y_2_MM, 90 * ARM_Z_2_MM, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 1)motor_Stepper_XYZO_setLength(
//            95.5 * ARM_X_2_MM, 6 * ARM_Y_2_MM, 160 * ARM_Z_2_MM, 0);
//        Status_CT++;
//    } else if (Status_CT == 10 && check_motions_finished() && tick_check + 3000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick; //Z轴统一加10
//        if (claw_type == P_TI || claw_type == P_SAN)motor_claw_controller(CLAW_CATCH - 0.2f);
//        Status_CT++;
//    } else if (Status_CT == 11 && check_motions_finished() && tick_check + 4000 < armGroup->ProgramTick) {
//        //按盖子
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == P_TI && stage_in[TI] == 0)motor_Stepper_XYZO_setLength(
//            92 * ARM_X_2_MM, 14 * ARM_Y_2_MM, 70 * ARM_Z_2_MM, 0);
//        if (claw_type == P_TI && stage_in[TI] == 1)motor_Stepper_XYZO_setLength(
//            78 * ARM_X_2_MM, 1 * ARM_Y_2_MM, 145 * ARM_Z_2_MM, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 0)motor_Stepper_XYZO_setLength(
//            80 * ARM_X_2_MM, 0 * ARM_Y_2_MM, 85 * ARM_Z_2_MM, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 1)motor_Stepper_XYZO_setLength(
//            95.5 * ARM_X_2_MM, 6 * ARM_Y_2_MM, 145
//            * ARM_Z_2_MM, 0);
//        Status_CT++;
//    } else if (Status_CT == 12 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        //抬升
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == TI) motor_Stepper_XYZO_setLength(78 * ARM_X_2_MM, 1 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == SAN) motor_Stepper_XYZO_setLength(95.5 * ARM_X_2_MM, 6 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == YUAN) motor_Stepper_XYZO_setLength(100 * ARM_X_2_MM, 65 * ARM_Y_2_MM, camera_see_point_z, 0);
//        //三角形盖子y为0z为0,正方形y为53,Z为63，
//        if (claw_type == SI) motor_Stepper_XYZO_setLength(74 * ARM_X_2_MM, 55 * ARM_Y_2_MM, camera_see_point_z, 0);
//        //三角形盖子y为0z为0,正方形y为53,Z为63
//        if (claw_type == P_TI && stage_in[TI] == 0) motor_Stepper_XYZO_setLength(
//            90 * ARM_X_2_MM, 14 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == P_TI && stage_in[TI] == 1)motor_Stepper_XYZO_setLength(
//            78 * ARM_X_2_MM, 1 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 0)motor_Stepper_XYZO_setLength(
//            80 * ARM_X_2_MM, 0 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if (claw_type == P_SAN && stage_in[SAN] == 1)motor_Stepper_XYZO_setLength(
//            95.5 * ARM_X_2_MM, 6 * ARM_Y_2_MM, camera_see_point_z, 0);
//        if ((claw_type == P_TI && stage_in[TI] == 1) || (claw_type == P_SAN && stage_in[SAN] == 1))stage_in[claw_type]
//                --;
//        stage_in[claw_type]++;
//        Status_CT++;
//    } else if (Status_CT == 13 && check_motions_finished() && tick_check + 4000 < armGroup->ProgramTick) {
//        //回到传送带
//        tick_check = armGroup->ProgramTick;
//        motor_r_controller_angle(0);
//        Status_CT++;
//    } else if (Status_CT == 14 && check_motions_finished() && tick_check + 4000 < armGroup->ProgramTick) {
//        //上述操作重复六次
//        tick_check = armGroup->ProgramTick;
//        PutCount--;
//        Status_CT = Status_BEGIN;
//        if (PutCount == 0) {
//            Status_CT = 15;
//        }
//    } else if (Status_CT == 15 && check_motions_finished() && tick_check + 4000 < armGroup->ProgramTick) {
//        //进入内部抓到外部的函数，在上面，名字是robot_first_In_test
//        tick_check = armGroup->ProgramTick;
//        isPut = YZF_FALSE;
//        //yzf_global_handler_add(robot_main_test);
//        yzf_global_handler_add(robot_first_In_test);
//        Status_CT = Status_FINISH;
//    } else if (Status_CT == 16 && check_motions_finished() && isPut) {
//        PutCount--;
//        Status_CT = Status_BEGIN;
//        if (PutCount == 0) {
//            Status_CT = Status_FINISH;
//        }
//        return YZF_HANDLER_NOT_FINISH;
//    } else if (Status_CT == 50 && tick_check + 4000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_Stepper_XYZO_setLength(camera_check_x,CAMERA_Y_OFFSET_SAN, camera_see_point_z, 0);
//        Status_CT++;
//    } else if (Status_CT == 51 && check_motions_finished() && tick_check + 4000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        armGroup->arm_clawR->setAngle(270);
//        Status_CT++;
//    } else if (Status_CT == 52 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == TI) motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, 93 * ARM_Z_2_MM, 0);
//        if (claw_type == P_TI) motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, 70 * ARM_Z_2_MM, 0);
//        Status_CT++;
//    } else if (Status_CT == 53 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_RELEASE);
//        Status_CT++;
//    } else if (Status_CT == 54 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == P_TI) motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, 103 * ARM_Z_2_MM, 0);
//        Status_CT++;
//    } else if (Status_CT == 55 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        armGroup->arm_clawR->setAngle(90);
//        Status_CT++;
//    } else if (Status_CT == 56 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        if (claw_type == P_TI) motor_Stepper_XYZO_setLength(camera_check_x, CAMERA_Y_OFFSET_SAN, 70 * ARM_Z_2_MM, 0);
//        Status_CT++;
//    } else if (Status_CT == 57 && check_motions_finished() && tick_check + 2000 < armGroup->ProgramTick) {
//        tick_check = armGroup->ProgramTick;
//        motor_claw_controller(CLAW_CATCH);
//        Status_CT = 4;
//    }
//    if (Status_CT == Status_FINISH) {
//        Status_CT = Status_BEGIN;
//        return YZF_OK;
//    }
//    return YZF_HANDLER_NOT_FINISH;
//}