//
// Created by 18744 on 2026/5/16.
//

#include "CameraSeek.hpp"

extern MainView *mainMenu;

ClawType claw_type=YUAN;
CameraRequestLock *cm_lock;

int camera_handle_id=0;
float camera_current_x=0;
float camera_x_offset=0;
YZF_BOOL camera_enable_x_offset_edit = YZF_FALSE;
float camera_current_y=0;
YZF_BOOL camera_finish = YZF_FALSE;
int STATUS_CAMERA = Status_BEGIN;
int camera_check_count;


YZF_BOOL SUB_SEEKER = YZF_FALSE;
int STATUS_SUB_SEEKER = Status_BEGIN;
int sub_dest_x=0, sub_dest_y=0;
float sub_kpy = 0.02,sub_kpx = 0.05;

float camera_check_x=CAMERA_SEE_POINT_X;
float camera_check_y=0;
float camera_check_z=0;
float camera_check_r=0;
float camera_check_claw_R = 0;
YZF_BOOL camera_check_second = YZF_FALSE;
YZF_BOOL camera_filter_enable = YZF_FALSE;
ClawType seek_claw_type=NONE_CLAW;

int tag_fail_count = 7;
int camera_offset_for_x= 0;
extern int PutCount;
extern ThreadWatchDog inst_camera_watchdog;

YZF_BOOL camera_thread_is_in=YZF_FALSE;
YZF_BOOL camera_sub_thread_is_in=YZF_FALSE;

int camera_see_point_z = CAMERA_SEE_POINT_Z;
int camera_see_point_x = CAMERA_SEE_POINT_X;
int camera_see_point_y = CAMERA_SEE_POINT_Y;
extern "C" YZF_RESULT robot_main_camera_sub_seek();
extern "C" void camera_watchdog_listener()
{
    yz_info("[CAMERA][WATCHDOG] WATCHDOG RESET\n");
    // if (camera_thread_is_in==YZF_FALSE)
    // {
    //     yz_info("[CAMERA][WATCHDOG] NOT CAMERA THREAD\n");
    //     yzf_global_handler_add(robot_main_camera_thread);
    // }
    // if (camera_sub_thread_is_in == YZF_FALSE)
    // {
    //     yz_info("[CAMERA][WATCHDOG] NOT CAMERA SUB THREAD\n");
    //     yzf_global_handler_add(robot_main_camera_sub_seek);
    // }
    seek_claw_type = NONE_CLAW;
    STATUS_SUB_SEEKER = Status_BEGIN;
    camera_filter_enable = YZF_TRUE;
    STATUS_CAMERA = 2;
    SUB_SEEKER = YZF_FALSE;
}

int MapClawTypeToReqCode(ClawType type) {
    switch (type) {
        case YUAN:
            return 0x36;
        case SI:
            return 0x35;
        case M_TI:
            return 0x34;
        case P_TI:
            return 0x33;
        case M_SAN:
            return 0x32;
        case P_SAN:
            return 0x31;
        case SAN:
            return 0x32;
        case TI:
            return 0x34;
        case NONE_CLAW:
            return 0x46;
        default:
            return 0x46;
    }
}

ClawType MatchClawType(char *tag) //tag
{
    if (strncmp(tag,"yuan",4)==0) {
        return YUAN;
    }else if (strncmp(tag,"ti",2)==0) {
        return TI;
    }else if (strncmp(tag,"si",2)==0) {
        return SI;
    }else if (strncmp(tag,"san",3)==0) {
        return SAN;
    }else if (strncmp(tag,"ps",2)==0) {
        return P_SAN;
    }else if (strncmp(tag,"ms",2)==0) {
        return M_SAN;
    }else if (strncmp(tag,"pt",2)==0) {
        return P_TI;
    }else if (strncmp(tag,"mt",2)==0) {
        return M_TI;
    }else
    {
        return NONE_CLAW;
    }
}

ClawType MatchRealType2ClawType(RealType tag) //tag
{
    switch (tag)
    {
    case REAL_YUAN:
        return YUAN;
    case REAL_SI:
        return SI;
    case REAL_M_TI:
        return M_TI;
    case REAL_M_SAN:
        return M_SAN;
    case REAL_P_TI:
        return P_TI;
    case REAL_P_SAN:
        return P_SAN;
    default:
        return NONE_CLAW;
    }
}

ScreenState screen;
TrackedTarget *select_target;
int seek_unknow_count = 0;
int seek_error_count = 8;
extern "C" YZF_RESULT robot_main_camera_sub_seek() {
    if(STATUS_SUB_SEEKER==Status_BEGIN&&check_motions_finished()&&tick_check+200<armGroup->ProgramTick) {
        tick_check = armGroup->ProgramTick;
        camera_handle_id=0;
        camera_check_count = 2;

        SUB_SEEKER = YZF_FALSE;

        if (seek_claw_type==NONE_CLAW)
        {
            screen_init(&screen);
        }
        STATUS_SUB_SEEKER++;

        inst_camera_watchdog.lastTime = armGroup->ProgramTick;

        camera_sub_thread_is_in=YZF_FALSE;
    }

    //角度校准
    if (STATUS_SUB_SEEKER == 1 && check_motions_finished()) {
        mainMenu->cameraInstance->RequestCamera(MapClawTypeToReqCode(seek_claw_type));
        inst_camera_watchdog.lastTime = armGroup->ProgramTick;
        STATUS_SUB_SEEKER++;
    }
    if (STATUS_SUB_SEEKER == 2 && mainMenu->cameraInstance->LockArea->isValid) {
        if (mainMenu->cameraInstance->LockArea->isSUCCUSS) {
            cm_lock = mainMenu->cameraInstance->LockArea;
            if(cm_lock->count>=1){
                seek_error_count = 8;

                if (seek_claw_type==NONE_CLAW && camera_filter_enable)
                {
                    int active_count = screen_update(&screen, cm_lock);
                    seek_unknow_count++;

                    if (active_count > 0)
                    {
                        select_target = screen_get_center_target(&screen, CAMERA_PIXEL_X, CAMERA_PIXEL_Y);

                        if (select_target)
                        {
                            RealType best_type = bayes_get_best(&select_target->type_filter);
                            float prob = bayes_get_prob(&select_target->type_filter, best_type);
                            if (prob > 0.92f)
                            {
                                seek_claw_type = MatchRealType2ClawType(best_type);
                                camera_filter_enable = YZF_FALSE;
                                STATUS_SUB_SEEKER = 1;
                                return YZF_HANDLER_NOT_FINISH;
                            }
                        }
                    }

                    if (seek_unknow_count > 10) {
                        STATUS_SUB_SEEKER = Status_BEGIN;
                        seek_unknow_count = 0;
                        yz_info("[CAMERA] Seek Count More Than 10 times. Reset \n");
                    }

                    STATUS_SUB_SEEKER = 1;
                }else
                {
                    int max = 10000000;
                    for(int i=0;i<cm_lock->count;i++){
                        int distance=(int)sqrt(pow((cm_lock->Result[i].y+cm_lock->Result[i].h/2-CAMERA_PIXEL_Y/2),2)+pow(abs(cm_lock->Result[i].x+cm_lock->Result[i].w/2-CAMERA_PIXEL_X/2),2));
                        if(distance < max){
                            camera_handle_id = i;
                            max = distance;
                        }
                    }
                    STATUS_SUB_SEEKER++;
                }

            }else {
                if (seek_claw_type != NONE_CLAW) {
                    tag_fail_count--;
                    if (tag_fail_count<=0) {
                        tag_fail_count=4;
                        yz_info("[CAMERA] CLAW_TYPE MAY ERROR\n");
                        camera_filter_enable = YZF_TRUE;
                        seek_claw_type = NONE_CLAW;
                        STATUS_SUB_SEEKER = Status_BEGIN;
                        return YZF_HANDLER_NOT_FINISH;
                    }
                }
                if (seek_claw_type == NONE_CLAW)
                {
                    seek_error_count--;
                    if (seek_error_count<=0)
                    {
                        seek_claw_type = ERROR_CLAW;
                        return YZF_OK;
                    }
                }
                STATUS_SUB_SEEKER = 1;
            }
        }else {
            STATUS_SUB_SEEKER = 1;
        }
    }
    if (STATUS_SUB_SEEKER == 3) {
        int err_x = cm_lock->Result[camera_handle_id].x+cm_lock->Result[camera_handle_id].w/2 - camera_offset_for_x - CAMERA_PIXEL_X/2;
        if (abs(err_x)>CAMERA_PIXEL_X) { //检查数据异常
            STATUS_SUB_SEEKER = 1;
        }

        float xg;
        float xg_bk;
        xg_bk = camera_current_x;
        if (fabs(err_x) > sub_dest_x*2.5) {
            xg= camera_current_x - sub_kpx*(err_x)*0.6;
        }else {
            xg= camera_current_x - sub_kpx*(err_x);
        }
        if (fabs(xg) > 55) {
            STATUS_SUB_SEEKER = 1;
            xg = xg_bk;
            return YZF_HANDLER_NOT_FINISH;
        }

        if (abs(err_x)< sub_dest_x) {
            camera_check_r = xg;
            tick_check = armGroup->ProgramTick;
            camera_check_count--;
            STATUS_SUB_SEEKER++;
            if (camera_check_count<=0) {
                STATUS_SUB_SEEKER = 5;
            }
            return YZF_HANDLER_NOT_FINISH;
        }

        tick_check = armGroup->ProgramTick;
        motor_r_controller_angle(xg);
        camera_current_x = xg;
        STATUS_SUB_SEEKER++;
    }
    if (STATUS_SUB_SEEKER == 4&& tick_check+300<armGroup->ProgramTick) {
        STATUS_SUB_SEEKER=1;
    }
    if (STATUS_SUB_SEEKER == 5 && check_motions_finished()) {
        tag_fail_count=7;
        seek_unknow_count = 0;
        if (seek_claw_type==NONE_CLAW)
        {
            screen_init(&screen);
        }
        mainMenu->cameraInstance->RequestCamera(MapClawTypeToReqCode(seek_claw_type));
        inst_camera_watchdog.lastTime = armGroup->ProgramTick;
        STATUS_SUB_SEEKER++;
    }
    if (STATUS_SUB_SEEKER == 6 && mainMenu->cameraInstance->LockArea->isValid) {
        if (mainMenu->cameraInstance->LockArea->isSUCCUSS) {
            cm_lock = mainMenu->cameraInstance->LockArea;
            if(cm_lock->count>=1){

                seek_error_count = 8;

                if (seek_claw_type==NONE_CLAW && camera_filter_enable)
                {
                    int active_count = screen_update(&screen, cm_lock);
                    seek_unknow_count++;

                    if (active_count > 0)
                    {
                        select_target = screen_get_center_target(&screen, CAMERA_PIXEL_X, CAMERA_PIXEL_Y);

                        if (select_target)
                        {
                            RealType best_type = bayes_get_best(&select_target->type_filter);
                            float prob = bayes_get_prob(&select_target->type_filter, best_type);
                            if (prob > 0.92f)
                            {
                                seek_claw_type = MatchRealType2ClawType(best_type);
                                camera_filter_enable = YZF_FALSE;
                                STATUS_SUB_SEEKER = 5;
                                return YZF_HANDLER_NOT_FINISH;
                            }
                        }
                    }

                    if (seek_unknow_count > 10) {
                        STATUS_SUB_SEEKER = 5;
                        seek_unknow_count = 0;
                        yz_info("[CAMERA] Seek Count More Than 10 times. Reset \n");
                    }

                    STATUS_SUB_SEEKER = 5;
                }else
                {
                    int max = 10000000;
                    for(int i=0;i<cm_lock->count;i++){
                        int distance=(int)sqrt(pow((cm_lock->Result[i].y+cm_lock->Result[i].h/2-CAMERA_PIXEL_Y/2),2)+pow(abs(cm_lock->Result[i].x+cm_lock->Result[i].w/2-CAMERA_PIXEL_X/2),2));
                        if(distance < max){
                            camera_handle_id = i;
                            max = distance;
                        }
                    }
                    STATUS_SUB_SEEKER++;
                }
            }else {
                if (seek_claw_type != NONE_CLAW) {
                    tag_fail_count--;
                    if (tag_fail_count<=0) {
                        tag_fail_count=4;
                        yz_info("[CAMERA] CLAW_TYPE MAY ERROR\n");
                        camera_filter_enable = YZF_TRUE;
                        seek_claw_type = NONE_CLAW;
                        STATUS_SUB_SEEKER = 5;
                        return YZF_HANDLER_NOT_FINISH;
                    }
                }
                STATUS_SUB_SEEKER = 5;
            }
        }else {
            STATUS_SUB_SEEKER = 5;
        }
    }
    if (STATUS_SUB_SEEKER == 7 && check_motions_finished()) {
        int err_y = cm_lock->Result[camera_handle_id].y+cm_lock->Result[camera_handle_id].h/2 - CAMERA_PIXEL_Y/2;

        if (fabs(camera_x_offset)<40&&camera_enable_x_offset_edit)
        {
            camera_x_offset = camera_x_offset - sub_kpy*err_y;
        }else
        {
            camera_current_y = camera_current_y - sub_kpy*err_y;
        }

        if (abs(err_y)<sub_dest_y) {
            STATUS_SUB_SEEKER = Status_FINISH;
            camera_check_y = camera_see_point_y+camera_current_y*ARM_Y_2_MM;
            if (PutCount%2==0) {
                camera_check_x = camera_see_point_x-20*ARM_X_2_MM +camera_x_offset*ARM_X_2_MM;
            }else {
                camera_check_x = camera_see_point_x+30*ARM_X_2_MM +camera_x_offset*ARM_X_2_MM;
            }

            if (strncmp(cm_lock->Result[camera_handle_id].tag,"yuan",4)==0) {
                claw_type = YUAN;
            }else if (strncmp(cm_lock->Result[camera_handle_id].tag,"ti",2)==0) {
                claw_type = TI;
                camera_check_claw_R = cm_lock->Result[camera_handle_id].angle*360;
                if (camera_check_claw_R < 90) {
                    camera_check_claw_R = -camera_check_claw_R;
                    camera_check_second = YZF_FALSE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R > 270.0) {
                    camera_check_claw_R =360.0f-camera_check_claw_R;
                    camera_check_second = YZF_FALSE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R > 90.0 &&camera_check_claw_R < 180.0) {
                    camera_check_claw_R = (180.0f-camera_check_claw_R);
                    camera_check_second = YZF_TRUE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R >= 180.0 &&camera_check_claw_R <= 270.0) {
                    camera_check_claw_R =  camera_check_claw_R - 180.0f;
                    camera_check_claw_R = -camera_check_claw_R;
                    camera_check_second = YZF_TRUE;
                }
            }else if (strncmp(cm_lock->Result[camera_handle_id].tag,"si",2)==0) {
                claw_type = SI;
                camera_check_claw_R = cm_lock->Result[camera_handle_id].angle*360;
                while (camera_check_claw_R > 45.0f) {
                    camera_check_claw_R -= 45.0f;
                }
                camera_check_claw_R = 45.0f - camera_check_claw_R;
            }else if (strncmp(cm_lock->Result[camera_handle_id].tag,"san",3)==0) {
                claw_type = SAN;
                camera_check_claw_R = cm_lock->Result[camera_handle_id].angle*360;
                if (camera_check_claw_R >= 180.0f) {
                    camera_check_claw_R = - (camera_check_claw_R-180.0f);
                }
            }else if (strncmp(cm_lock->Result[camera_handle_id].tag,"ps",2)==0) {
                claw_type = P_SAN;
                camera_check_claw_R = cm_lock->Result[camera_handle_id].angle*360;
                if (camera_check_claw_R >= 180.0f) {
                    camera_check_claw_R = - (camera_check_claw_R-180.0f);
                }
            }else if (strncmp(cm_lock->Result[camera_handle_id].tag,"ms",2)==0) {
                claw_type = SAN;
                camera_check_claw_R = cm_lock->Result[camera_handle_id].angle*360;
                if (camera_check_claw_R >= 180.0f) {
                    camera_check_claw_R = - (camera_check_claw_R-180.0f);
                }
            }else if (strncmp(cm_lock->Result[camera_handle_id].tag,"pt",2)==0) {
                claw_type = P_TI;
                camera_check_claw_R = cm_lock->Result[camera_handle_id].angle*360;
                if (camera_check_claw_R < 90) {
                    camera_check_claw_R = -camera_check_claw_R;
                    camera_check_second = YZF_FALSE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R > 270.0) {
                    camera_check_claw_R =360.0f-camera_check_claw_R;
                    camera_check_second = YZF_FALSE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R > 90.0 &&camera_check_claw_R < 180.0) {
                    camera_check_claw_R = (180.0f-camera_check_claw_R);
                    camera_check_second = YZF_TRUE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R >= 180.0 &&camera_check_claw_R <= 270.0) {
                    camera_check_claw_R =  camera_check_claw_R - 180.0f;
                    camera_check_claw_R = -camera_check_claw_R;
                    camera_check_second = YZF_TRUE;
                }
            }else if (strncmp(cm_lock->Result[camera_handle_id].tag,"mt",2)==0) {
                claw_type = TI;
                camera_check_claw_R = cm_lock->Result[camera_handle_id].angle*360;
                if (camera_check_claw_R < 90) {
                    camera_check_claw_R = -camera_check_claw_R;
                    camera_check_second = YZF_FALSE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R > 270.0) {
                    camera_check_claw_R =360.0f-camera_check_claw_R;
                    camera_check_second = YZF_FALSE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R > 90.0 &&camera_check_claw_R < 180.0) {
                    camera_check_claw_R = (180.0f-camera_check_claw_R);
                    camera_check_second = YZF_TRUE;
                    return YZF_HANDLER_NOT_FINISH;
                }
                if (camera_check_claw_R >= 180.0 &&camera_check_claw_R <= 270.0) {
                    camera_check_claw_R =  camera_check_claw_R - 180.0f;
                    camera_check_claw_R = -camera_check_claw_R;
                    camera_check_second = YZF_TRUE;
                }
            }
            return YZF_HANDLER_NOT_FINISH;
        }

        tick_check = armGroup->ProgramTick;
        if (PutCount%2==0) {
            motor_Stepper_XYZO_setLength(camera_see_point_x+camera_x_offset*ARM_X_2_MM - 20 *ARM_X_2_MM, camera_see_point_y+camera_current_y*ARM_Y_2_MM,camera_see_point_z,0);
        }else {
            motor_Stepper_XYZO_setLength(camera_see_point_x+camera_x_offset*ARM_X_2_MM + 30 *ARM_X_2_MM, camera_see_point_y+camera_current_y*ARM_Y_2_MM,camera_see_point_z,0);
        }

        STATUS_SUB_SEEKER++;
    }
    if (STATUS_SUB_SEEKER == 8 && check_motions_finished()) {
        STATUS_SUB_SEEKER=5;
    }

    if(STATUS_SUB_SEEKER==Status_FINISH){
        camera_sub_thread_is_in=YZF_FALSE;
        STATUS_SUB_SEEKER = Status_BEGIN;
        SUB_SEEKER =YZF_TRUE;
        return YZF_OK;
    }
    return YZF_HANDLER_NOT_FINISH;
}

extern "C" YZF_RESULT robot_main_camera_thread() {
    if(STATUS_CAMERA==Status_BEGIN) {
        camera_thread_is_in = YZF_TRUE;
        tick_check = armGroup->ProgramTick;
        camera_check_r = 0;
        camera_check_y = 0;
        camera_current_x = 0;
        camera_current_y = 0;
        camera_handle_id = 0;
        camera_check_x = 0;
        camera_check_z = 0;
        camera_check_second = YZF_FALSE;
        camera_check_claw_R = 0;
        camera_check_count = 5;
        camera_finish = YZF_FALSE;
        seek_claw_type = NONE_CLAW;
        seek_error_count = 8;
        camera_see_point_z = CAMERA_SEE_POINT_Z;
        if (PutCount%2==0) {
            motor_Stepper_XYZO_setLength(camera_see_point_x-20*ARM_X_2_MM, camera_see_point_y,camera_see_point_z,0);
        }else {
            motor_Stepper_XYZO_setLength(camera_see_point_x+35*ARM_X_2_MM, camera_see_point_y,camera_see_point_z,0);
        }

        STATUS_CAMERA=100;
    }
    if (STATUS_CAMERA == 100 && check_motions_finished()) {
        motor_claw_controller(0.0);
        STATUS_CAMERA = 1;
    }
    if (STATUS_CAMERA == 1)
    {
        sub_dest_x = 35;
        sub_dest_y = 35;
        sub_kpy = 0.08;
        sub_kpx = 0.048;
        STATUS_SUB_SEEKER = Status_BEGIN;
        camera_enable_x_offset_edit = YZF_TRUE;
        camera_filter_enable = YZF_FALSE;
        seek_claw_type = NONE_CLAW;
        camera_x_offset = 0.0f;
        SUB_SEEKER =YZF_FALSE;
        seek_error_count = 8;

        inst_camera_watchdog.isEnable = YZF_TRUE;
        inst_camera_watchdog.lastTime = armGroup->ProgramTick;

        camera_see_point_z = CAMERA_SEE_POINT_Z;
        camera_see_point_x = CAMERA_SEE_POINT_X;
        camera_see_point_y = CAMERA_SEE_POINT_Y;
        yzf_global_handler_add(robot_main_camera_sub_seek);
        STATUS_CAMERA++;
    }
    if (STATUS_CAMERA == 2&& SUB_SEEKER)
    {
        if (seek_claw_type == ERROR_CLAW)
        {
            claw_type = ERROR_CLAW;
            return YZF_OK;
        }
        sub_dest_x = 24;
        sub_dest_y = 15;
        sub_kpy = 0.06;
        sub_kpx = 0.045;
        STATUS_SUB_SEEKER = Status_BEGIN;
        seek_claw_type = NONE_CLAW;
        camera_enable_x_offset_edit = YZF_TRUE;
        camera_filter_enable = YZF_TRUE;
        SUB_SEEKER =YZF_FALSE;
        seek_error_count = 8;

        inst_camera_watchdog.lastTime = armGroup->ProgramTick;

        camera_see_point_z = CAMERA_SEE_POINT_Z;
        yzf_global_handler_add(robot_main_camera_sub_seek);
        STATUS_CAMERA++;
    }
    if (STATUS_CAMERA == 3 && SUB_SEEKER)
    {
        if (seek_claw_type == ERROR_CLAW)
        {
            claw_type = ERROR_CLAW;
            return YZF_OK;
        }
        sub_dest_x = 15;
        sub_dest_y = 8;
        sub_kpy = 0.02;
        sub_kpx = 0.035;
        STATUS_SUB_SEEKER = Status_BEGIN;
        camera_enable_x_offset_edit = YZF_FALSE;
        camera_filter_enable = YZF_TRUE;
        SUB_SEEKER =YZF_FALSE;

        inst_camera_watchdog.lastTime = armGroup->ProgramTick;
        seek_error_count = 8;

        camera_see_point_z = 205*ARM_Z_2_MM;
        yzf_global_handler_add(robot_main_camera_sub_seek);
        STATUS_CAMERA++;
    }
    if (STATUS_CAMERA == 4 && SUB_SEEKER)
    {
        if (seek_claw_type == ERROR_CLAW)
        {
            claw_type = ERROR_CLAW;
            return YZF_OK;
        }

        inst_camera_watchdog.isEnable = YZF_FALSE;
        inst_camera_watchdog.lastTime = armGroup->ProgramTick;

        STATUS_CAMERA = Status_FINISH;
    }
    if(STATUS_CAMERA==Status_FINISH){
        inst_camera_watchdog.isEnable = YZF_FALSE;
        camera_thread_is_in = YZF_FALSE;
        STATUS_CAMERA = Status_BEGIN;
        yz_info("[CAMERA] CLAW_TYPE:%d ANGLE:%.3f\n",(int)claw_type,(float)camera_check_claw_R);
        yz_info("[CAMERA] 校准完成，等待下次开始。");
        camera_finish =YZF_TRUE;
        return YZF_OK;
    }
    return YZF_HANDLER_NOT_FINISH;
}

// ============ 基础似然矩阵（物块在中心时的值） ============
// 行：观测类型，列：真实类型
static const float base_likelihood[OBS_COUNT][REAL_TYPE_COUNT] = {
    // YUAN,   SI,  M_TI,  P_TI, M_SAN, P_SAN
    { 0.85, 0.03, 0.03, 0.03, 0.03, 0.03 },  // OBS_YUAN
    { 0.03, 0.85, 0.03, 0.03, 0.03, 0.03 },  // OBS_SI
    { 0.02, 0.02, 0.45, 0.40, 0.05, 0.06 },  // OBS_TI (不分材质)
    { 0.02, 0.02, 0.05, 0.06, 0.45, 0.40 },  // OBS_SAN (不分材质)
    { 0.02, 0.02, 0.55, 0.35, 0.04, 0.02 },  // OBS_MT (明确金属TI)
    { 0.02, 0.02, 0.35, 0.55, 0.02, 0.04 },  // OBS_PT (明确塑料TI)
    { 0.02, 0.02, 0.04, 0.02, 0.55, 0.35 },  // OBS_MS (明确金属SAN)
    { 0.02, 0.02, 0.02, 0.04, 0.35, 0.55 },  // OBS_PS (明确塑料SAN)
};

// ============ 参数 ============
#define CRITICAL_DISTANCE  80.0f   // 超过这个距离，材质完全不可靠
#define MAX_MATERIAL_ERROR 0.48f   // 最差情况材质混淆率（接近瞎猜的0.5）
#define MAX_SHAPE_ERROR    0.25f   // 最差情况形状错误率（比中心高10%）

// ============ 辅助函数：判断两个类型是否材质相同、形状相同 ============
static int same_material(RealType a, RealType b) {
    // 金属组
    if ((a == REAL_M_TI || a == REAL_M_SAN) &&
        (b == REAL_M_TI || b == REAL_M_SAN)) return 1;
    // 塑料组
    if ((a == REAL_P_TI || a == REAL_P_SAN) &&
        (b == REAL_P_TI || b == REAL_P_SAN)) return 1;
    // 圆和方没有材质概念，只和自己相同
    return (a == b);
}

static int same_shape(RealType a, RealType b) {
    // 圆
    if (a == REAL_YUAN && b == REAL_YUAN) return 1;
    // 方
    if (a == REAL_SI   && b == REAL_SI)   return 1;
    // 三角类（TI/SAN 都算三角）
    if ((a == REAL_M_TI || a == REAL_P_TI || a == REAL_M_SAN || a == REAL_P_SAN) &&
        (b == REAL_M_TI || b == REAL_P_TI || b == REAL_M_SAN || b == REAL_P_SAN)) return 1;
    return 0;
}

// ============ 核心：根据距离重建似然矩阵 ============
static void rebuild_likelihood(BayesFilter *f, ObsType obs, float distance) {
    float uniform = 1.0f / REAL_TYPE_COUNT;

    // 距离归一化：0=中心，1=临界距离或更远
    float t = fminf(distance / CRITICAL_DISTANCE, 1.0f);

    // 当前距离下的混淆率
    float mat_error  = 0.05f + t * (MAX_MATERIAL_ERROR - 0.05f);  // 5%→48%
    float shape_error = 0.02f + t * (MAX_SHAPE_ERROR - 0.02f);    // 2%→25%

    // 真实类型：遍历所有可能
    for (int real = 0; real < REAL_TYPE_COUNT; real++) {
        float sum_other_material = 0.0f;  // 材质不同但形状相同的概率总和
        float sum_other_shape    = 0.0f;  // 形状不同但材质相同的概率总和
        float sum_other_both     = 0.0f;  // 材质形状都不同的概率总和
        int   count_other_mat = 0, count_other_shape = 0, count_other_both = 0;

        // 统计各类"错误目标"的数量
        for (int r = 0; r < REAL_TYPE_COUNT; r++) {
            if (r == real) continue;
            int mat_same  = same_material((RealType)r, (RealType)real);
            int shap_same = same_shape((RealType)r, (RealType)real);

            if (!mat_same && shap_same)  count_other_mat++;   // 材质错、形状对
            else if (mat_same && !shap_same) count_other_shape++; // 形状错、材质对
            else if (!mat_same && !shap_same) count_other_both++;  // 都错
        }

        // 分配错误概率
        float prob_other_mat   = (count_other_mat   > 0) ? mat_error  / count_other_mat   : 0.0f;
        float prob_other_shape = (count_other_shape > 0) ? shape_error / count_other_shape : 0.0f;
        float prob_other_both  = (count_other_both  > 0) ? (mat_error * shape_error) / count_other_both : 0.0f;

        // 填充这一列
        for (int r = 0; r < REAL_TYPE_COUNT; r++) {
            if (r == real) {
                // 正确的概率 = 1 - 所有错误概率
                f->dynamic_likelihood[obs][real] = 1.0f -
                    (count_other_mat   * prob_other_mat) -
                    (count_other_shape * prob_other_shape) -
                    (count_other_both  * prob_other_both);
            } else {
                int mat_same  = same_material((RealType)r, (RealType)real);
                int shap_same = same_shape((RealType)r, (RealType)real);

                if (!mat_same && shap_same)
                    f->dynamic_likelihood[obs][r] = prob_other_mat;
                else if (mat_same && !shap_same)
                    f->dynamic_likelihood[obs][r] = prob_other_shape;
                else if (!mat_same && !shap_same)
                    f->dynamic_likelihood[obs][r] = prob_other_both;
                else
                    f->dynamic_likelihood[obs][r] = 0.0f;  // 不应该到这里
            }
        }
    }
}

// ============ 初始化 ============
void bayes_init(BayesFilter *f) {
    float uniform = 1.0f / REAL_TYPE_COUNT;
    for (int i = 0; i < REAL_TYPE_COUNT; i++) {
        f->belief[i] = uniform;
    }
    // 初始化似然为基值
    for (int o = 0; o < OBS_COUNT; o++) {
        for (int r = 0; r < REAL_TYPE_COUNT; r++) {
            f->dynamic_likelihood[o][r] = base_likelihood[o][r];
        }
    }
}

// ============ 更新接口 ============
void bayes_update(BayesFilter *f, ObsType obs, float confidence, float distance_to_center) {
    float new_belief[REAL_TYPE_COUNT];
    float sum = 0.0f;
    float uniform = 1.0f / REAL_TYPE_COUNT;

    // 距离衰减因子
    float t = fminf(distance_to_center / CRITICAL_DISTANCE, 1.0f);
    float reliability = 1.0f - t * 0.8f;  // 距离越远，可靠性越低

    if (confidence < 0.0f) confidence = 0.0f;
    if (confidence > 1.0f) confidence = 1.0f;

    // 直接使用 base_likelihood，用距离和置信度缩放
    float effective_conf = confidence * reliability;

    for (int real = 0; real < REAL_TYPE_COUNT; real++) {
        // 混合似然和均匀分布
        float scaled_like = effective_conf * base_likelihood[obs][real]
                          + (1.0f - effective_conf) * uniform;
        new_belief[real] = scaled_like * f->belief[real];
        sum += new_belief[real];
    }

    // 归一化
    if (sum > 1e-10f) {
        float inv_sum = 1.0f / sum;
        for (int i = 0; i < REAL_TYPE_COUNT; i++) {
            f->belief[i] = new_belief[i] * inv_sum;
        }
    } else {
        bayes_reset(f);
    }
}

// void bayes_update(BayesFilter *f, ObsType obs, float confidence, float distance_to_center) {
//     // 1. 根据距离重建当前观测对应的似然
//     rebuild_likelihood(f, obs, distance_to_center);
//
//     // 2. 贝叶斯更新（和之前一样，但用 dynamic_likelihood）
//     float new_belief[REAL_TYPE_COUNT];
//     float sum = 0.0f;
//     float uniform = 1.0f / REAL_TYPE_COUNT;
//
//     if (confidence < 0.0f) confidence = 0.0f;
//     if (confidence > 1.0f) confidence = 1.0f;
//
//     for (int real = 0; real < REAL_TYPE_COUNT; real++) {
//         // 置信度缩放
//         float scaled_like = confidence * f->dynamic_likelihood[obs][real]
//                           + (1.0f - confidence) * uniform;
//         new_belief[real] = scaled_like * f->belief[real];
//         sum += new_belief[real];
//     }
//
//     // 归一化
//     if (sum > 1e-10f) {
//         float inv_sum = 1.0f / sum;
//         for (int i = 0; i < REAL_TYPE_COUNT; i++) {
//             f->belief[i] = new_belief[i] * inv_sum;
//         }
//     } else {
//         bayes_reset(f);
//     }
// }

// ============ 查询 ============
RealType bayes_get_best(const BayesFilter *f) {
    int best = 0;
    for (int i = 1; i < REAL_TYPE_COUNT; i++) {
        if (f->belief[i] > f->belief[best]) {
            best = i;
        }
    }
    return (RealType)best;
}

float bayes_get_prob(const BayesFilter *f, RealType type) {
    return f->belief[type];
}

void bayes_reset(BayesFilter *f) {
    bayes_init(f);
}

BayesFilter bayes;
void bayes_camera_filter_sample(char *tag,float conf,float distance)
{
    bayes_init(&bayes);

    ObsType obs = OBS_YUAN;
    if (strncmp(tag, "yuan", 4)==0)      obs = OBS_YUAN;
    else if (strncmp(tag, "si", 2)==0)   obs = OBS_SI;
    else if (strncmp(tag, "ti", 2)==0)   obs = OBS_TI;
    else if (strncmp(tag, "san", 3)==0)  obs = OBS_SAN;
    else if (strncmp(tag, "mt", 2)==0)   obs = OBS_MT;
    else if (strncmp(tag, "pt", 2)==0)   obs = OBS_PT;
    else if (strncmp(tag, "ms", 2)==0)   obs = OBS_MS;
    else if (strncmp(tag, "ps", 2)==0)   obs = OBS_PS;

    bayes_update(&bayes, obs, conf ,distance);
}


ClawType bayes_camera_filter_check()
{
    RealType best = bayes_get_best(&bayes);
    float best_prob = bayes_get_prob(&bayes, best);

    if (best_prob > 0.7f) {
        switch (best) {
            case REAL_YUAN:  return YUAN;
            case REAL_SI:    return SI;
            case REAL_M_TI:  return M_TI;
            case REAL_P_TI:  return P_TI;
            case REAL_M_SAN: return M_SAN;
            case REAL_P_SAN: return P_SAN;
            default: return NONE_CLAW;
        }
    }else
    {
        return NONE_CLAW;
    }
}


//==========================================================


// ============ 辅助：两个框的IoU ============
static float calc_iou(float x1, float y1, float w1, float h1,
                       float x2, float y2, float w2, float h2) {
    // 计算交集
    float inter_x1 = fmaxf(x1, x2);
    float inter_y1 = fmaxf(y1, y2);
    float inter_x2 = fminf(x1 + w1, x2 + w2);
    float inter_y2 = fminf(y1 + h1, y2 + h2);

    float inter_w = fmaxf(0.0f, inter_x2 - inter_x1);
    float inter_h = fmaxf(0.0f, inter_y2 - inter_y1);
    float inter_area = inter_w * inter_h;

    if (inter_area <= 0.0f) return 0.0f;

    float area1 = w1 * h1;
    float area2 = w2 * h2;
    float union_area = area1 + area2 - inter_area;

    return inter_area / union_area;
}

// ============ 辅助：两个框中心距离 ============
static float calc_center_distance(float x1, float y1, float w1, float h1,
                                   float x2, float y2, float w2, float h2) {
    float cx1 = x1 + w1 / 2.0f;
    float cy1 = y1 + h1 / 2.0f;
    float cx2 = x2 + w2 / 2.0f;
    float cy2 = y2 + h2 / 2.0f;
    return sqrtf((cx1 - cx2) * (cx1 - cx2) + (cy1 - cy2) * (cy1 - cy2));
}

// ============ 辅助：合并两个框（取平均） ============
static void merge_boxes(float x1, float y1, float w1, float h1,
                         float x2, float y2, float w2, float h2,
                         float *mx, float *my, float *mw, float *mh) {
    *mx = (x1 + x2) / 2.0f;
    *my = (y1 + y2) / 2.0f;
    *mw = (w1 + w2) / 2.0f;
    *mh = (h1 + h2) / 2.0f;
}

// ============ 辅助：从tag映射到观测类型 ============
static ObsType tag_to_obs(const char *tag) {
    if (strncmp(tag, "yuan", 4) == 0) return OBS_YUAN;
    if (strncmp(tag, "si", 2) == 0)   return OBS_SI;
    if (strncmp(tag, "mt", 2) == 0)   return OBS_MT;
    if (strncmp(tag, "pt", 2) == 0)   return OBS_PT;
    if (strncmp(tag, "ms", 2) == 0)   return OBS_MS;
    if (strncmp(tag, "ps", 2) == 0)   return OBS_PS;
    if (strncmp(tag, "ti", 2) == 0)   return OBS_TI;
    if (strncmp(tag, "san", 3) == 0)  return OBS_SAN;
    return OBS_YUAN; // 默认
}

// ============ 初始化 ============
void screen_init(ScreenState *state) {
    state->target_count = 0;
    state->next_id = 0;
    for (int i = 0; i < MAX_TARGETS; i++) {
        state->targets[i].active = false;
        state->targets[i].id = -1;
        state->targets[i].observe_count = 0;
        state->targets[i].miss_count = 0;
        bayes_init(&state->targets[i].type_filter);
    }
}

// ============ 核心：合并同一帧的重叠框 ============
static int merge_frame_detections(CameraDetectResult *results, int count,
                                   CameraDetectResult *merged, int max_merged) {
    bool used[16] = {false};  // 假设count不会超过16
    int merged_count = 0;

    for (int i = 0; i < count && merged_count < max_merged; i++) {
        if (used[i]) continue;

        // 当前框作为种子
        float mx = (float)results[i].x;
        float my = (float)results[i].y;
        float mw = (float)results[i].w;
        float mh = (float)results[i].h;
        int merge_num = 1;

        // 找所有和它重叠的框
        for (int j = i + 1; j < count; j++) {
            if (used[j]) continue;

            float iou = calc_iou(mx, my, mw, mh,
                                 (float)results[j].x, (float)results[j].y,
                                 (float)results[j].w, (float)results[j].h);

            if (iou >= OVERLAP_THRESHOLD) {
                // 合并：加权平均（按面积加权更好，这里简化为直接平均）
                float new_mx, new_my, new_mw, new_mh;
                merge_boxes(mx, my, mw, mh,
                            (float)results[j].x, (float)results[j].y,
                            (float)results[j].w, (float)results[j].h,
                            &new_mx, &new_my, &new_mw, &new_mh);
                mx = new_mx; my = new_my; mw = new_mw; mh = new_mh;
                merge_num++;
                used[j] = true;
            }
        }

        // 保存合并结果
        merged[merged_count].x = (int)mx;
        merged[merged_count].y = (int)my;
        merged[merged_count].w = (int)mw;
        merged[merged_count].h = (int)mh;

        // tag取第一个框的（合并的框应该类型相同）
        strncpy(merged[merged_count].tag, results[i].tag, 19);
        merged[merged_count].tag[19] = '\0';

        merged[merged_count].confidence = results[i].confidence; // 简化
        merged[merged_count].angle = results[i].angle;

        used[i] = true;
        merged_count++;
    }

    return merged_count;
}


int screen_update(ScreenState *state, CameraRequestLock *lock) {
    if (!lock || !lock->isValid || !lock->isSUCCUSS || lock->count <= 0) {
        // 没有有效数据，所有目标miss+1
        for (int i = 0; i < MAX_TARGETS; i++) {
            if (state->targets[i].active) {
                state->targets[i].miss_count++;
                if (state->targets[i].miss_count >= MISS_TIMEOUT) {
                    state->targets[i].active = false;
                    state->target_count--;
                }
            }
        }
        return state->target_count;
    }

// 也许不需要
// static CameraDetectResult merged[MAX_TARGETS];
//     // 1. 合并帧内重叠框
//     int merged_count = merge_frame_detections(lock->Result, lock->count,
//                                                merged, MAX_TARGETS);

    // 2. 标记所有现有目标为"本帧未匹配"
    bool matched[MAX_TARGETS] = {false};

    // 3. 匹配：每个合并后的检测框，找最近的现有目标
    for (int m = 0; m < lock->count; m++) {

        float mx = (float)lock->Result[m].x + lock->Result[m].w/2.0f;
        float my = (float)lock->Result[m].y + lock->Result[m].h/2.0f;

        int best_idx = -1;
        float best_score = 1e10f;  // 距离越小越好

        for (int t = 0; t < MAX_TARGETS; t++) {
            if (!state->targets[t].active || matched[t]) continue;

            float dx = state->targets[t].center_x - mx;
            float dy = state->targets[t].center_y - my;
            float dist = sqrtf(dx*dx + dy*dy);

            // 距离必须在合理范围内（比如150像素）
            if (dist < 150.0f && dist < best_score) {
                best_score = dist;
                best_idx = t;
            }
        }

        mx = (float)lock->Result[m].x;
        my = (float)lock->Result[m].y;
        float mw = (float)lock->Result[m].w;
        float mh = (float)lock->Result[m].h;

        // 找最佳匹配的现有目标
        if (best_idx >= 0) {
            // 匹配成功：更新现有目标
            TrackedTarget *t = &state->targets[best_idx];

            // 平滑位置（EMA滤波，减少抖动）
            float alpha = 0.7f;  // 新观测权重
            t->x = alpha * mx + (1.0f - alpha) * t->x;
            t->y = alpha * my + (1.0f - alpha) * t->y;
            t->w = alpha * mw + (1.0f - alpha) * t->w;
            t->h = alpha * mh + (1.0f - alpha) * t->h;
            t->center_x = t->x + t->w / 2.0f;
            t->center_y = t->y + t->h / 2.0f;

            // 更新贝叶斯滤波器
            ObsType obs = tag_to_obs(lock->Result[m].tag);
            float dist = sqrtf(powf(t->center_x - ((float)CAMERA_PIXEL_X/2), 2) +
                              powf(t->center_y - ((float)CAMERA_PIXEL_Y/2), 2)); // 假设640x480
            bayes_update(&t->type_filter, obs, lock->Result[m].confidence*0.8, dist);

            t->last_confidence = lock->Result[m].confidence*0.8;
            t->observe_count++;
            t->miss_count = 0;
            matched[best_idx] = true;
            RealType rt = bayes_get_best(&t->type_filter);
            yz_info("[SCREEN] UPDATE: ID=%d, TAG=%d, CONF=%.2f, POS=(%.2f,%.2f)\n", t->id, (int)rt, bayes_get_prob(&t->type_filter,rt), t->x, t->y);

        } else {
            // 新目标：创建
            for (int t = 0; t < MAX_TARGETS; t++) {
                if (!state->targets[t].active) {
                    TrackedTarget *nt = &state->targets[t];
                    nt->id = state->next_id++;
                    nt->active = true;
                    nt->x = mx;
                    nt->y = my;
                    nt->w = mw;
                    nt->h = mh;
                    nt->center_x = mx + mw / 2.0f;
                    nt->center_y = my + mh / 2.0f;
                    nt->observe_count = 1;
                    nt->miss_count = 0;
                    nt->last_confidence = lock->Result[m].confidence;

                    bayes_init(&nt->type_filter);
                    ObsType obs = tag_to_obs(lock->Result[m].tag);
                    float dist = sqrtf(powf(nt->center_x - ((float)CAMERA_PIXEL_X/2), 2) +
                                      powf(nt->center_y - ((float)CAMERA_PIXEL_Y/2), 2));
                    bayes_update(&nt->type_filter, obs, lock->Result[m].confidence, dist);

                    state->target_count++;
                    matched[t] = true;
                    RealType rt = bayes_get_best(&nt->type_filter);
                    yz_info("[SCREEN] CREATE: ID=%d, TAG=%d, CONF=%.2f, POS=(%.2f,%.2f)\n", nt->id, (int)rt, bayes_get_prob(&nt->type_filter,rt), nt->x, nt->y);
                    break;
                }
            }
        }
    }

    // 4. 未匹配的现有目标：miss++
    for (int t = 0; t < MAX_TARGETS; t++) {
        if (state->targets[t].active && !matched[t]) {
            state->targets[t].miss_count++;
            if (state->targets[t].miss_count >= MISS_TIMEOUT) {
                state->targets[t].active = false;
                state->target_count--;

                TrackedTarget *dt = &state->targets[t];
                RealType rt = bayes_get_best(&dt->type_filter);
                yz_info("[SCREEN] DEACTIVE: ID=%d, TAG=%d, CONF=%.2f, POS=(%.2f,%.2f)\n", dt->id, (int)rt, bayes_get_prob(&dt->type_filter,rt), dt->x, dt->y);
            }else
            {
                TrackedTarget *dt = &state->targets[t];
                RealType rt = bayes_get_best(&dt->type_filter);
                yz_info("[SCREEN] DEACTIVE: ID=%d, TAG=%d, CONF=%.2f, POS=(%.2f,%.2f)\n", dt->id, (int)rt, bayes_get_prob(&dt->type_filter,rt), dt->x, dt->y);
            }
        }
    }

    return state->target_count;
}

// ============ 查询 ============
TrackedTarget* screen_get_center_target(ScreenState *state, int screen_w, int screen_h) {
    TrackedTarget *best = NULL;
    float best_dist = 1e10f;
    float cx = screen_w / 2.0f;
    float cy = screen_h / 2.0f;

    for (int i = 0; i < MAX_TARGETS; i++) {
        if (!state->targets[i].active) continue;
        float dx = state->targets[i].center_x - cx;
        float dy = state->targets[i].center_y - cy;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist < best_dist) {
            best_dist = dist;
            best = &state->targets[i];
        }
    }
    return best;
}

TrackedTarget* screen_get_by_id(ScreenState *state, int id) {
    for (int i = 0; i < MAX_TARGETS; i++) {
        if (state->targets[i].active && state->targets[i].id == id) {
            return &state->targets[i];
        }
    }
    return NULL;
}

TrackedTarget* screen_get_best_target(ScreenState *state) {
    TrackedTarget *best = NULL;
    float best_prob = 0.0f;

    for (int i = 0; i < MAX_TARGETS; i++) {
        if (!state->targets[i].active) continue;
        RealType type = bayes_get_best(&state->targets[i].type_filter);
        float prob = bayes_get_prob(&state->targets[i].type_filter, type);
        if (prob > best_prob) {
            best_prob = prob;
            best = &state->targets[i];
        }
    }
    return best;
}