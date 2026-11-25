//
// Created by 32700 on 2026/5/5.
//
#include "../Inc/Motorclass.hpp"
/********灰度管脚定义********/
YZL_GPIO limit_z1={YZF_PIN_0,GPIOB};
YZL_GPIO limit_z2={YZF_PIN_1,GPIOB};
YZL_GPIO limit_z3={YZF_PIN_11,GPIOF};
YZL_GPIO limit_x1={YZF_PIN_5,GPIOC};
YZL_GPIO limit_x2={YZF_PIN_4,GPIOC};
YZL_GPIO limit_x3={YZF_PIN_7,GPIOA};
YZL_GPIO limit_x4={YZF_PIN_0,GPIOC};
YZL_GPIO conveyor_sensor={YZF_PIN_14,GPIOE};
YZL_GPIO higher_stage_sensor={YZF_PIN_11,GPIOE};
YZL_GPIO lower_stage_sensor={YZF_PIN_9,GPIOE};
YZL_GPIO conveyor={YZF_PIN_13,GPIOE};
/********灰度函数封装********/
void sensor_gpio_init()
{
    /* ========== Z轴上限位传感器 ========== */
    yzf_gpio_open_clock(limit_z1.port);
    yzf_gpio_open_clock(limit_z2.port);
    yzf_gpio_set_IN_PULL_UP(limit_z1.port, limit_z1.pin);
    yzf_gpio_set_IN_PULL_UP(limit_z2.port, limit_z2.pin);
    /* ========== Z轴下限位传感器 ========== */
    yzf_gpio_open_clock(limit_z3.port);
    yzf_gpio_set_IN_PULL_UP(limit_z3.port, limit_z3.pin);
    /* ========== X轴前限位传感器 (夹爪方向) ========== */
    yzf_gpio_open_clock(limit_x1.port);
    yzf_gpio_open_clock(limit_x2.port);
    yzf_gpio_set_IN_PULL_UP(limit_x1.port, limit_x1.pin);
    yzf_gpio_set_IN_PULL_UP(limit_x2.port, limit_x2.pin);
    /* ========== X轴后限位传感器 ========== */
    yzf_gpio_open_clock(limit_x3.port);
    yzf_gpio_open_clock(limit_x4.port);
    yzf_gpio_set_IN_PULL_UP(limit_x3.port, limit_x3.pin);
    yzf_gpio_set_IN_PULL_UP(limit_x4.port, limit_x4.pin);
    /* ========== 传送带红外传感器 ========== */
    yzf_gpio_open_clock(conveyor_sensor.port);
    yzf_gpio_set_IN_PULL_UP(conveyor_sensor.port, conveyor_sensor.pin);
    /* ========== 上下层到位传感器 ========== */
    yzf_gpio_open_clock(higher_stage_sensor.port);
    yzf_gpio_set_IN_PULL_UP(higher_stage_sensor.port, higher_stage_sensor.pin);
    yzf_gpio_open_clock(lower_stage_sensor.port);
    yzf_gpio_set_IN_PULL_UP(lower_stage_sensor.port, lower_stage_sensor.pin);
    /* ========== 传送带电机控制引脚 ========== */
    yzf_gpio_open_clock(conveyor.port);
    yzf_gpio_set_PP(conveyor.port, conveyor.pin);
    yzf_gpio_set_Pull_Down(conveyor.port, conveyor.pin);
    yzf_gpio_set_0(conveyor.port, conveyor.pin);
}
limit_sensor arm_sensor(const char H_L_C_F[]){
    /* ========== Z轴上限位传感器 ========== */
    yzf_gpio_open_clock(limit_z1.port);
    yzf_gpio_open_clock(limit_z2.port);
    yzf_gpio_set_IN_PULL_UP(limit_z1.port, limit_z1.pin);
    yzf_gpio_set_IN_PULL_UP(limit_z2.port, limit_z2.pin);
    /* ========== Z轴下限位传感器 ========== */
    yzf_gpio_open_clock(limit_z3.port);
    yzf_gpio_set_IN_PULL_UP(limit_z3.port, limit_z3.pin);
    /* ========== X轴前限位传感器 (夹爪方向) ========== */
    yzf_gpio_open_clock(limit_x1.port);
    yzf_gpio_open_clock(limit_x2.port);
    yzf_gpio_set_IN_PULL_UP(limit_x1.port, limit_x1.pin);
    yzf_gpio_set_IN_PULL_UP(limit_x2.port, limit_x2.pin);
    /* ========== X轴后限位传感器 ========== */
    yzf_gpio_open_clock(limit_x3.port);
    yzf_gpio_open_clock(limit_x4.port);
    yzf_gpio_set_IN_PULL_UP(limit_x3.port, limit_x3.pin);
    yzf_gpio_set_IN_PULL_UP(limit_x4.port, limit_x4.pin);
    if(strcmp(H_L_C_F,"H")==0){
        int z1=yzf_gpio_get(limit_z1.port,limit_z1.pin);
        int z2=yzf_gpio_get(limit_z2.port,limit_z2.pin);
        if(z1&&z2)return POSITION_CURRENT;
        else if(z1&&(!z2))return TOO_HIGH;
        else if ((!z1)&&z2)return TOO_LOW;
        else return TOO_LOW;
    }else if(strcmp(H_L_C_F,"L")==0){
        int z3=yzf_gpio_get(limit_z3.port,limit_z3.pin);
        if(z3)return TOO_HIGH;
        else return POSITION_CURRENT;
    }else if(strcmp(H_L_C_F,"C")==0){
        int x1=yzf_gpio_get(limit_x1.port,limit_x1.pin);
        int x2=yzf_gpio_get(limit_x2.port,limit_x2.pin);
        if(x1&&x2)return POSITION_CURRENT;
        else if(x1&&(!x2))return TOO_FAR;
        else if ((!x1)&&x2)return TOO_CLOSE;
        else return TOO_FAR;
    }else if(strcmp(H_L_C_F,"F")==0){
        int x3=yzf_gpio_get(limit_x3.port,limit_x3.pin);
        int x4=yzf_gpio_get(limit_x4.port,limit_x4.pin);
        if(x3&&x4)return POSITION_CURRENT;
        else if(x3&&(!x4))return TOO_CLOSE;
        else if ((!x3)&&x4)return TOO_FAR;
        else return TOO_CLOSE;
    }else return INPUT_ERROR;
}
limit_sensor IR_sensor(const char CO[]){

    /* ========== 传送带红外传感器 ========== */
    yzf_gpio_open_clock(conveyor_sensor.port);
    yzf_gpio_set_IN_PULL_UP(conveyor_sensor.port, conveyor_sensor.pin);
    /* ========== 上下层到位传感器 ========== */
    yzf_gpio_open_clock(higher_stage_sensor.port);
    yzf_gpio_set_IN_PULL_UP(higher_stage_sensor.port, higher_stage_sensor.pin);
    yzf_gpio_open_clock(lower_stage_sensor.port);
    yzf_gpio_set_IN_PULL_UP(lower_stage_sensor.port, lower_stage_sensor.pin);
    if (strcmp(CO, "C") == 0) {
        int c = yzf_gpio_get(conveyor_sensor.port, conveyor_sensor.pin);
        if (c)return FEEDBACK_ON;
        else return FEEDBACK_OFF;
    } else if (strcmp(CO, "L") == 0){
        int ls = yzf_gpio_get(lower_stage_sensor.port, lower_stage_sensor.pin);
        if (ls)return FEEDBACK_ON;
        else return FEEDBACK_OFF;
    }else if (strcmp(CO, "H") == 0){
        int hs = yzf_gpio_get(higher_stage_sensor.port, higher_stage_sensor.pin);
        if (hs)return FEEDBACK_ON;
        else return FEEDBACK_OFF;
    }
    else return INPUT_ERROR;
}
uint32_t TIME=0;
extern uint32_t SystemTickCNT;
YZF_BOOL flag_TIME=YZF_FALSE;
YZF_BOOL conveyor_addLength(uint32_t Length){
    yzf_gpio_open_clock(conveyor.port);
    yzf_gpio_set_PP(conveyor.port, conveyor.pin);
    yzf_gpio_set_Pull_Down(conveyor.port, conveyor.pin);
    yzf_gpio_set_1(conveyor.port, conveyor.pin);
    if(!flag_TIME){
        TIME=SystemTickCNT;
        flag_TIME=YZF_TRUE;
        return YZF_TRUE;
    }
    if(SystemTickCNT-TIME>=Length){
        yzf_gpio_set_0(conveyor.port, conveyor.pin);
        flag_TIME=YZF_FALSE;
        return YZF_FALSE;
    }
    return YZF_TRUE;
}
int dir[2] = {0};
void motor_Stepper_set_dir(int x, int y){
    if(x<=0)dir[0] = 0;
    else dir[0] = 1;
    if(y<=0)dir[1] = 0;
    else dir[1] = 1;
}
void motor_Stepper_getLength(SpaceSensorArmAsyncLine *Line_id_TimX,int length0,int length1){
    motor_Stepper_set_dir(length0,length1);
    if(Line_id_TimX == armGroup->Line_xy_Tim1){
        if(dir[0]==0)armGroup->Line_xy_Tim1->nowPosition[0]=armGroup->Line_xy_Tim1->nowPosition[0]+length0;
        if(dir[0]==1)armGroup->Line_xy_Tim1->nowPosition[0]=armGroup->Line_xy_Tim1->nowPosition[0]+length0;
        if(dir[1]==0)armGroup->Line_xy_Tim1->nowPosition[1]=armGroup->Line_xy_Tim1->nowPosition[1]+length1;
        if(dir[1]==1)armGroup->Line_xy_Tim1->nowPosition[1]=armGroup->Line_xy_Tim1->nowPosition[1]+length1;
    }
    if(Line_id_TimX == armGroup->Line_oz_Tim8){
        if(dir[0]==0){
            dir[0]=1;
            armGroup->Line_oz_Tim8->nowPosition[0]=armGroup->Line_oz_Tim8->nowPosition[0]+length0;

        }
        else{
            dir[0]=0;
            armGroup->Line_oz_Tim8->nowPosition[0]=armGroup->Line_oz_Tim8->nowPosition[0]+length0;
        }

        if(dir[1]==0){
            dir[1]=1;
            armGroup->Line_oz_Tim8->nowPosition[1]=armGroup->Line_oz_Tim8->nowPosition[1]+length1;
            return;
        }
        if(dir[1]==1){
            dir[1]=0;
            armGroup->Line_oz_Tim8->nowPosition[1]=armGroup->Line_oz_Tim8->nowPosition[1]+length1;
        }
    }
}
ArmAsyncBlock motor_blocks[4];
void motor_Stepper_OZ_setLength(int length0, int length1){
    if(armGroup->Line_oz_Tim8->isHanded()){
        length0=length0-armGroup->Line_oz_Tim8->nowPosition[0];
        length1=length1-armGroup->Line_oz_Tim8->nowPosition[1];
        if (length0>4)length0-=8;
        if (length0<-4)length0+=8;
        motor_Stepper_getLength(armGroup->Line_oz_Tim8,length0,length1);
        if (armGroup->Line_oz_Tim8->nowPosition[0]>=8)armGroup->Line_oz_Tim8->nowPosition[0]-=8;
        if (armGroup->Line_oz_Tim8->nowPosition[0]<0)armGroup->Line_oz_Tim8->nowPosition[0]+=8;
        length0=abs(length0)*STAGE_MOVE_2_GRID;
        length1=abs(length1);
        motor_blocks[0].Dir = dir[0];
        motor_blocks[0].RawStep = length0;
        motor_blocks[0].isInit = YZF_TRUE;
        motor_blocks[0].isFinish = YZF_FALSE;
        motor_blocks[1].Dir = dir[1];
        motor_blocks[1].RawStep = length1;
        motor_blocks[1].isFinish = YZF_FALSE;
        motor_blocks[1].isInit = YZF_TRUE;
        if(length0!=0)armGroup->Line_oz_Tim8->Push(1, motor_blocks);
        if(length1!=0)armGroup->Line_oz_Tim8->Push(2, motor_blocks + 1);
    }
}
void motor_Stepper_XY_setLength(int length0, int length1){
    if(armGroup->Line_xy_Tim1->isHanded()){
        length0=length0-armGroup->Line_xy_Tim1->nowPosition[0];
        length1=length1-armGroup->Line_xy_Tim1->nowPosition[1];
        motor_Stepper_getLength(armGroup->Line_xy_Tim1,length0,length1);
        length0=abs(length0);
        length1=abs(length1);
        motor_blocks[0].Dir = dir[0];
        motor_blocks[0].RawStep = length0;
        motor_blocks[0].isInit = YZF_TRUE;
        motor_blocks[0].isFinish = YZF_FALSE;
        motor_blocks[1].Dir = dir[1];
        motor_blocks[1].RawStep = length1;
        motor_blocks[1].isFinish = YZF_FALSE;
        motor_blocks[1].isInit = YZF_TRUE;
        if(length0!=0)armGroup->Line_xy_Tim1->Push(1, motor_blocks);
        if(length1!=0)armGroup->Line_xy_Tim1->Push(2, motor_blocks + 1);
    }
}
void motor_Stepper_controller(SpaceSensorArmAsyncLine *Line_id_TimX, int length0, int length1){
    if(Line_id_TimX->isHanded()){
        motor_Stepper_getLength(Line_id_TimX,length0,length1);
        length0=abs(length0);
        length1=abs(length1);
        motor_blocks[0].Dir = dir[0];
        motor_blocks[0].RawStep = length0;
        motor_blocks[0].isInit = YZF_TRUE;
        motor_blocks[0].isFinish = YZF_FALSE;
        motor_blocks[1].Dir = dir[1];
        motor_blocks[1].RawStep = length1;
        motor_blocks[1].isInit = YZF_TRUE;
        motor_blocks[1].isFinish = YZF_FALSE;
        if(length0!=0)Line_id_TimX->Push(1, motor_blocks);
        if(length1!=0)Line_id_TimX->Push(2, motor_blocks + 1);
    }
}
void motor_Stepper_XYZO_setLength(int lengthX, int lengthY,int lengthZ,int id_position){//3200，6400，2000，xyz,before
    lengthX=(int)(lengthX/2);lengthY=(int)(lengthY);lengthZ=(int)(lengthZ/5*2);//1600,1600,800,now
    motor_Stepper_XY_setLength(lengthX,lengthY);
    motor_Stepper_OZ_setLength(id_position,lengthZ);
}
YZF_BOOL try_start_motion(SpaceSensorArmAsyncLine *line, int len0, int len1, int *started_flag) {
    if (!(*started_flag)){
        motor_Stepper_controller(line,len0,len1);
        *started_flag = 1;
        return YZF_TRUE;
    }
    return (*started_flag) ? YZF_TRUE : YZF_FALSE;
}

uint8_t r[6]={0xee,0xff,0x00,0x00,0x55,0xaa};
#define P180 (1<<15)
void motor_r_controller_angle(double a) {
    yz_info("[ANGLE_R] %.3lf\n",a);
    motor_r_controller(P180*(a/360.0));
}
void motor_r_controller(int32_t length){
    while (length < 0 || length > P180) {
        if (length < 0) {
            length += P180;
        }else {
            length -= P180;
        }
    }
    r[2]=(length & 0x7f00) >> 8;
    r[3]=length & 0x00ff;
    yz_shell_send(UART4, r ,6);
}
uint8_t claw[8]={0xee,0xff,0x00,0x00,0x00,0x00,0x55,0xaa};
void motor_claw_controller(float length){
    auto *float_bytes = reinterpret_cast<uint8_t *>(&length);
    claw[2] = float_bytes[0];
    claw[3] = float_bytes[1];
    claw[4] = float_bytes[2];
    claw[5] = float_bytes[3];
    yz_shell_send(UART5, claw ,8);
}
YZF_BOOL check_motions_finished(){
    if(armGroup->Line_xy_Tim1->isHanded() && armGroup->Line_oz_Tim8->isHanded() && armGroup->arm_clawR->isActionFinish())return YZF_TRUE;
    else return YZF_FALSE;
}
uint16_t dist_data=0;
ClawType claw_type_choose_by_height(){
    if (claw_type==P_SAN||claw_type==P_TI||claw_type==SAN||claw_type==TI) {
        if (dist_data<Z_BLOCK_SHELL_LIMIT) {
            claw_type=(claw_type==P_SAN||claw_type==SAN)?SAN:claw_type;
            claw_type=(claw_type==P_TI||claw_type==TI)?TI:claw_type;
        }else{
            claw_type=(claw_type==P_SAN||claw_type==SAN)?P_SAN:claw_type;
            claw_type=(claw_type==P_TI||claw_type==TI)?P_SAN:claw_type;
        }
        return claw_type;
    }
}

void arm_put_shake(float correct_angle,float shake_angle,int end_z,int shake7_z,int id_circle,int delay_time_ms) {
    motor_r_controller_angle(correct_angle-shake_angle);
    int begin_z=shake7_z*7+end_z;
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z,id_circle);//YUAN
    HAL_Delay(delay_time_ms);
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z-shake7_z,id_circle);//YUAN
    motor_r_controller_angle(correct_angle+shake_angle);
    HAL_Delay(delay_time_ms);
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z-2*shake7_z,id_circle);//YUAN
    motor_r_controller_angle(correct_angle-shake_angle);
    HAL_Delay(delay_time_ms);
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z-shake7_z*3,id_circle);
    motor_r_controller_angle(correct_angle+shake_angle);
    HAL_Delay(delay_time_ms);
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z-shake7_z*4,id_circle);
    motor_r_controller_angle(correct_angle-shake_angle);
    HAL_Delay(delay_time_ms);
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z-shake7_z*5,id_circle);
    motor_r_controller_angle(correct_angle+shake_angle);
    HAL_Delay(delay_time_ms);
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z-shake7_z*6,id_circle);
    motor_r_controller_angle(correct_angle-shake_angle);
    HAL_Delay(delay_time_ms);
    motor_r_controller_angle(correct_angle);
    motor_Stepper_XYZO_setLength(THE_PUT_X_DEFAULT(0), THE_PUT_Y_DEFAULT(7),begin_z-shake7_z*7,id_circle);
    HAL_Delay(delay_time_ms);
}
void arm_put_shake_ex(float correct_angle,float shake_angle,int delay_time_ms){
    motor_claw_controller(0.0f);
    motor_r_controller_angle(correct_angle-shake_angle);
    HAL_Delay(delay_time_ms);
    motor_r_controller_angle(correct_angle+shake_angle);
    HAL_Delay(delay_time_ms);

    motor_r_controller_angle(correct_angle-shake_angle);
    HAL_Delay(delay_time_ms);
    motor_r_controller_angle(correct_angle+shake_angle);
    HAL_Delay(delay_time_ms);

    motor_r_controller_angle(correct_angle);
}
int shake_ex_position=0;
YZF_BOOL arm_put_shake_ex2(float correct_angle,float shake_angle,int delay_time_ms,int def_x,int def_y,int def_z,int id_circle,int offset) {
    if(shake_ex_position == 0){
        motor_Stepper_XYZO_setLength(def_x, def_y,def_z,id_circle);
        shake_ex_position++;
        return YZF_TRUE;
    }
    if(shake_ex_position == 1&& check_motions_finished()){
        motor_claw_controller(0.0);
        motor_r_controller_angle(correct_angle-shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle+shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle);
        HAL_Delay(delay_time_ms);
        motor_Stepper_XYZO_setLength(def_x, def_y+offset,def_z,id_circle);
        shake_ex_position++;
        return YZF_TRUE;
    }
    if(shake_ex_position == 2&& check_motions_finished()){
        motor_claw_controller(0.0);
        motor_r_controller_angle(correct_angle-shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle+shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle);
        HAL_Delay(delay_time_ms);
        motor_Stepper_XYZO_setLength(def_x, def_y+offset*2,def_z,id_circle);
        shake_ex_position++;
        return YZF_TRUE;
    }
    if(shake_ex_position == 3&& check_motions_finished()){
        motor_r_controller_angle(correct_angle-shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle+shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle);
        HAL_Delay(delay_time_ms);
        motor_Stepper_XYZO_setLength(def_x, def_y-offset,def_z,id_circle);
        shake_ex_position++;
        return YZF_TRUE;
    }
    if(shake_ex_position == 4&& check_motions_finished()){
        motor_r_controller_angle(correct_angle-shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle+shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle);
        HAL_Delay(delay_time_ms);
        motor_Stepper_XYZO_setLength(def_x, def_y-offset*2,def_z,id_circle);
        shake_ex_position++;
        return YZF_TRUE;
    }
    if(shake_ex_position == 5&& check_motions_finished()){
        motor_r_controller_angle(correct_angle-shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle+shake_angle);
        HAL_Delay(delay_time_ms);
        motor_r_controller_angle(correct_angle);
        HAL_Delay(delay_time_ms);
        motor_Stepper_XYZO_setLength(def_x, def_y,def_z,id_circle);
        shake_ex_position++;
        return YZF_TRUE;
    }else if(shake_ex_position == 6 && check_motions_finished()){
        shake_ex_position=0;
        return YZF_FALSE;
    }
    return YZF_TRUE;
}