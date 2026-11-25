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
YZL_GPIO conveyor_sensor={YZF_PIN_12,GPIOF};
YZL_GPIO higher_stage_sensor={YZF_PIN_8,GPIOE};
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
    yzf_gpio_set_0(conveyor.port, conveyor.pin);
}
limit_sensor arm_limit_higher(char H_L_C_F[]){
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
limit_sensor IR_sensor(char CO[]){
    if (strcmp(CO, "C") == 0) {
        int c = yzf_gpio_get(conveyor_sensor.port, conveyor_sensor.pin);
        if (c)return FEEDBACK_ON;
        else return FEEDBACK_OFF;
    } else if (strcmp(CO, "LS") == 0){
        int ls = yzf_gpio_get(lower_stage_sensor.port, lower_stage_sensor.pin);
        if (ls)return FEEDBACK_ON;
        else return FEEDBACK_OFF;
    }else if (strcmp(CO, "HS") == 0){
        int hs = yzf_gpio_get(higher_stage_sensor.port, higher_stage_sensor.pin);
        if (hs)return FEEDBACK_ON;
        else return FEEDBACK_OFF;
    }
    else return INPUT_ERROR;
}
uint32_t TIME=0;
extern uint32_t SystemTickCNT;
YZF_BOOL flag_TIME=YZF_FALSE;
YZF_BOOL conveyor_receive(YZF_GPIO_POINTER port,YZF_GPIO_PIN pin){
    return yzf_gpio_get(port, pin);
}
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
        if(dir[0]==0)armGroup->Line_oz_Tim8->nowPosition[0]=armGroup->Line_oz_Tim8->nowPosition[0]+length0;
        if(dir[0]==1)armGroup->Line_oz_Tim8->nowPosition[0]=armGroup->Line_oz_Tim8->nowPosition[0]+length0;
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
void motor_Stepper_setLength(SpaceSensorArmAsyncLine *Line_id_TimX, int length0, int length1){
    if(Line_id_TimX->isHanded()){
        length0=length0-Line_id_TimX->nowPosition[0];
        length1=length1-Line_id_TimX->nowPosition[1];
        motor_Stepper_getLength(Line_id_TimX,length0,length1);
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
        if(length0!=0)Line_id_TimX->Push(1, motor_blocks);
        if(length1!=0)Line_id_TimX->Push(2, motor_blocks + 1);
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
void motor_Stepper_XYZO_setLength(int lengthX, int lengthY,int lengthZ,int lengthO){//xyz,只能前进不能后退，
    motor_Stepper_setLength(armGroup->Line_xy_Tim1,lengthX,lengthY);
    motor_Stepper_setLength(armGroup->Line_oz_Tim8,lengthO,lengthZ);
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
void motor_r_controller_angle(float a) {
    motor_r_controller(P180*a/360.0f);
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