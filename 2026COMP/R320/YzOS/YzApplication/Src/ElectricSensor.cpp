//
// Created by 32700 on 2026/5/23.
//
#include "../Inc/ElectricalSensor.hpp"
YZF_BOOL flag_sensor = YZF_TRUE;
int wheel_at=RED_YUAN;
int flag_in[9]={0,0,0,0,0,0,0,0,0};
void stage_in_update(int color_id){
    if(yzf_gpio_get(GPIOF, YZF_PIN_11) == YZF_TRUE){
        flag_in[color_id] = 1;
    }
}
int sensor_readData(int color_mode){
    tick_check=armGroup->ProgramTick;
    if(flag_sensor) {
        if (yzf_gpio_get(GPIOB, YZF_PIN_1) == YZF_FALSE) {
            if (tick_check + 10000 < armGroup->ProgramTick) {
                wheel_at = (color_mode + 7) % 8;
            }
            flag_sensor = YZF_FALSE;
        }
    }
    if(yzf_gpio_get(GPIOB, YZF_PIN_1) == YZF_TRUE){
        flag_sensor = YZF_TRUE;
    }
    stage_in_update(wheel_at);
    return (position_stage)wheel_at;
}



