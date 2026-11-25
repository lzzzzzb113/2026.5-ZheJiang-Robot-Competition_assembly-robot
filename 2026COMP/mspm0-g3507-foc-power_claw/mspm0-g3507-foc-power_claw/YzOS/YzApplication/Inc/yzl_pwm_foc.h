//
// Created by 32827 on 2025/11/9.
//

#ifndef STM32C8T6_FOC_YZL_PWM_FOC_H
#define STM32C8T6_FOC_YZL_PWM_FOC_H
#include "yzf.h"
#include "IQmathLib.h"
#include "yzl_math_pid.h"

typedef GPTIMER_Regs *YZF_TIM_POINTER;

typedef struct {
    YZF_TIM_POINTER pwm_control1;
    int ch1;
    YZF_TIM_POINTER pwm_control2;
    int ch2;
    YZF_TIM_POINTER pwm_control3;
    int ch3;

    int MaxDuty;
    int MinDuty;
    int Freq;
    int AngleSampleRate;
    int PolarNum;
    int SpeedUpdateTime;

    YzPID position_pid;
    float speed_filter[7];
    YzPID speed_pid;
    YzPID iq_pid;
    YzPID id_pid;

    // data save
    int zero_raw_rad;       // 0度对应的编码器值
    int cur_raw_rad;        // 当前编码器值
    int last_real_raw_rad;  // 上一次相对编码器值
    int current_real_raw_rad;// 当前相对编码器值
    int position_offset_rad;
    int position_err_rad;

    int current_speed_radi;
    float current_speed_rad;
    float current_sector_rad;
    _iq24 current_sector_rad_q24;

    float iu,iv;
    float iAlpha,iBeta;
    float iq,id;

    _iq24 iu_q24,iv_q24;
    _iq24 iAlpha_q24,iBeta_q24;
    _iq24 iq_q24,id_q24;

    void *update_angle_p;
    float (*update_angle)(void *p);

} YZ_PWM_FOC;


YZF_RESULT yzl_foc_init(YZ_PWM_FOC *foc);

float yzl_foc_sliding_window_mean_filter(float *input, float newdata, int length);

YZF_RESULT yzl_foc_update(YZ_PWM_FOC *foc);
#endif //STM32C8T6_FOC_YZL_PWM_FOC_H
