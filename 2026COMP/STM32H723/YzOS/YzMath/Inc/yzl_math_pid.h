//
// Created by 32827 on 2026/4/13.
//

#ifndef STM32H723_LTS_BSP_YZL_MATH_PID_H
#define STM32H723_LTS_BSP_YZL_MATH_PID_H

#include "yzl_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PID_LIMIT(x,max,min) x = x>max?max:(x<min?min:x);

typedef struct
{
    ///标准PID 部分
    //需要预先配置部分
    float kp;           //当前比例系数
    float ki;           //当前积分系数
    float kd;           //当前微分系数
    float max_out;      //输出最大值
    float min_out;      //输出最小值
    float max_sum;      //限制最大积分    单神经元无需设置
    float min_sum;      //限制最小积分    单神经元无需设置

    //标准数据存储位置
    float SetValue;     //设定值
    float ActualValue;  //实际值
    float e;            //误差
    float e_last;       //上次误差
    float e_last2;      //上上次误差
    float e_sum;        //积分           单神经元不记录
    float out;          //输出

    ///单神经元部分
    //需要预先设置部分
    float k_neuro;      //比例输出变化,影响整体响应速度
    float n_p;          //比例学习速度
    float n_i;          //积分学习速度
    float n_d;          //微分学习速度
    float k_max;        //权重最大值，防止发散
    float k_min;        //权重最小值，防止发散
} YzPID;

YZF_RESULT yzl_pid_default_init(YzPID *pid,float default_SetValue);

YZF_RESULT yzl_pid_neuro_init(YzPID *pid,float default_SetValue);

float yzl_pid_default_pos(YzPID *pid,float actual);

float yzl_pid_default_inc(YzPID *pid,float actual);

/**
 * @brief 单神经元自适应PID控制器 - 位置式实现
 * @param pid     PID结构体指针
 * @param actual  当前实际反馈值
 * @return float  控制输出值
 *
 * @note 权重w1/w2/w3直接对应ki/kp/kd，会在线学习更新
 *       初始值由用户设置的kp,ki,kd决定
 */
float yzl_pid_neuro_pos(YzPID *pid,float actual);

#ifdef __cplusplus
};
#endif

#endif //STM32H723_LTS_BSP_YZL_MATH_PID_H