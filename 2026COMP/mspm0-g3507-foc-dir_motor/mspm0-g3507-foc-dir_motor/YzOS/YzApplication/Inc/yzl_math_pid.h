//
// Created by 32827 on 2026/4/13.
//

#ifndef STM32H723_LTS_BSP_YZL_MATH_PID_H
#define STM32H723_LTS_BSP_YZL_MATH_PID_H

#include "yz.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PID_LIMIT(x,max,min) x = x>max?max:(x<min?min:x);

typedef struct
{
    ///标准PID部分
    float kp;           // 比例系数（会被神经元在线调整）
    float ki;           // 积分系数（会被神经元在线调整）
    float kd;           // 微分系数（会被神经元在线调整）
    float max_out;      // 输出最大值
    float min_out;      // 输出最小值
    float max_sum;      // 积分累加器最大值
    float min_sum;      // 积分累加器最小值

    // 数据存储
    float SetValue;     // 设定值
    float ActualValue;  // 实际值
    float e;            // 当前误差 e(k)
    float e_last;       // 上次误差 e(k-1)
    float e_last2;      // 上上次误差 e(k-2)
    float e_sum;        // 积分累加器 Σe
    float out;          // 输出 u(k)

    ///单神经元学习部分
    float k_neuro;      // 神经元整体增益（通常设为1或很小）
    float n_p;          // 比例系数学习率
    float n_i;          // 积分系数学习率
    float n_d;          // 微分系数学习率
    float k_max;        // 系数上限
    float k_min;        // 系数下限

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
 * @deprecated 误差累计造成foc旋转
 */
float yzl_pid_neuro_pos(YzPID *pid,float actual);

/**
 * @brief 单神经元自适应PID - 参数自整定版
 * @param pid     PID结构体指针
 * @param actual  当前实际反馈值
 * @return float  控制输出值
 *
 * @note 核心改进：
 *       1. 保留传统PID的e_sum积分累加器
 *       2. 神经元只负责在线调整kp, ki, kd参数
 *       3. 输出计算仍用标准PID公式
 */
float yzl_pid_neuro_adaptive_simplified(YzPID *pid,float actual);

#ifdef __cplusplus
};
#endif

#endif //STM32H723_LTS_BSP_YZL_MATH_PID_H