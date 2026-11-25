//
// Created by 32827 on 2026/4/13.
//

#include "yzl_math_pid.h"

YZF_RESULT yzl_pid_default_init(YzPID *pid,float default_SetValue)
{
    pid->ActualValue = 0;
    pid->e = 0;
    pid->e_last = 0;
    pid->e_last2 = 0;
    pid->e_sum = 0;
    pid->out = 0;

    pid->SetValue = default_SetValue;

    return YZF_OK;
}

YZF_RESULT yzl_pid_neuro_init(YzPID *pid,float default_SetValue)
{
    pid->ActualValue = 0;
    pid->e = 0;
    pid->e_last = 0;
    pid->e_last2 = 0;
    pid->e_sum = 0;
    pid->out = 0;

    pid->SetValue = default_SetValue;

    return YZF_OK;
}

float yzl_pid_default_pos(YzPID *pid,float actual)
{

    pid->ActualValue = actual;
    pid->e = pid->SetValue - pid->ActualValue;
    pid->e_sum += pid->e;

    PID_LIMIT(pid->e_sum,pid->min_out,pid->max_out);

    pid->out = pid->kp * pid->e + pid->ki * pid->e_sum + pid->kd * (pid->e - pid->e_last);

    pid->e_last2 = pid->e_last;
    pid->e_last = pid->e;

    PID_LIMIT(pid->out,pid->min_out,pid->max_out);

    return pid->out;
}

float yzl_pid_default_inc(YzPID *pid,float actual)
{

    pid->ActualValue = actual;
    pid->e = pid->SetValue - pid->ActualValue;

    pid->out = pid->kp*(pid->e-pid->e_last) + pid->ki * pid->e + pid->kd * (pid->e - pid->e_last * 2 + pid->e_last2);

    pid->e_last2 = pid->e_last;
    pid->e_last = pid->e;

    PID_LIMIT(pid->out,pid->min_out,pid->max_out);

    return pid->out;
}

float yzl_pid_neuro_pos(YzPID *pid, float actual)
{
    // ========== 1. 计算当前误差 ==========
    pid->ActualValue = actual;
    pid->e = pid->SetValue - pid->ActualValue;  // e(k)

    // ========== 2. 计算神经元的三路输入 ==========
    float x1 = pid->e;                                          // 积分项输入
    float x2 = pid->e - pid->e_last;                           // 比例/微分项输入
    float x3 = pid->e - 2.0f * pid->e_last + pid->e_last2;    // 二阶微分项输入

    // ========== 3. 权重归一化（防止输出过大） ==========
    float sum_w = fabsf(pid->ki) + fabsf(pid->kp) + fabsf(pid->kd);
    if (sum_w < 0.0001f) sum_w = 0.0001f;  // 防除零

    float w1_norm = pid->ki / sum_w;
    float w2_norm = pid->kp / sum_w;
    float w3_norm = pid->kd / sum_w;

    // ========== 4. 计算神经元输出（位置式） ==========
    // Δu = K_neuro * (w1_norm*x1 + w2_norm*x2 + w3_norm*x3)
    float delta_u = pid->k_neuro * (w1_norm * x1 + w2_norm * x2 + w3_norm * x3);

    // 位置式：u(k) = u(k-1) + Δu
    float output = pid->out + delta_u;

    // ========== 5. 输出限幅 ==========
    if (output > pid->max_out) output = pid->max_out;
    if (output < pid->min_out) output = pid->min_out;

    // ========== 6. 有监督Hebb学习规则：更新权重 ==========
    // 公式：Δw = η * e(k) * u(k) * x_i(k)
    float error_signal = pid->e;      // z(t) = e(t)
    float current_output = output;    // u(k)

    // 注意：w1对应积分(ki)，使用n_i学习率；w2对应比例(kp)，使用n_p学习率；w3对应微分(kd)，使用n_d学习率
    pid->ki += pid->n_i * error_signal * current_output * x1;   // 积分权重更新
    pid->kp += pid->n_p * error_signal * current_output * x2;   // 比例权重更新
    pid->kd += pid->n_d * error_signal * current_output * x3;   // 微分权重更新

    // 权重限幅，防止发散（可选，根据系统需要调整范围）
    PID_LIMIT(pid->ki,pid->k_max,pid->k_min);
    PID_LIMIT(pid->kp,pid->k_max,pid->k_min);
    PID_LIMIT(pid->kd,pid->k_max,pid->k_min);

    // ========== 7. 更新历史值 ==========
    pid->e_last2 = pid->e_last;
    pid->e_last = pid->e;
    pid->out = output;      // 保存输出用于下次累加

    return output;
}