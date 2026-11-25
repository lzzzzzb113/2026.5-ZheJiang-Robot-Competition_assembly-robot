//
// Created by 32827 on 2025/11/9.
//
#include "yzl_pwm_foc.h"

#include "yzl_spi_qmi8658a.h"
#include "yz_log.h"
#include "yz_time.h"

void yzl_pwm_foc_set(YZ_PWM_FOC *foc,float u, float v, float w){

    if(u>1.0f){
        u=1.0f;
    }else if(u<0.0f){
        u=0.0f;
    }
    if(v>1.0f){
        v=1.0f;
    }else if(v<0.0f){
        v=0.0f;
    }
    if(w>1.0f){
        w=1.0f;
    }else if(w<0.0f){
        w=0.0f;
    }

    __disable_irq();
    DL_Timer_setCaptureCompareValue(foc->pwm_control1, (foc->MaxDuty-foc->MinDuty)*u+foc->MinDuty,foc->ch1);
    DL_Timer_setCaptureCompareValue(foc->pwm_control2, (foc->MaxDuty-foc->MinDuty)*v+foc->MinDuty,foc->ch2);
    DL_Timer_setCaptureCompareValue(foc->pwm_control3, (foc->MaxDuty-foc->MinDuty)*w+foc->MinDuty,foc->ch3);
    __enable_irq();
}


#define deg_to_rad(a) (M_PI * (a) / 180)
#define SQRTf3 1.7320508f
#define SQRTQ3 _IQ24(1.7320508)
void yzl_pwm_foc_clarke(float ia, float ib,float *alpha, float *beta){
    //等幅变换
    //K = 2/3 ，最高幅度与三相值相等
    *alpha = ia;
    *beta = (ia+2*ib)/SQRTf3;
}

void yzl_pwm_foc_clarke_q24(_iq24 ia, _iq24 ib,_iq24 *alpha, _iq24 *beta){
    //等幅变换
    //K = 2/3 ，最高幅度与三相值相等
    *alpha = ia;
    *beta = _IQ24div(ia+2*ib,SQRTQ3);
}

void yzl_pwm_foc_park(float radius,float alpha, float beta, float *d, float *q){
    float cr = cosf(radius);
    float sr = sinf(radius);
    *d=alpha*cr + beta*sr;
    *q=-alpha*sr + beta*cr;
}

void yzl_pwm_foc_park_q24(_iq24 radius,_iq24 alpha, _iq24 beta, _iq24 *d, _iq24 *q)
{
    _iq24 cr = _IQ24cos(radius);
    _iq24 sr = _IQ24sin(radius);
    *d=_IQ24mpy(alpha,cr) + _IQ24mpy(beta,sr);
    *q=_IQ24mpy(-alpha,sr) + _IQ24mpy(beta,cr);
}

void yzl_pwm_foc_inv_park(YZ_PWM_FOC *foc){
    float cr = cosf(foc->current_sector_rad);
    float sr = sinf(foc->current_sector_rad);
    foc->iAlpha=foc->id*cr - foc->iq*sr;
    foc->iBeta=foc->id*sr + foc->iq*cr;
}

void yzl_pwm_foc_inv_park_q24(YZ_PWM_FOC *foc)
{
    _iq24 cr = _IQ24cos(foc->current_sector_rad_q24);
    _iq24 sr = _IQ24sin(foc->current_sector_rad_q24);
    foc->iAlpha_q24=_IQ24mpy(foc->id_q24,cr) - _IQ24mpy(foc->iq_q24,sr);
    foc->iBeta_q24=_IQ24mpy(foc->id_q24,sr) + _IQ24mpy(foc->iq_q24,cr);
}

void yzl_pwm_foc_svpwm_qd(YZ_PWM_FOC *foc)
{
    static const float rad60 = deg_to_rad(60);
    static const int KMAP[8] = {4,6,5,5,3,1,2,2};
    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    float t_m,t_n,t_0;
    int k;

    yzl_pwm_foc_inv_park(foc);

    float alpha,beta;
    alpha = foc->iAlpha;
    beta = foc->iBeta;

    k = (beta>0 ? 4 : 0) + (alpha > 0 ? 1 : 0);
    k += (fabsf(beta) > (fabsf(alpha) * SQRTf3)) ? 2 : 0;
    k = KMAP[k];

    t_m = sinf(rad60*k)*alpha - cosf(rad60*k)*beta;
    t_n = beta*cosf(rad60*(k-1)) - alpha*sinf(rad60*(k-1));
    t_0 = 1 - t_m - t_n;

    float d_u = t_m * v[(k - 1)%6][0] + t_n * v[k % 6][0] + t_0 / 2;
    float d_v = t_m * v[(k - 1)%6][1] + t_n * v[k % 6][1] + t_0 / 2;
    float d_w = t_m * v[(k - 1)%6][2] + t_n * v[k % 6][2] + t_0 / 2;

    // yz_debug("%d,%.3f,%.3f,%.3f,%.3f\n",k,d_u,d_v,d_w,rad);
    yzl_pwm_foc_set(foc,d_u,d_v,d_w);
}

void yzl_pwm_foc_svpwm_qd_q24(YZ_PWM_FOC *foc)
{
    static const _iq24 rad60 = _IQ24(deg_to_rad(60));
    static const int KMAP[8] = {4,6,5,5,3,1,2,2};
    const _iq24 v[6][3] = {{_IQ24(1), _IQ24(0), _IQ24(0)},
                            {_IQ24(1), _IQ24(1), _IQ24(0)},
                            {_IQ24(0), _IQ24(1), _IQ24(0)},
                            {_IQ24(0), _IQ24(1), _IQ24(1)},
                            {_IQ24(0), _IQ24(0), _IQ24(1)},
                            {_IQ24(1), _IQ24(0), _IQ24(1)}};
    _iq24 t_m,t_n,t_0;
    int k;
    yzl_pwm_foc_inv_park_q24(foc);
    _iq24 alpha,beta;
    alpha = foc->iAlpha_q24;
    beta = foc->iBeta_q24;
    k = (beta>_IQ24(0) ? 4 : 0) + (alpha > _IQ24(0) ? 1 : 0);
    k += (_IQabs(beta) > _IQ24mpy(_IQabs(alpha),SQRTQ3)) ? 2 : 0;
    k = KMAP[k];

    t_m = _IQ24mpy(_IQ24sin(rad60*k),alpha) - _IQ24mpy(_IQ24cos(rad60*k),beta);
    t_n = _IQ24mpy(beta,_IQ24cos(rad60*(k-1))) - _IQ24mpy(alpha,_IQ24sin(rad60*(k-1)));
    t_0 = _IQ24(1) - t_m - t_n;

    _iq24 d_u = _IQ24mpy(t_m,v[(k - 1)%6][0]) + _IQ24mpy(t_n,v[k % 6][0]) + _IQ24div(t_0,_IQ24(2));
    _iq24 d_v = _IQ24mpy(t_m,v[(k - 1)%6][1]) + _IQ24mpy(t_n,v[k % 6][1]) + _IQ24div(t_0,_IQ24(2));
    _iq24 d_w = _IQ24mpy(t_m,v[(k - 1)%6][2]) + _IQ24mpy(t_n,v[k % 6][2]) + _IQ24div(t_0,_IQ24(2));

    yzl_pwm_foc_set(foc,_IQ24toF(d_u),_IQ24toF(d_v),_IQ24toF(d_w));
}


void ResetRealZero(YZ_PWM_FOC *foc)
{
    foc->iq = 0.0f;
    foc->id = 0.8f;
    foc->current_sector_rad = 0.0f;
    yz_time_block_delay_tick(300);
    foc->update_angle(foc);
    foc->zero_raw_rad = foc->cur_raw_rad;//设置0位
    foc->update_angle(foc);//冲刷
    yz_time_block_delay_tick(100);
}

YZF_RESULT yzl_foc_init(YZ_PWM_FOC *foc)
{
    yzl_pid_default_init(&foc->position_pid, 0.0f);

    yzl_pid_default_init(&foc->speed_pid,0.0f);

    yzl_pid_default_init(&foc->iq_pid,0.0f);

    yzl_pid_default_init(&foc->id_pid, 0.0f);

    foc->position_pid.SetValue = 0.0f;
    foc->speed_pid.SetValue = 0.0f;
    foc->iq_pid.SetValue =0.0f;
    foc->id_pid.SetValue = 0.0f;

    foc->iq_pid.out = 0.0f;
    foc->iq_pid.ActualValue = 0.0f;
    foc->id_pid.out = 0.0f;
    foc->id_pid.ActualValue = 0.0f;
    foc->speed_pid.out = 0.0f;
    foc->speed_pid.ActualValue = 0.0f;
    foc->position_pid.out = 0.0f;
    foc->position_pid.ActualValue = 0.0f;

    for (int i=0;i<3;i++)
    {
        foc->speed_filter[i]=0.0f;
    }

    ResetRealZero(foc);

    return YZF_OK;
}

float yzl_foc_sliding_window_mean_filter(float *input, float newdata, int length)
{
    //去除最大最小值取平均
    int i = 0;
    float sum = 0;
    float max = -2, min = 2;
    for (i = 0; i < length - 1; i++)
    {
        input[i] = input[i + 1];
        if (input[i] != 0)
        {
            sum += input[i];
        }
        if(input[i]>max){
            max=input[i];
        }
        if(input[i]<min){
            min=input[i];
        }
    }
    input[length - 1] = newdata;
    if(newdata>max){
        max=newdata;
    }
    if(newdata<min){
        min=newdata;
    }
    sum += input[length - 1] - max - min;
    return sum / (length - 2);
}

extern uint16_t adc_u[8];
extern uint16_t adc_v[8];
uint32_t icount = 0;
uint32_t speedCount = 0;
uint32_t positionCount = 0;
#define RAW_RAD_K 0.00019174f
extern QMI8658 qmi8658;
float gz_sum=0.0f;
int angle_sample_rate = 0;
extern float gz_f_fix;
YZF_RESULT yzl_foc_update(YZ_PWM_FOC *foc)
{
    icount++;
    foc->iu = -((((float)adc_u[0])/4095.0f - 0.5f) * 3.3f);
    foc->iv = -((((float)adc_v[0])/4095.0f - 0.5f) * 3.3f);
    foc->iu_q24 = _IQ24(foc->iu);
    foc->iv_q24 = _IQ24(foc->iv);

    foc->update_angle(foc);
    yzl_pwm_foc_clarke_q24(foc->iu_q24 ,foc->iv_q24,&foc->iAlpha_q24,&foc->iBeta_q24);
    yzl_pwm_foc_park_q24(foc->current_sector_rad_q24,foc->iAlpha_q24,foc->iBeta_q24,&foc->id_q24,&foc->iq_q24); //转换电流坐标系

    if (icount > positionCount)
    {
        positionCount = icount + 5;
        foc->position_pid.SetValue = 0;
        yzl_pid_neuro_adaptive_simplified(&foc->position_pid,(float) foc->position_err_rad*RAW_RAD_K);
        foc->speed_pid.SetValue = foc->position_pid.out;

        // yz_printf("%.3f,%.3f,%.3f,%.3f\n",foc->id_pid.e,foc->iq_pid.e,foc->speed_pid.e,foc->position_pid.e);
        // yz_printf("%.3f,%.3f\n",foc->position_pid.ActualValue,foc->position_pid.SetValue);
    }

    float f_speed = yzl_foc_sliding_window_mean_filter(foc->speed_filter,foc->current_speed_rad,3);
    if (icount > speedCount)
    {
        speedCount = icount + 2;
        yzl_pid_neuro_adaptive_simplified(&foc->speed_pid,f_speed);
    }

    if (icount % 20 == 0)
    {
        yzl_qmi8658_update_data(&qmi8658);
        gz_sum += (qmi8658.gz_f-gz_f_fix)*(((float)angle_sample_rate)*0.00005f);
        if (gz_sum < 0.0)
        {
            gz_sum += 360.0f;
        }else if (gz_sum > 360.0)
        {
            gz_sum -= 360.0f;
        }
        float ag = gz_sum/360.0f;
        foc->position_offset_rad  = (int)(ag*32767.0);
        angle_sample_rate = 0;
    }else
    {
        angle_sample_rate += foc->AngleSampleRate;
    }

    foc->iq_pid.SetValue = foc->speed_pid.out;
    foc->id_pid.SetValue = (1.0f-foc->iq_pid.SetValue)*0.05f;

    yzl_pid_default_pos(&foc->iq_pid, foc->iq);
    yzl_pid_default_pos(&foc->id_pid, foc->id);

    foc->iq = foc->iq_pid.out;
    foc->id = foc->id_pid.out;
    foc->iq_q24 = _IQ24(foc->iq);
    foc->id_q24 = _IQ24(foc->id);

    yzl_pwm_foc_svpwm_qd_q24(foc);

    return YZF_OK;
}