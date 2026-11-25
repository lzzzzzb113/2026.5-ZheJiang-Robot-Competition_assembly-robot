//
// Created by 32827 on 2024/11/22.
//
#include "yzl_math.h"
#include "yzl_mem.h"

#define LN2 0.6931471805599453 // ln(2) 的近似值[reference:9]
y_f64 yzl_math_fast_ln(y_f64 x) {
    if (x <= 0.0) {
        return -1.0;
    }

    int exponent = 0;
    while (x > 2.0) {
        x /= 2.0;
        exponent++;
    }
    while (x < 0.5) {
        x *= 2.0;
        exponent--;
    }

    y_f64 y = (x - 1.0) / (x + 1.0);
    y_f64 y2 = y * y;
    y_f64 sum = 0.0;
    y_f64 term = y;
    int n = 1;

    while (term > 1e-12 || term < -1e-12) {
        sum += term / (2.0 * n - 1.0);
        term *= y2;
        n++;
    }
    y_f64 log_result = 2.0 * sum;

    return log_result + exponent * LN2;
}

y_f32 yzl_abs_f64(y_f32 x){
    return (x > 0) ? x : -x;
}
/*
 * arcCos w/ inverse and newton's method
 * function to find roots of :  cos(x) - x
 * newton's method formula   :  xk+1 = xk - (cos(x) - x)/-sin(x)
 * 				     = xk + (cos(x) - x)/sin(x)
 */
y_f32 yzl_arccos_f32(y_f32 x) {
    if(x==1.0f){
        return 0.0f;
    }
    if(x==-1.0f){
        return YZL_M_PI;
    }
    y_f64 inp_x = 1.0; // initial guess
    y_f64 temp_res = 0.1; // xk to be compared to epsilon
    while (yzl_abs_f64(temp_res) > YZL_TRIG_EPSILON) {
        temp_res = (COS(inp_x) - x) / (y_f64)SIN(inp_x); // calculating f(xk)/f'(xk)
        inp_x += temp_res; // xk+1 += xk
    }
     return inp_x;
}

/*
 * arcTan with the formula arctan = arccos(1/sq_root(1+x^2))
 */
y_f32 yzl_arctan_f32(y_f32 x) {
    y_f32  temp;
    arm_sqrt_f32(1 + (x * x),&temp);
    return yzl_arccos_f32(1 / (temp));
}

y_f32 g_atan_tab[YZL_LUT_NUM];
void yzl_atan2_tab_init(void){
    uint32_t i = 0;

    for (i = 0; i < YZL_LUT_NUM; i++)
    {
        g_atan_tab[i] = yzl_arctan_f32((y_f32)i / (YZL_LUT_NUM - 1));
    }
}

y_f32 yzl_arcsin_f32(y_f32 x)
{
    float d=1;
    float t=x;
    unsigned char cnt = 1;
    float result = 0;
    float X2 = x*x;

    if (x >= 1.0f)
    {
        return YZL_M_PI/2;
    }
    if (x <= -1.0f)
    {
        return -YZL_M_PI/2;
    }
    do
    {
        result += t / (d * ((cnt << 1) - 1));
        t *= X2 * ((cnt << 1) - 1);//
        d *= (cnt << 1);//2 4 6 8 10 ...
        cnt++;
    }while(cnt <= 6);

    return result;
}
y_f32 yzl_atan2_tab_calc(y_f32 y, y_f32 x){
    y_f32 Y_X, Y_X_temp;
    y_f32 result = 0;

    Y_X = (x == 0) ? (y_f32)x / y : (y_f32)y / x;
    Y_X_temp = Y_X;

    if (y == 0)
    {
        if (x >= 0)
            return 0;
        else
            return (YZL_M_PI);  //Return Pi
    }
    else if (x > 0 && y > 0)
    {
        //第一象限
        Y_X = (Y_X <= 1) ? Y_X  : 1 / Y_X;
        result = (Y_X_temp > 1)? YZL_M_PI_2 - g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))] : g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))];
    }
    else if (x < 0 && y > 0)
    {
        //第二象限
        Y_X = (Y_X < -1)? (-1 / Y_X) : -Y_X;
        result = (Y_X_temp >= -1) ? YZL_M_PI - g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))] : //小于45°
                 YZL_M_PI_2 + g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))];
    }
    else if (x < 0 && y < 0)
    {
        //第三象限
        Y_X = (Y_X <= 1) ? Y_X : 1 / Y_X;
        result = (Y_X_temp <= 1) ? g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))] - YZL_M_PI: -YZL_M_PI_2 - g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))];
    }
    else if(x > 0 && y < 0)
    {
        //第四象限
        Y_X = (Y_X < -1) ?(-1 / Y_X) : -Y_X;
        result = (Y_X_temp >= -1) ? -g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))] : //小于45°
                 -(YZL_M_PI_2 - g_atan_tab[(int16_t)(Y_X*(YZL_LUT_NUM - 1))]);
    }
    else if(x == 0 && y > 0)
    {
        result = YZL_M_PI_2;
    }
    else
    {
        result = -YZL_M_PI_2;
    }

    return result;
}

y_f64 yzl_math_avg_f32(y_f32 *data,y_i32 step,y_i32 len){
    y_f64 sum = 0;
    y_i32 l=len;
    while (l--){
        sum += *data;
        data += step;
    }
    return sum/(y_f64)len;
}
y_f64 yzl_math_avg_f64(y_f64 *data,y_i32 step, y_i32 len){
    y_f64 sum = 0;
    y_i32 l=len;
    while (l--){
        sum += *data;
        data += step;
    }
    return sum/(y_f64)len;
}
y_f64 yzl_math_variance_f32(y_f32 *x,y_i32 step,y_i32 len){
    y_f64 x_avg=yzl_math_avg_f32(x,step,len);
    y_f64 sum=0;
    for (int i = 0; i < len; ++i) {
        sum += (x[i*step]-x_avg)*(x[i*step]-x_avg);
    }
    return sum/(y_f64)len;
}
y_f64 yzl_math_variance_f64(y_f64 *x,y_i32 step,y_i32 len){
    y_f64 x_avg=yzl_math_avg_f64(x,step,len);
    y_f64 sum=0;
    for (int i = 0; i < len; ++i) {
        sum += (x[i*step]-x_avg)*(x[i*step]-x_avg);
    }
    return sum/(y_f64)len;
}
y_f64 yzl_math_covariance_f32(y_f32 *x,y_f32 *y,y_i32 step_x,y_i32 step_y,y_i32 len){
    y_f64 x_avg=yzl_math_avg_f32(x,step_x,len);
    y_f64 y_avg=yzl_math_avg_f32(y,step_y,len);
    y_f64 sum=0;
    for (int i = 0; i < len; ++i) {
        sum += (x[i*step_x]-x_avg)*(y[i*step_y]-y_avg);
    }
    return sum/(y_f64)len;
}
y_f64 yzl_math_covariance_f64(y_f64 *x,y_f64 *y,y_i32 step_x,y_i32 step_y,y_i32 len){
    y_f64 x_avg=yzl_math_avg_f64(x,step_x,len);
    y_f64 y_avg=yzl_math_avg_f64(y,step_y,len);
    y_f64 sum=0;
    for (int i = 0; i < len; ++i) {
        sum += (x[i*step_x]-x_avg)*(y[i*step_y]-y_avg);
    }
    return sum/(y_f64)len;
}
void yzl_math_normal_distribution_mul(y_normal_distribution *dst, y_normal_distribution *src1,  y_normal_distribution *src2){
    dst->mean=(src1->mean*src2->variance+src2->mean*src1->variance)/(src1->variance+src2->variance);
    dst->variance=src1->variance*src2->variance/(src1->variance+src2->variance);
}