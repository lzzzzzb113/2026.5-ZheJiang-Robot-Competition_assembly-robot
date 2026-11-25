//
// Created by 32827 on 2024/11/22.
//

#ifndef YZL_MATH_H
#define YZL_MATH_H

#include "yzx_base.h"
#include "yzl.h"

#if YZL_MATH_USE_HAL
#include "arm_math.h"
#endif

#ifdef   __cplusplus
extern "C"
{
#endif

//三角函数精度
#define YZL_TRIG_EPSILON 0.006
#define YZL_M_PI        3.1415926535897932384626433832795f
#define YZL_M_PI_2      (3.1415926535897932384626433832795f / 2)
#define	YZL_LUT_NUM		4096*2	/*该值越大表精度越高,精度为atan(1/LUT_NUM)*/    //Need 4K RAM

#define COS arm_cos_f32
#define SIN arm_sin_f32

typedef float y_f32;
typedef double y_f64;
typedef int y_i32;
typedef uint64_t y_i64;


typedef struct {
    y_f32 mean;
    y_f32 variance;
} y_normal_distribution;

y_f64 yzl_math_fast_ln(y_f64 x);

y_f32 yzl_arccos_f32(y_f32 x);
y_f32 yzl_arcsin_f32(y_f32 x);
y_f32 yzl_arctan_f32(y_f32 x);

void yzl_atan2_tab_init(void);
y_f32 yzl_atan2_tab_calc(y_f32 y, y_f32 x);

/**
 * @description: 计算均值
 *
 * @param data 数据
 * @param step 步长
 * @param len 长度
 * */
y_f64 yzl_math_avg_f32(y_f32 *data, y_i32 step, y_i32 len);

y_f64 yzl_math_avg_f64(y_f64 *data, y_i32 step, y_i32 len);

/**
 * @description: 计算方差
 *
 * @param data 数据
 * @param step 步长
 * */
y_f64 yzl_math_variance_f32(y_f32 *x, y_i32 step, y_i32 len);

y_f64 yzl_math_variance_f64(y_f64 *x, y_i32 step, y_i32 len);

/**
 * @description: 计算协方差
 *
 * @param x 数据1
 * @param y 数据2
 * @param step_x 数据1步长
 * @param step_y 数据2步长
 * @param len 长度
 * */
y_f64 yzl_math_covariance_f32(y_f32 *x, y_f32 *y, y_i32 step_x, y_i32 step_y, y_i32 len);

y_f64 yzl_math_covariance_f64(y_f64 *x, y_f64 *y, y_i32 step_x, y_i32 step_y, y_i32 len);

void yzl_math_normal_distribution_mul(y_normal_distribution *dst, y_normal_distribution *src1, y_normal_distribution *src2);

#ifdef   __cplusplus
}
#endif
#endif //YZL_MATH_H
