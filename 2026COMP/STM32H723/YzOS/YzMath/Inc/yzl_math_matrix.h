//
// Created by 32827 on 2026/4/8.
//

#ifndef EMBPNN_YZL_MATH_MATRIX_H
#define EMBPNN_YZL_MATH_MATRIX_H

#include "yzl_math.h"

#ifdef   __cplusplus
extern "C"
{
#endif

#define MARTIX_F64_ENABLE 1
#define MARTIX_F32_ENABLE 1

#define y_matrix_get(D, i, j) (((D)->x)[i*((D)->m)+j])
#define y_matrix_T_get(D, i, j) (((D)->x)[j*((D)->n)+i])

#ifdef MARTIX_F32_ENABLE

#define y_vector_size_f32(D) (sizeof(y_f32)*((D)->len))
#define y_matrix_size_f32(D) (sizeof(y_f32)*((D)->m)*((D)->n))

typedef struct {
    y_f32 *x;
    y_i32 len;
} y_vector_f32;

typedef struct {
    y_f32 *x;
    y_i32 m;
    y_i32 n;
} y_matrix_f32;

/**
 *  初始化矩阵
 *
 * @param matrix 提供的矩阵句柄
 * @param m 行数
 * @param n 列数
 */
void yzl_math_matrix_init_f32(y_matrix_f32 *matrix,y_i32 m,y_i32 n);


/**
 *  释放矩阵内存
 * @param matrix 提供的矩阵句柄
 */
void yzl_math_matrix_deinit_f32(y_matrix_f32 *matrix);

/**
 *  全部填充零
 * @param matrix 提供的矩阵句柄
 */
void yzl_math_matrix_clear_f32(y_matrix_f32 *matrix);

void yzl_math_vector_init_f32(y_vector_f32 *vector, y_i32 n);

void yzl_math_vector_deinit_f32(y_vector_f32 *vector);

void yzl_math_matrix_mul_f32(y_matrix_f32 *matrix_left,y_matrix_f32 *matrix_right,y_matrix_f32 *matrix_out);

void yzl_math_matrix_mul_vector_f32(y_matrix_f32 *matrix_left, y_vector_f32 *vector_right, y_vector_f32 *vector_out);

void yzl_math_matrix_add_f32(y_matrix_f32 *matrix_left, y_matrix_f32 *matrix_right, y_matrix_f32 *matrix_out);

void yzl_math_matrix_I_f32(y_matrix_f32 *matrix);

void yzl_math_matrix_print_f32(y_matrix_f32 *matrix);

void yzl_math_vector_print_f32(y_vector_f32 *vector);

void yzl_math_matrix_test_f32();
/**
 * @description: 计算谐方差矩阵
 *
 * @param x 数据
 * @param d 维度
 * @param len 长度
 * */
void yzl_math_matrix_covariance_f32(y_f32 **x, y_i32 d, y_i32 len, y_matrix_f32 *cov);
#endif

#if MARTIX_F64_ENABLE

#define y_matrix_size_f64(D) (sizeof(y_f64)*((D)->m)*((D)->n))
#define y_vector_size_f64(D) (sizeof(y_f64)*((D)->len))

typedef struct {
    y_f64 *x;
    y_i32 len;
} y_vector_f64;

typedef struct {
    y_f64 *x;
    y_i32 m;
    y_i32 n;
} y_matrix_f64;


/**
 *  初始化矩阵
 *
 * @param matrix 提供的矩阵句柄
 * @param m 行数
 * @param n 列数
 */
void yzl_math_matrix_init_f64(y_matrix_f64 *matrix,y_i32 m,y_i32 n);

/**
 *  释放矩阵内存
 * @param matrix 提供的矩阵句柄
 */
void yzl_math_matrix_deinit_f64(y_matrix_f64 *matrix);

/**
 *  全部填充零
 * @param matrix 提供的矩阵句柄
 */
void yzl_math_matrix_clear_f64(y_matrix_f64 *matrix);

void yzl_math_vector_init_f64(y_vector_f64 *vector, y_i32 n);

void yzl_math_vector_deinit_f64(y_vector_f64 *vector);

void yzl_math_matrix_mul_f64(y_matrix_f64 *matrix_left, y_matrix_f64 *matrix_right, y_matrix_f64 *matrix_out);

void yzl_math_matrix_mul_vector_f64(y_matrix_f64 *matrix_left, y_vector_f64 *vector_right, y_vector_f64 *vector_out);

void yzl_math_matrix_add_f64(y_matrix_f64 *matrix_left, y_matrix_f64 *matrix_right, y_matrix_f64 *matrix_out);

void yzl_math_matrix_I_f64(y_matrix_f64 *matrix);

void yzl_math_vector_print_f64(y_vector_f64 *vector);

void yzl_math_matrix_print_f64(y_matrix_f64 *matrix);

void yzl_math_matrix_covariance_f64(y_f64 **x, y_i32 d, y_i32 len, y_matrix_f64 *cov);

#endif


#ifdef   __cplusplus
}
#endif

#endif //EMBPNN_YZL_MATH_MATRIX_H