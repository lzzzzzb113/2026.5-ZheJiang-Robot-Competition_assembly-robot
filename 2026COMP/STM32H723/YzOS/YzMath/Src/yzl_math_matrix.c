//
// Created by 32827 on 2026/4/8.
//

#include "yzl_math_matrix.h"

#include "yzl_mem.h"

#ifdef MARTIX_F32_ENABLE
void yzl_math_matrix_init_f32(y_matrix_f32 *matrix,y_i32 m,y_i32 n){
    matrix->m=m;
    matrix->n=n;
    matrix->x=(y_f32 *)YZL_MEM_MANAGER.pMalloc(sizeof(y_f32)*(m*n+1));
    memset(matrix->x,0,sizeof(y_f32)*(m*n+1));
    matrix->x=yzl_mem_address_align(matrix->x,sizeof(y_f32));
}

void yzl_math_matrix_deinit_f32(y_matrix_f32 *matrix){
    if(matrix->x!=YZF_NULL_PTR) {
        YZL_MEM_MANAGER.pFree(matrix->x);
    }
    matrix->x=YZF_NULL_PTR;
}

void yzl_math_matrix_clear_f32(y_matrix_f32 *matrix){
    for (int i = 0; i < matrix->m; i++) {
        for (int j = 0; j < matrix->n; j++) {
            y_matrix_get(matrix, i, j) = 0.0f;
        }
    }
}

void yzl_math_vector_init_f32(y_vector_f32 *vector, y_i32 n){
    vector->len=n;
    vector->x=(y_f32 *)YZL_MEM_MANAGER.pMalloc(sizeof(y_f32)*(n+1));
    memset(vector->x,0,sizeof(y_f32)*(n+1));
    vector->x=yzl_mem_address_align(vector->x,sizeof(y_f32));
}

void yzl_math_vector_deinit_f32(y_vector_f32 *vector){
    YZL_MEM_MANAGER.pFree(vector->x);
    vector->x=NULL;
}

void yzl_math_matrix_mul_f32(y_matrix_f32 *matrix_left,y_matrix_f32 *matrix_right,y_matrix_f32 *matrix_out){
    for(int i = 0; i < matrix_left->m; i++){
        for(int j = 0; j < matrix_right->n; j++){
            for (int k = 0; k < matrix_left->n; k++) {
                y_matrix_get(matrix_out,i,j) += y_matrix_get(matrix_left,i,k) * y_matrix_get(matrix_right,k,j);
            }
        }
    }
}

void yzl_math_matrix_mul_vector_f32(y_matrix_f32 *matrix_left, y_vector_f32 *vector_right, y_vector_f32 *vector_out){
    for(int i = 0; i < matrix_left->m; i++) {
        for (int k = 0; k < matrix_left->n; ++k) {
            vector_out->x[i] += y_matrix_get(matrix_left, i, k) * vector_right->x[k];
        }
    }
}

void yzl_math_matrix_add_f32(y_matrix_f32 *matrix_left, y_matrix_f32 *matrix_right, y_matrix_f32 *matrix_out){
    for (int i = 0; i < matrix_left->m; i++) {
        for (int j = 0; j < matrix_left->n; j++) {
            y_matrix_get(matrix_out, i, j) = y_matrix_get(matrix_left, i, j) + y_matrix_get(matrix_right, i, j);
        }
    }
}

void yzl_math_matrix_I_f32(y_matrix_f32 *matrix){
    for (int i = 0; i < (matrix->m>=matrix->n?matrix->n:matrix->m); i++) {
        y_matrix_get(matrix,i,i)=1;
    }
}

void yzl_math_matrix_print_f32(y_matrix_f32 *matrix){
    yz_printf("\n");
    for (int i = 0; i < matrix->m; ++i) {
        for (int j = 0; j < matrix->n; ++j) {
            yz_printf("%f ",y_matrix_get(matrix,i,j));
        }
        yz_printf("\n");
    }
}

void yzl_math_vector_print_f32(y_vector_f32 *vector){
    yz_printf("\n");
    for (int i = 0; i < vector->len; ++i) {
        yz_printf("%f ", vector->x[i]);
    }
    yz_printf("\n");
}

void yzl_math_matrix_test_f32()
{
    y_matrix_f32 a,b,c;
    yzl_math_matrix_init_f32(&a,4,4);
    yzl_math_matrix_init_f32(&b,4,4);
    yzl_math_matrix_init_f32(&c,4,4);
    yzl_math_matrix_I_f32(&a);
    y_matrix_get(&a,0,1) = 1;
    yzl_math_matrix_print_f32(&a);
    yzl_math_matrix_I_f32(&b);
    y_matrix_get(&b,1,0) = 1;
    yzl_math_matrix_print_f32(&b);

    yzl_math_matrix_add_f32(&a,&b,&c);
    yzl_math_matrix_print_f32(&c);

    yzl_math_matrix_clear_f32(&c);
    yzl_math_matrix_mul_f32(&a,&b,&c);
    yzl_math_matrix_print_f32(&c);

    yzl_math_matrix_deinit_f32(&a);
    yzl_math_matrix_deinit_f32(&b);
    yzl_math_matrix_deinit_f32(&c);
}
/**
 *  X X X ---- M
 *  X X X
 *  X X X
 *  |
 *  |
 *  N
 * */
void yzl_math_matrix_covariance_f32(y_f32 **x,y_i32 d,y_i32 len,y_matrix_f32 *cov){
    for (int i = 0; i < d; i++) {
        for (int j = 0; j < i+1; j++) {
            y_matrix_get(cov,i,j)=yzl_math_covariance_f32(x[i],x[j],1,1,len);
            if(j<i){
                y_matrix_T_get(cov,i,j)=y_matrix_get(cov,i,j);
            }
        }
    }
}
#endif


#if MARTIX_F64_ENABLE
void yzl_math_matrix_init_f64(y_matrix_f64 *matrix,y_i32 m,y_i32 n){
    matrix->m=m;
    matrix->n=n;
    matrix->x=(y_f64 *)YZL_MEM_MANAGER.pMalloc(sizeof(y_f64)*(m*n+1));
    memset(matrix->x,0,sizeof(y_f64)*(m*n+1));
    matrix->x=yzl_mem_address_align(matrix->x,sizeof(y_f64));
}

void yzl_math_matrix_deinit_f64(y_matrix_f64 *matrix){
    if(matrix->x!=YZF_NULL_PTR){
        YZL_MEM_MANAGER.pFree(matrix->x);
    }
    matrix->x=YZF_NULL_PTR;
}

void yzl_math_matrix_clear_f64(y_matrix_f64 *matrix){
    for (int i = 0; i < matrix->m; i++) {
        for (int j = 0; j < matrix->n; j++) {
            y_matrix_get(matrix, i, j) = 0.0;
        }
    }
}

void yzl_math_vector_init_f64(y_vector_f64 *vector, y_i32 n){
    vector->len=n;
    vector->x=(y_f64 *)YZL_MEM_MANAGER.pMalloc(sizeof(y_f64)*(n+1));
    memset(vector->x,0,sizeof(y_f64)*(n+1));
    vector->x=yzl_mem_address_align(vector->x,sizeof(y_f64));
}

void yzl_math_vector_deinit_f64(y_vector_f64 *vector){
    YZL_MEM_MANAGER.pFree(vector->x);
    vector->x=NULL;
}

void yzl_math_matrix_mul_vector_f64(y_matrix_f64 *matrix_left, y_vector_f64 *vector_right, y_vector_f64 *vector_out){
    for(int i = 0; i < matrix_left->m; i++) {
        for (int k = 0; k < matrix_left->m; ++k) {
            vector_out->x[i] +=  y_matrix_get(matrix_left, i, k) * vector_right->x[k];
        }
    }
}
void yzl_math_matrix_mul_f64(y_matrix_f64 *matrix_left,y_matrix_f64 *matrix_right,y_matrix_f64 *matrix_out){
    for(int i = 0; i < matrix_left->m; i++) {
        for (int j = 0; j < matrix_right->n; j++) {
            for (int k = 0; k < matrix_left->n; ++k) {
                y_matrix_get(matrix_out, i, j) += y_matrix_get(matrix_left, i, k) * y_matrix_get(matrix_right, k, j);
            }
        }
    }
}

void yzl_math_matrix_add_f64(y_matrix_f64 *matrix_left, y_matrix_f64 *matrix_right, y_matrix_f64 *matrix_out){
    for (int i = 0; i < matrix_left->m; i++) {
        for (int j = 0; j < matrix_left->n; j++) {
            y_matrix_get(matrix_out, i, j) = y_matrix_get(matrix_left, i, j) + y_matrix_get(matrix_right, i, j);
        }
    }
}

void yzl_math_matrix_I_f64(y_matrix_f64 *matrix){
    for (int i = 0; i < (matrix->m>=matrix->n?matrix->n:matrix->m); i++) {
        y_matrix_get(matrix,i,i)=1;
    }
}

void yzl_math_matrix_print_f64(y_matrix_f64 *matrix){
    yz_printf("\n");
    for (int i = 0; i < matrix->m; ++i) {
        for (int j = 0; j < matrix->n; ++j) {
            yz_printf("%lf ",y_matrix_get(matrix,i,j));
        }
        yz_printf("\n");
    }
}

void yzl_math_vector_print_f64(y_vector_f64 *vector){
    yz_printf("\n");
    for (int i = 0; i < vector->len; ++i) {
        yz_printf("%lf ", vector->x[i]);
    }
    yz_printf("\n");
}

void yzl_math_matrix_covariance_f64(y_f64 **x,y_i32 d,y_i32 len,y_matrix_f64 *cov){
    for (int i = 0; i < d; i++) {
        for (int j = 0; j < i+1; j++) {
            y_matrix_get(cov,i,j)=yzl_math_covariance_f64(x[i],x[j],1,1,len);
            if(j<i){
                y_matrix_T_get(cov,i,j)=y_matrix_get(cov,i,j);
            }
        }
    }
}
#endif
