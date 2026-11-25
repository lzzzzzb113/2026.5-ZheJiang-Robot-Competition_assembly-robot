//
// Created by 32827 on 2024/11/23.
//
#include "yzl_ai_bayes_filters.h"
void yzl_bayes_filter_update(y_bayes_filter * filter){
    yzl_math_normal_distribution_mul(&filter->posterior, &filter->prior, &filter->likelihood);
}

y_f32 yzl_bayes_filter_static_binary(y_static_binary_bayes_filter * filter,YZF_BOOL z)
{
    y_f32 p1_x = z == YZF_TRUE ? filter->pr_1 : filter->pr_0;
    filter->l = filter->l + yzl_math_fast_ln(p1_x/(1-p1_x)) - yzl_math_fast_ln(filter->confi/(1-filter->confi));
    filter->confi = 1 - 1/(1+exp(filter->l));
    return filter->confi;
}