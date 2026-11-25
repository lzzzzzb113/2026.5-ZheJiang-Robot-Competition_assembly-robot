//
// Created by 32827 on 2024/11/23.
//

#ifndef YZL_AI_BAYES_FILTERS_H
#define YZL_AI_BAYES_FILTERS_H

#include "yzx_base.h"
#include "yzl_math.h"

typedef struct
{
    y_normal_distribution  prior;
    y_normal_distribution  likelihood;
    y_normal_distribution  posterior;
} y_bayes_filter;

void yzl_bayes_filter_update(y_bayes_filter * filter);

typedef struct
{
    y_f32 def_1;//为1的概率
    y_f32 pr_0; //测量为0,1的概率
    y_f32 pr_1; //侧廊为1,1的概率
    y_f32 l;    //中间值
    y_f32 confi;//当前1的置信度
} y_static_binary_bayes_filter;

/**
 *  静态二值贝叶斯滤波
 *
 * @param filter 过滤器实例
 * @param z 测量准确度
 */
y_f32 yzl_bayes_filter_static_binary(y_static_binary_bayes_filter * filter,YZF_BOOL z);
#endif //YZL_AI_BAYES_FILTERS_H
