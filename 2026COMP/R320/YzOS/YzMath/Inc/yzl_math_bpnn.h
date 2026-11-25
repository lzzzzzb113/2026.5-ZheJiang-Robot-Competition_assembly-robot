//
// Created by 32827 on 2026/4/8.
//

#ifndef EMBPNN_YZL_MATH_BPNN_H
#define EMBPNN_YZL_MATH_BPNN_H

#include "yzl_math_matrix.h"
#include "ff.h"

#ifdef   __cplusplus
extern "C"
{
#endif

#define ANN_DATA_TYPE y_f32
#define ACTIVATION_TYPE 0
#define LOOKUP_SIZE 4096

typedef ANN_DATA_TYPE (*genann_actfun)(void *ann, ANN_DATA_TYPE a);
typedef void (*genann_randomize)(void *ann);

typedef struct  {
    int inputs;          // 输入层神经元个数
    int hidden_layers;   // 隐藏层层数
    int hidden;          // 每个隐藏层的神经元个数
    int outputs;         // 输出层神经元个数

    genann_actfun activation_hidden;  // 隐藏层激活函数
    genann_actfun activation_output;  // 输出层激活函数
    genann_randomize randomize;       // 权重初始化函数

    int total_weights;   // 权重总数
    int total_neurons;   // 神经元总数（含输入层）

    ANN_DATA_TYPE *weight;      // 权重数组（长度为 total_weights）
    ANN_DATA_TYPE *output;      // 所有神经元的输出值（长度为 total_neurons）
    ANN_DATA_TYPE *delta;       // 误差项数组（隐藏层+输出层，长度为 total_neurons - inputs）

    ANN_DATA_TYPE interval;
    ANN_DATA_TYPE sigmod_max;
    ANN_DATA_TYPE sigmod_min;
    ANN_DATA_TYPE sigmod_sheet[LOOKUP_SIZE];
} genann;

/**
 *  创建神经元
 * @param inputs 输入层神经元个数
 * @param hidden_layers 隐藏层层数
 * @param hidden 每个隐藏层的神经元个数
 * @param outputs 输出层神经元个数
 * @return
 */
genann *genann_init(int inputs, int hidden_layers, int hidden, int outputs);

/**
 *  释放内存
 * @param ann 预配置神经网络
 */
void genann_free(genann *ann);

/**
 *  运行
 * @param ann 预配置神经网络
 * @param inputs 输入
 * @return 输出
 */
ANN_DATA_TYPE const *genann_run(genann const *ann, ANN_DATA_TYPE const *inputs);

/**
 * 单词训练
 * @param ann 预配置神经网络
 * @param inputs 输入
 * @param desired_outputs 期望输出
 * @param learning_rate 学习率
 */
void genann_train(genann const *ann, ANN_DATA_TYPE const *inputs, ANN_DATA_TYPE const *desired_outputs, ANN_DATA_TYPE learning_rate);

/**
 * 初始化sigmoid激活函数查表初始化
 * @param ann 预配置神经网络
 */
void genann_init_sigmoid_lookup(genann *ann);

/**
 * 神经读取
 * @param in 文件指针
 * @return 加载的神经网络
 */
genann *genann_read_fatfs(FIL *in);

/**
 * 神经网络保存
 * @param ann 预配置神经网络
 * @param out 文件指针
 */
void genann_write_fatfs(genann *ann, FIL *out);


#ifdef   __cplusplus
}
#endif


#endif //EMBPNN_YZL_MATH_BPNN_H