//
// Created by 32827 on 2026/4/8.
//

#include "yzl_math_bpnn.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>   // for strtod
#include <ctype.h>    // for isspace
#include "yzf_rng.h"

void genann_randomize_k(genann *ann) {
    yzf_rng_enable_clock();
    yzf_rng_enable();

    int i;
    for (i = 0; i < ann->total_weights; ++i) {
        double r = ((yzf_rng_get_random_number()%1000)/1000.0);   /* 宏定义应返回 [0,1) 的随机数 */
        ann->weight[i] = r - 0.5;
    }
}

genann *genann_init(int inputs, int hidden_layers, int hidden, int outputs)
{
    //Check
    if (hidden_layers < 0) return 0;
    if (inputs < 1) return 0;
    if (outputs < 1) return 0;
    if (hidden_layers > 0 && hidden < 1) return 0;

    /* 计算隐藏层权重总数：
     * - 第一隐藏层: (inputs + 1) * hidden   （+1 是偏置项）
     * - 后续隐藏层: (hidden + 1) * hidden * (hidden_layers - 1) */
    const int hidden_weights = hidden_layers ? (inputs+1) * hidden + (hidden_layers-1) * (hidden+1) * hidden : 0;
    /* 输出层权重总数:
     * - 如果有隐藏层: (hidden + 1) * outputs
     * - 如果无隐藏层: (inputs + 1) * outputs */
    const int output_weights = (hidden_layers ? (hidden+1) : (inputs+1)) * outputs;
    const int total_weights = (hidden_weights + output_weights);

    /* 总神经元个数 = 输入层 + 所有隐藏层神经元 + 输出层 */
    const int total_neurons = (inputs + hidden * hidden_layers + outputs);

    /* 需要额外分配的空间: 权重数组 + 输出值数组 + 误差项数组。
     * 输出值数组长度为 total_neurons，误差项数组长度为 (total_neurons - inputs)，
     * 因为输入层没有误差项。 */
    const int size = sizeof(genann) + sizeof(ANN_DATA_TYPE) * (total_weights + total_neurons + (total_neurons - inputs));

    //分配内存
    genann *ret = YZL_MEM_MANAGER.pMalloc(size);
    if (!ret) return 0;

    ret->inputs = inputs;
    ret->hidden_layers = hidden_layers;
    ret->hidden = hidden;
    ret->outputs = outputs;

    ret->total_weights = total_weights;
    ret->total_neurons = total_neurons;

    /* 设置指针指向分配的内存区域 */
    ret->weight = (ANN_DATA_TYPE*)((char*)ret + sizeof(genann));
    ret->output = ret->weight + ret->total_weights;
    ret->delta = ret->output + ret->total_neurons;

    /* 随机初始化权重 */
    genann_randomize_k(ret);

    return ret;
}

void genann_free(genann *ann) {
    /* 权重、输出值和误差项都在同一块连续内存中，直接 free 即可 */
    YZL_MEM_MANAGER.pFree(ann);
}

/* 前向传播: 根据输入 inputs 计算网络输出。
 * 返回一个指向输出层值数组的指针（该数组位于 ann->output 中的对应位置）。
 * 注意: 返回的指针在下次调用 genann_run 或 genann_train 后可能失效。 */
ANN_DATA_TYPE const *genann_run(genann const *ann, ANN_DATA_TYPE const *inputs) {
    ANN_DATA_TYPE const *w = ann->weight;          /* 当前权重指针 */
    ANN_DATA_TYPE *o = ann->output + ann->inputs;  /* 第一个隐藏层或输出层（无隐藏层时）的起始位置 */
    ANN_DATA_TYPE const *i = ann->output;          /* 输入层输出指针（初始指向输入值存储区） */

    /* 将输入值复制到 ann->output 的前 inputs 个位置，这样后续层可以统一处理 */
    memcpy(ann->output, inputs, sizeof(ANN_DATA_TYPE) * ann->inputs);

    int h, j, k;

    /* 如果没有隐藏层，直接计算输出层 */
    if (!ann->hidden_layers) {
        ANN_DATA_TYPE *ret = o;
        for (j = 0; j < ann->outputs; ++j) {
            ANN_DATA_TYPE sum = *w++ * -1.0;      /* 偏置项权重 * (-1) 模拟偏置输入为 -1 */
            for (k = 0; k < ann->inputs; ++k) {
                sum += *w++ * i[k];
            }
            *o++ = ann->activation_output((genann *)ann, sum);
        }
        return ret;
    }

    /* 计算第一隐藏层 */
    for (j = 0; j < ann->hidden; ++j) {
        ANN_DATA_TYPE sum = *w++ * -1.0;    //输入偏置
        for (k = 0; k < ann->inputs; ++k) {
            sum += *w++ * i[k];
        }
        *o++ = ann->activation_hidden((genann *)ann, sum);
    }

    i += ann->inputs;   /* 移动 i 到第一隐藏层输出位置，供下一层使用 */

    /* 计算后续隐藏层（如果有多层） */
    for (h = 1; h < ann->hidden_layers; ++h) {
        for (j = 0; j < ann->hidden; ++j) {
            ANN_DATA_TYPE sum = *w++ * -1.0; //输入
            for (k = 0; k < ann->hidden; ++k) {
                sum += *w++ * i[k];
            }
            *o++ = ann->activation_hidden((genann *)ann, sum);
        }
        i += ann->hidden;
    }

    ANN_DATA_TYPE const *ret = o;   /* 输出层结果的起始位置 */

    /* 计算输出层 */
    for (j = 0; j < ann->outputs; ++j) {
        ANN_DATA_TYPE sum = *w++ * -1.0;
        for (k = 0; k < ann->hidden; ++k) {
            sum += *w++ * i[k];
        }
        *o++ = ann->activation_output((genann *)ann, sum);
    }

    /* 确保所有权重都被使用，且所有输出都被写入 */
    assert(w - ann->weight == ann->total_weights);
    assert(o - ann->output == ann->total_neurons);

    return ret;
}


/* 训练网络: 使用反向传播算法更新权重。
 * ann:            神经网络
 * inputs:         输入向量
 * desired_outputs: 期望输出向量
 * learning_rate:  学习率
 */
void genann_train(genann const *ann, ANN_DATA_TYPE const *inputs, ANN_DATA_TYPE const *desired_outputs, ANN_DATA_TYPE learning_rate) {
    /* 先进行前向传播，计算各层输出 */
    genann_run(ann, inputs);

    int h, j, k;

    /* ========== 1. 计算输出层的误差项 (delta) ========== */
    {
        ANN_DATA_TYPE const *o = ann->output + ann->inputs + ann->hidden * ann->hidden_layers; /* 第一个输出神经元的值 */
        ANN_DATA_TYPE *d = ann->delta + ann->hidden * ann->hidden_layers;                     /* 第一个输出层的 delta */
        ANN_DATA_TYPE const *t = desired_outputs;                                             /* 第一个期望输出 */

        /* 根据激活函数类型计算误差项
         * 对于线性激活: delta = 期望 - 实际输出
         * 对于 sigmoid 等: delta = (期望 - 实际) * 实际 * (1 - 实际)  (即误差乘以导数) */
#if ACTIVATION_TYPE
            for (j = 0; j < ann->outputs; ++j) {
                *d++ = *t++ - *o++;
            }
#else
            for (j = 0; j < ann->outputs; ++j) {
                *d++ = (*t - *o) * *o * (1.0 - *o);
                ++o; ++t;
            }
#endif
    }

    /* ========== 2. 反向传播，计算隐藏层的误差项 ========== */
    /* 从最后一层隐藏层向前传播到第一层隐藏层 */
    for (h = ann->hidden_layers - 1; h >= 0; --h) {

        /* 当前层的第一个输出和第一个 delta 的位置 */
        ANN_DATA_TYPE const *o = ann->output + ann->inputs + (h * ann->hidden);
        ANN_DATA_TYPE *d = ann->delta + (h * ann->hidden);

        /* 下一层的第一个 delta 的位置 (可能是输出层或下一隐藏层) */
        ANN_DATA_TYPE const * const dd = ann->delta + ((h+1) * ann->hidden);

        /* 下一层的第一个权重的位置 */
        ANN_DATA_TYPE const * const ww = ann->weight + ((ann->inputs+1) * ann->hidden) + ((ann->hidden+1) * ann->hidden * (h));

        for (j = 0; j < ann->hidden; ++j) {
            ANN_DATA_TYPE delta = 0;

            /* 累加当前神经元对下一层所有神经元的影响 */
            for (k = 0; k < (h == ann->hidden_layers-1 ? ann->outputs : ann->hidden); ++k) {
                const ANN_DATA_TYPE forward_delta = dd[k];
                /* 下一层第 k 个神经元连接到当前层第 j 个神经元的权重索引
                 * 权重布局: 每个神经元有 (hidden+1) 个权重（包括偏置），
                 * 偏置权重在第一个位置，所以第 j 个神经元的权重偏移为 (j+1) */
                const int windex = k * (ann->hidden + 1) + (j + 1);
                const ANN_DATA_TYPE forward_weight = ww[windex];
                delta += forward_delta * forward_weight;
            }

            /* 当前神经元的 delta = 输出值 * (1-输出值) * 下一层反向传播的误差总和 */
            *d = *o * (1.0 - *o) * delta;
            ++d; ++o;
        }
    }

    /* ========== 3. 更新输出层的权重 ========== */
    {
        /* 输出层第一个 delta */
        ANN_DATA_TYPE const *d = ann->delta + ann->hidden * ann->hidden_layers;

        /* 输出层第一个权重指针 */
        ANN_DATA_TYPE *w = ann->weight + (ann->hidden_layers
                ? ((ann->inputs+1) * ann->hidden + (ann->hidden+1) * ann->hidden * (ann->hidden_layers-1))
                : (0));

        /* 输出层输入（即上一层的输出） */
        ANN_DATA_TYPE const * const i = ann->output + (ann->hidden_layers
                ? (ann->inputs + (ann->hidden) * (ann->hidden_layers-1))
                : 0);

        for (j = 0; j < ann->outputs; ++j) {
            /* 更新偏置权重 (输入为 -1) */
            *w++ += *d * learning_rate * -1.0;
            /* 更新连接权重 */
            for (k = 1; k < (ann->hidden_layers ? ann->hidden : ann->inputs) + 1; ++k) {
                *w++ += *d * learning_rate * i[k-1];
            }
            ++d;
        }

        assert(w - ann->weight == ann->total_weights);
    }

    /* ========== 4. 更新隐藏层的权重（从最后一层到第一层） ========== */
    for (h = ann->hidden_layers - 1; h >= 0; --h) {

        /* 当前层第一个 delta */
        ANN_DATA_TYPE const *d = ann->delta + (h * ann->hidden);

        /* 当前层的输入（即上一层的输出） */
        ANN_DATA_TYPE const *i = ann->output + (h
                ? (ann->inputs + ann->hidden * (h-1))
                : 0);

        /* 当前层的第一个权重指针 */
        ANN_DATA_TYPE *w = ann->weight + (h
                ? ((ann->inputs+1) * ann->hidden + (ann->hidden+1) * (ann->hidden) * (h-1))
                : 0);

        for (j = 0; j < ann->hidden; ++j) {
            /* 更新偏置权重 */
            *w++ += *d * learning_rate * -1.0;
            /* 更新连接权重 */
            for (k = 1; k < (h == 0 ? ann->inputs : ann->hidden) + 1; ++k) {
                *w++ += *d * learning_rate * i[k-1];
            }
            ++d;
        }
    }
}

void genann_init_sigmoid_lookup(genann *ann) {
    const double f = (ann->sigmod_max - ann->sigmod_min) / LOOKUP_SIZE;
    int i;

    /* interval = LOOKUP_SIZE / (sigmoid_dom_max - sigmoid_dom_min)
     * 用于将输入值快速转换为查找表索引 */
    ann->interval = LOOKUP_SIZE / (ann->sigmod_max -  ann->sigmod_min);
    for (i = 0; i < LOOKUP_SIZE; ++i) {
        ann->sigmod_sheet[i] = ann->activation_output(ann, ann->sigmod_min + f * i);
    }
}

/* 辅助函数：读取下一个非空白 token（连续非空白字符） */
static int read_token(FIL *f, char *buf, size_t bufsize) {
    char c;
    UINT br;
    size_t i = 0;

    /* 跳过空白字符 */
    do {
        if (f_read(f, &c, 1, &br) != FR_OK || br == 0) {
            return -1;   /* 文件错误或 EOF */
        }
    } while (c == ' ' || c == '\t' || c == '\n' || c == '\r');

    /* 读取 token */
    buf[i++] = c;
    while (i < bufsize - 1) {
        if (f_read(f, &c, 1, &br) != FR_OK || br == 0) {
            break;       /* 文件结束，但已有字符，仍可构成 token */
        }
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            break;
        }
        buf[i++] = c;
    }
    buf[i] = '\0';
    return (int)i;
}

/* 从文件中读取一个整数 */
static int read_int(FIL *f, int *val) {
    char token[32];
    if (read_token(f, token, sizeof(token)) <= 0) {
        return -1;
    }
    if (sscanf(token, "%d", val) != 1) {
        return -1;
    }
    return 0;
}

/* 从文件中读取一个双精度浮点数（支持科学计数法） */
static int read_double(FIL *f, double *val) {
    char token[64];
    if (read_token(f, token, sizeof(token)) <= 0) {
        return -1;
    }
    char *endptr;
    *val = strtod(token, &endptr);
    if (endptr == token) {
        return -1;   /* 转换失败 */
    }
    return 0;
}

/* 从 FatFs 文件中读取神经网络 */
genann *genann_read_fatfs(FIL *in) {
    int inputs, hidden_layers, hidden, outputs;

    /* 读取网络结构：四个整数 */
    if (read_int(in, &inputs) != 0 ||
        read_int(in, &hidden_layers) != 0 ||
        read_int(in, &hidden) != 0 ||
        read_int(in, &outputs) != 0) {
        return NULL;
    }

    /* 初始化神经网络 */
    genann *ann = genann_init(inputs, hidden_layers, hidden, outputs);
    if (!ann) {
        return NULL;
    }

    /* 读取所有权重 */
    for (int i = 0; i < ann->total_weights; ++i) {
        double w;
        if (read_double(in, &w) != 0) {
            genann_free(ann);
            return NULL;
        }
        ann->weight[i] = w;
    }

    return ann;
}

/* 将神经网络写入 FatFs 文件 */
void genann_write_fatfs(genann *ann, FIL *out) {
    UINT bw;
    char buf[256];

    /* 写入结构行（带换行符） */
    snprintf(buf, sizeof(buf), "%d %d %d %d\n",
             ann->inputs, ann->hidden_layers, ann->hidden, ann->outputs);
    f_write(out, buf, strlen(buf), &bw);

    /* 写入所有权重，每个权重占一行（科学计数法，保留 20 位小数） */
    for (int i = 0; i < ann->total_weights; ++i) {
        snprintf(buf, sizeof(buf), "%.20e\n", ann->weight[i]);
        f_write(out, buf, strlen(buf), &bw);
    }
}