//
// Created by 18744 on 2026/5/16.
//

#ifndef R320_CAMERASEEK_H
#define R320_CAMERASEEK_H

#include "yzx_base.h"
#include "MainUiView.hpp"

// ============ 重叠判断参数 ============
#define OVERLAP_THRESHOLD  0.5f    // IoU超过这个就算同一目标
#define MAX_DISTANCE       100.0f  // 中心点距离超过这个就算不同目标（即使IoU不够）
#define MISS_TIMEOUT       3       // 连续丢失3帧就移除目标

#define MAX_TARGETS 6

extern float camera_check_x;
extern float camera_check_y;
extern float camera_check_z;
extern float camera_check_r;
extern float camera_check_claw_R;
extern YZF_BOOL camera_check_second;
extern YZF_BOOL camera_finish;
extern ClawType claw_type;
extern int camera_see_point_z;
extern uint32_t tick_check;

extern "C" YZF_RESULT robot_main_camera_thread();

void bayes_camera_filter_sample(char *tag,float conf,float distance);

ClawType bayes_camera_filter_check();

//========================= 滤波代码 ==========================
// 6种真实类型
typedef enum {
    REAL_YUAN = 0,
    REAL_SI,
    REAL_M_TI,
    REAL_P_TI,
    REAL_M_SAN,
    REAL_P_SAN,
    REAL_TYPE_COUNT
} RealType;

// 摄像头可能返回的观测标签（你代码里strncmp判断的那些）
typedef enum {
    OBS_YUAN = 0,
    OBS_SI,
    OBS_TI,      // 不分材质的TI
    OBS_SAN,     // 不分材质的SAN
    OBS_MT,      // "mt" -> M_TI
    OBS_PT,      // "pt" -> P_TI
    OBS_MS,      // "ms" -> M_SAN
    OBS_PS,      // "ps" -> P_SAN
    OBS_COUNT
} ObsType;

// 滤波器结构体
typedef struct {
    float belief[REAL_TYPE_COUNT];  // 当前信念（后验概率）
    float dynamic_likelihood[OBS_COUNT][REAL_TYPE_COUNT]; // 动态似然，每次更新时重
} BayesFilter;
//
// // 初始化，设为均匀分布
// void bayes_init(BayesFilter *f);
//
// // 输入一次观测，更新信念
// // obs:        本次摄像头返回的观测类型
// // confidence: 摄像头给的置信度（0~1），暂时预留，后面可用来缩放似然
// void bayes_update(BayesFilter *f, ObsType obs, float confidence);
//
// // 获取当前最可能的类型（后验概率最大的）
// RealType bayes_get_best(const BayesFilter *f);
//
// // 获取指定类型的当前概率
// float bayes_get_prob(const BayesFilter *f, RealType type);
//
// // 重置为均匀分布（切换目标时调用）
// void bayes_reset(BayesFilter *f);

void bayes_init(BayesFilter *f);
void bayes_update(BayesFilter *f, ObsType obs, float confidence, float distance_to_center);
RealType bayes_get_best(const BayesFilter *f);
float bayes_get_prob(const BayesFilter *f, RealType type);
void bayes_reset(BayesFilter *f);

typedef struct {
    int id;                     // 唯一ID，用于追踪
    bool active;                // 是否活跃（当前帧有观测）

    // 位置信息（滤波后的）
    float x, y;                 // 左上角
    float w, h;                 // 宽高
    float center_x, center_y;   // 中心点

    // 贝叶斯滤波器（这个目标的类型概率）
    BayesFilter type_filter;

    // 统计
    int observe_count;          // 被观测到的次数
    int miss_count;             // 连续丢失次数
    float last_confidence;      // 最近一次置信度
} TrackedTarget;

typedef struct {
    TrackedTarget targets[MAX_TARGETS];
    int target_count;           // 当前活跃目标数
    int next_id;                // 下一个分配的ID
} ScreenState;

/**
 * 初始化 屏幕维护状态
 * @param state 屏幕状态结构体
 * @Checked
 */
void screen_init(ScreenState *state);

// ============ 核心更新接口 ============
// 输入一帧的CameraRequestLock，更新屏幕状态
// 返回本次更新后有多少个活跃目标
int screen_update(ScreenState *state, CameraRequestLock *lock);

// ============ 查询接口 ============
// 获取离屏幕中心最近的目标
TrackedTarget* screen_get_center_target(ScreenState *state, int screen_w, int screen_h);

// 获取指定ID的目标
TrackedTarget* screen_get_by_id(ScreenState *state, int id);

// 获取最可能的目标类型（综合概率最高的活跃目标）
TrackedTarget* screen_get_best_target(ScreenState *state);


#endif //R320_CAMERASEEK_H
