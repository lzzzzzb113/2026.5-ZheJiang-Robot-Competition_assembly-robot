//
// Created by 32827 on 2025/4/24.
//

#ifndef YZL_SPACE_SENSOR_ARM_PLATE_HPP
#define YZL_SPACE_SENSOR_ARM_PLATE_HPP
#include "yzl_space_sensor_arm.hpp"
#include "yzl_softport.h"

#ifdef YZSCREEN
#include "yzl_ui_widget_pp_text.hpp"
#include "yzl_ui_widget_pp_text_button.hpp"
#endif

//用于控件解算,围绕哪个旋转类型
typedef enum {
    RotateTypeX,
    RotateTypeY,
    RotateTypeZ
} SpaceSensorArmRotateType;

typedef struct {
    YZL_GPIO pwm_out;               //pwm输出引脚
    YZF_TIM_POINTER pwm_gen_tim;    //pwm生成器 TIM1,TIM2......
    uint16_t pwm_ch_id;             //pwm通道 TIM1_CH1,TIM1_CH2...... CH1 即为1
    uint32_t pwm_auto_reload;       //pwm通道 自动计数重载值

    int  pwm_compare_max;           //pwm通道 最大设置比较值
    int  pwm_compare_min;           //pwm通道 最小设置比较值
    int  pwm_compare_default;       //pwm通道 上电初始化默认比较值
    float maxAngle;                 //电机最大寻转角度

    SpaceSensorArmRotateType type;
} SpaceSensorArmPlatePortParam;

/**
 * 用于舵机控制输出
 * 注意目前不会自动初始化TIM，需要STM32CubeMX提前初始化
 * */
class SpaceSensorArmPlate : public SpaceSensorArm{
public:
    int maxLimit;
    int minLimit;
    int nowParam;
    int pwm_auto_reload;
    float maxAngle;
    YZF_TIM_POINTER tim;
    uint16_t ch_id;
    SpaceSensorArmRotateType rotateType;
#ifdef YZSCREEN
    YZF_BOOL hasWidget=YZF_FALSE;
    WidgetTextButton *widgetText;
#endif

    SpaceSensorArmPlate(SpaceSensorArm *parent,YZF_TIM_POINTER tim,uint16_t ch_id,Pose defaultPose,SpaceSensorArmRotateType type,int pwm_auto_reload,int maxParam,int minParam,float maxAngle,int defaultParam,YZF_BOOL isRoot);
    SpaceSensorArmPlate(SpaceSensorArmPlatePortParam *param,Pose defaultPose,SpaceSensorArm *parent,YZF_BOOL isRoot);

    float setAngle(float angle) override;
    float getAngle() override;
    /**
     * 姿态解算，未完成
     * @warning 不可使用
     * */
    Position getPosition(SpaceSensorArm *ArmEnd) override;

    void addAction(SpaceSensorAction *action) override;
    void EnableOut();
    void DisableOut();

#ifdef YZSCREEN
    void LinkTextUI(WidgetTextButton *pWidgetText);
    void CancelLink();
#endif
};
#endif //YZL_SPACE_SENSOR_ARM_PLATE_HPP
