//
// Created by 32827 on 2025/5/7.
//

#ifndef STM32H723_LTS_BSP_ARMGROUP_HPP
#define STM32H723_LTS_BSP_ARMGROUP_HPP

#include "yzl_space_sensor_arm_pole.hpp"
#include "yzl_space_sensor_arm_plate.hpp"

class ArmGroup {
public:

    SpaceSensorArmPlate *arm01,*arm02,*arm03,*arm04,*arm05,*arm06;//6个舵机的位置
    SpaceSensorArmPole *arm07,*arm08,*arm09,*arm10;//X,Y轴参数

    ArmGroup();
    ~ArmGroup();

    /**
     * 初始化函数，需要被调用
     */
    void Init();
    /**
     *  简单的各位置调整
     *  @link ViewParams.actP 用于调试参数
     *
     *  @param index 机械臂编号
     *  @param timeUnit 实现时间
     */
    void addParams(int index,float timeUnit);
    void delParams(int index,float timeUnit);
    /**
     * 机械臂位置更新函数
     */
    void act();
    /**
     * 末端位置解算，过于复杂未实现
     */
    void getEnd();
    /**
     * 关键角度计算，未实现
     * @return 机械臂的参数表
     */
    float * compile();
    /**
     * 开启输出
     */
    void OutEnable();
    /**
     * 关闭输出
     */
    void OutDisable();

    /**
     * 作为机械臂的时间基准，ProgramTickHandler会被系统时钟基准更新函数调用
     *
     * @tick 每次调用的时间间隔
     */
    static uint32_t ProgramTick;
    static YZF_BOOL isLoadTickHandler;
    static YZF_RESULT ProgramTickHandler(uint32_t tick);
};
#endif //STM32H723_LTS_BSP_ARMGROUP_HPP
