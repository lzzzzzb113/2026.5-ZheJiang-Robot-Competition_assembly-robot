//
// Created by 32827 on 2025/5/7.
//

#ifndef STM32H723_LTS_BSP_ARMGROUP_HPP
#define STM32H723_LTS_BSP_ARMGROUP_HPP

#include "yzl_space_sensor_arm_pole.hpp"
#include "yzl_space_sensor_arm_plate.hpp"
#include "yzl_space_sensor_arm_async_line.hpp"
extern int Status_CT;
extern int now_wheel;
/********机械臂限位以及其他数值********/
#define Length_Time 1000
#define arm_x_total 279   //mm
#define arm_y_total 185
#define arm_z_total 263
#define ARM_DC 185
#define ARM_X_2_MM 43
#define ARM_Y_2_MM 37
#define ARM_Z_2_MM 27
#define ARM_O_2_ANGLE 27
#define X_CONVEYOR_WITH_XMAX 245
#define Z_CLAW_WITH_CONVEYOR
#define Z_SENSOR_BEGIN 83
#define Z_BLOCK_SHELL_LIMIT 110
#define X_CONVEYOR 200
#define ARM_CLAW_MID 124
#define clawR_LengthToAngle 0.011f
#define DC4_LengthToAngle 0.011f
#define CLAW_CATCH  0.6f
#define CLAW_RELEASE (-0.80f)
#define CAMERA_PIXEL_X 640
#define CAMERA_PIXEL_Y 480
#define CAMERA_X_TO_CLAW 112
#define CAMERA_SEE_POINT_X (210*ARM_X_2_MM)
#define CAMERA_SEE_POINT_Y (20*ARM_Y_2_MM)
#define CAMERA_SEE_POINT_Z (225*ARM_Z_2_MM)
#define STAGE_MOVE_2_GRID (7976/2)
#define CAMERA_Y_OFFSET_SAN (camera_check_y+(112)*ARM_Y_2_MM)//112
#define THE_PUT_X_DEFAULT(offset) ((5+(offset))*ARM_X_2_MM)
#define THE_PUT_Y_DEFAULT(offset) ((154+(offset))*ARM_Y_2_MM)
#define THE_PUT_Z_DEFAULT(offset) ((42+(offset))*ARM_Z_2_MM)

typedef enum {
    SI = 0,
    SAN,
    TI,
    YUAN,
    P_TI,
    M_TI,
    P_SAN,
    M_SAN,
    NONE_CLAW,
    ERROR_CLAW
}ClawType;

typedef struct
{
    YZF_BOOL isEnable;
    uint32_t lastTime;
    void (*reset_listener)();
} ThreadWatchDog;

class ArmGroup {
public:

    SpaceSensorArmAsyncLine *Line_oz_Tim8,*Line_xy_Tim1;
    SpaceSensorArmPlate *arm_clawR;
    ThreadWatchDog *camera_watchdog;

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
