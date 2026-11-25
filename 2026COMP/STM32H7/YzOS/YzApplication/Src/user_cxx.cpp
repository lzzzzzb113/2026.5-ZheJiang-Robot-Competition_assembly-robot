//
// Created by 32827 on 2025/3/22.
//
#include "user_cxx.h"
#include "yz_tim.h"
#include "yz_heart.h"
#include "yzl_transform_segment.h"

#include "MainUiView.hpp"
#include "ArmGroup.hpp"

YZL_UI_ROOT *uiRoot;

MainView *mainMenu;
ArmGroup *armGroup;
const char *bgImg="/MMC/test.565";

uint32_t SystemTickCNT=0;//可以当成全局时钟，每ms +1
//在时钟心跳轮询队列被调用
YZF_RESULT SystemTickCount(uint32_t tick){
    SystemTickCNT += tick;
    return YZF_HANDLER_NOT_FINISH;
}

YZF_RESULT yzl_program_logic_loop();
/**
 * UI 主界面配置初始化
 * */
YZF_RESULT yzl_ui_main_view_boot(){
    //从全局表中取UI_ROOT的地址
    uiRoot=(YZL_UI_ROOT *) yzf_global_data_get(yz_status_sheet_get(YZ_SHEET_POINTER_FOR_UI_ROOT));

    //初始化机械臂
    armGroup = new ArmGroup();
    armGroup->Init();
    armGroup->OutEnable();

    //在UI_ROOT中创建首层View,ViewMenu以显示
    uiRoot->ViewTop++;
    mainMenu=new MainView(&YZL_TopView((*uiRoot)),bgImg,armGroup);

    //在全局表中配置首层View地址
    YZF_GLOBAL_INDEX index=yzf_global_data_add(&YZL_TopView((*uiRoot)));
    yz_status_sheet_set(YZ_SHEET_POINTER_FOR_UI_VIEW,index);

    //在全局表中配置初始化数据指针
    YZF_GLOBAL_INDEX index2=yzf_global_data_add(YZF_NULL_PTR);
    yz_status_sheet_set(YZ_SHEET_POINTER_FOR_UI_VIEW_INIT_DATA,index2);

    //添加全局时钟
    yz_heart_process_add(SystemTickCount);
    return YZF_OK;
}
/**
 * 界面初始化后会被调用请保持
 * 代码应以以下格式:
 *  ...
 *  yzf_global_handler_add(yzl_program_logic_loop);
 *  return YZF_OK;
 * */
extern "C" YZF_RESULT yzl_ui_init_done(){
    yzf_global_handler_add(yzl_program_logic_loop); //添加yzl_program_logic_loop函数到轮询队列
    return YZF_OK;
}
/**
 * 此函数应不断循环，所以请
 * return YZF_HANDLER_NOT_FINISH;此函数未完成
 * return YZF_OK;会结束当前函数的轮询
 * */
YZF_RESULT yzl_program_logic_loop(){
    armGroup->act();
    // mainMenu->cameraInstance->act();
    return YZF_HANDLER_NOT_FINISH;
}
/**
 * 完成按键的逻辑输入到界面事件的转换
 * */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin==GPIO_PIN_4){
        mainMenu->ButtonUp();       //向上按钮
    }else if(GPIO_Pin==GPIO_PIN_3){
        if(yzf_gpio_get(GPIOE,YZF_PIN_4)){
            mainMenu->ButtonEsc();  //退出按钮
            return;
        }
        mainMenu->ButtonDown();     //向下按钮
    }else if(GPIO_Pin==GPIO_PIN_2){
        mainMenu->ButtonEnter();    //确认按钮
    }
}