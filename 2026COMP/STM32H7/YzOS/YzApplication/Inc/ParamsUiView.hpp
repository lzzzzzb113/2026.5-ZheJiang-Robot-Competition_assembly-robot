//
// Created by 32827 on 2025/5/7.
//

#ifndef STM32H723_LTS_BSP_UI_VIEW_PARAMS_HPP
#define STM32H723_LTS_BSP_UI_VIEW_PARAMS_HPP

#include "view/yzl_ui_view_pp_base.hpp"
#include "widget/yzl_ui_widget_pp_text_button.hpp"
#include "yzl_ui_pp_layout_base_ver.hpp"
#include "ArmGroup.hpp"
#include "yzl_softport.h"


/**
 * 保留用于调试机械臂参数界面
 */
class ViewParams : public ViewBase {
public:
    /*** 控件定义 ***/
    WidgetTextButton *mText1,*mText2,*mText3,*mText4,*mText5,*mText6;
    LayoutVertical *layout;

    /*** 变量定义 ***/
    const char *bgPath;
    YZL_ACTIVE_DATA *actDataPtr;
    ArmGroup *armGroup;

    /*** 控件逻辑变量 ***/
    uint32_t MoveDir=0;
    uint8_t ButtonSelectType = 0;
    YZF_BOOL ButtonIsDown = YZF_FALSE;
    uint32_t lastActTime=0;


    static YZL_GPIO ButtonUpPin;
    static YZL_GPIO ButtonDownPin;

    ViewParams(YZL_VIEW *viewRam,const char *bgPath,ArmGroup *armGroup);

    ~ViewParams();

    YZF_BOOL initP(void *initStructPointer, void *actStructPointer) override;

    YZF_BOOL deInitP() override;

    void wakeupP(void *actStructPointer) override;

    void sleepP(void *actStructPointer) override;

    char *getViewName() override;

    int getViewId() override;

    void resetBackground(UI_Rect *rect) override;

    void actP(void *actStructPointer) override;

    int SelectIndex;
    void ButtonUp();
    void ButtonDown();
    void ButtonEnter();
    void ButtonEsc();
};
#endif //STM32H723_LTS_BSP_UI_VIEW_PARAMS_HPP
