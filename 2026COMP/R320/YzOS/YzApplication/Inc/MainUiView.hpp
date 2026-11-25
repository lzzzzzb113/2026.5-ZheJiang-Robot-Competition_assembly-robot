//
// Created by 32827 on 2026/1/17.
//

#ifndef STM32H723_LTS_BSP_MAINUIVIEW_H
#define STM32H723_LTS_BSP_MAINUIVIEW_H

#include "yz_uart.h"

#include "ArmGroup.hpp"
#include "yzl_ui_pp_layout_base_ver.hpp"
#include "yzl_space_sensor_camera.hpp"

#include "view/yzl_ui_view_pp_base.hpp"
#include "widget/yzl_ui_widget_pp_text_button.hpp"

#include "ParamsUiView.hpp"
#include "RobotCupProgram.hpp"

extern "C" YZF_RESULT CameraStatusListener(void *param, CameraStatus status);
extern "C" YZF_RESULT CameraRequestListener(void *viewMenu, CameraRequestStatus event,YZF_RECV_AREA *data);

typedef enum
{
    DIST_DECODE_STX=0,
    DIST_DECODE_NE
} DistanceDecodeStatus;

class MainView : ViewBase
{
public:
    /*** 参数定义 ***/
    const char *backgroundPath;
    ArmGroup *armGroup;
    CameraInstance *cameraInstance;
    YZL_ACTIVE_DATA *actDataPtr;
    uint16_t dist_data;
    uint16_t dist_data_prev;
    DistanceDecodeStatus distDecodeStatus;

    /*** 控件定义 ***/
    WidgetTextButton *mText1,*mText2,*mText3,*mText4,*mText5,*mText6;
    LayoutVertical *layout;

    /*** 子界面定义 ***/
    ViewParams *view_params=YZF_NULL_PTR;

    MainView(YZL_VIEW *viewRam, const char *bgPath , ArmGroup *armGroup);
    ~MainView() override;

    /*** 重写 ***/
    YZF_BOOL initP(void *initStructPointer, void *actStructPointer) override;
    void wakeupP(void *actStructPointer) override;
    void actP(void *actStructPointer) override;
    YZF_BOOL deInitP() override;

    char* getViewName() override;
    int getViewId() override;
    void resetBackground(UI_Rect *rect) override;

    /*** 控件逻辑 ***/
    int SelectIndex;
    void ButtonUp();
    void ButtonDown();
    void ButtonEnter();
    void ButtonEsc();

    void DistanceUpdate();

    static void Button1Listener(void *widget, YZL_UI_CTR_EVENT event);
    static void Button2Listener(void *widget, YZL_UI_CTR_EVENT event);
    static void Button3Listener(void *widget, YZL_UI_CTR_EVENT event);
    static void Button5Listener(void *widget, YZL_UI_CTR_EVENT event);
};
#endif //STM32H723_LTS_BSP_MAINUIVIEW_H