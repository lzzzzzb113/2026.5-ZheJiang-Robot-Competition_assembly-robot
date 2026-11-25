//
// Created by 32827 on 2026/1/17.
//

#include "MainUiView.hpp"
#include "yzl_ui_thread.h"
#include "StageStauts.hpp"

uint8_t recv_temp[256];
CameraRequestLock request_lock;
MainView::MainView(YZL_VIEW* viewRam,const char *bgPath , ArmGroup* armGroup)
    : ViewBase(viewRam)
{
    this->backgroundPath = bgPath;
    this->armGroup = armGroup;
    this->actDataPtr = YZF_NULL_PTR;
    this->SelectIndex = 0;
    this->cameraInstance = new CameraInstance(yz_uart_receive_IT_RECV_get(USART1),&request_lock);
    this->cameraInstance->setStatusListener(this, CameraStatusListener);
    this->cameraInstance->setRequestListener(this, CameraRequestListener);
    this->dist_data = 0;
    this->dist_data_prev = 0;
}

MainView::~MainView() = default;

YZF_BOOL MainView::initP(void* initStructPointer, void* actStructPointer)
{
    ViewBase::initP(initStructPointer, actStructPointer);

    layout = new LayoutVertical(this);
    layout->setRange(0,0,YZ_SCREEN_WIDTH,YZ_SCREEN_HEIGHT,6);
    layout->setMargin(2);

    mText1 = new WidgetTextButton((*this)[-1],layout->nextRect(),0,
                          (utf8 *) "参数调试");
    addChild(mText1);
    mText1->setListener(Button1Listener);
    mText2=new WidgetTextButton((*this)[-1],layout->nextRect(),0,
                          (utf8 *) "抓取测试");
    addChild(mText2);
    mText2->setListener(Button2Listener);
    mText3=new WidgetTextButton((*this)[-1],layout->nextRect(),0,
                          (utf8 *) "机械复位");
    addChild(mText3);
    mText3->setListener(Button3Listener);
    mText4=new WidgetTextButton((*this)[-1],layout->nextRect(),0,
                          (utf8 *) "DISCONNECT");
    addChild(mText4);
    mText5=new WidgetTextButton((*this)[-1],layout->nextRect(),0,
                          (utf8 *) "请求测试");
    addChild(mText5);
    mText5->setListener(Button5Listener);
    mText6=new WidgetTextButton((*this)[-1],layout->nextRect(),0,
                          (utf8 *) "NONE");
    addChild(mText6);

    actDataPtr=(YZL_ACTIVE_DATA *)actStructPointer;

    return YZF_TRUE;
}

void MainView::wakeupP(void* actStructPointer)
{
    ViewBase::wakeupP(actStructPointer);

    yzl_ui_draw_565a((char *)this->backgroundPath,0,0,0,0,128,128);
    yzl_ui_refresh();

    mText1->instance->status=YZL_UI_CTR_EVENT_TOUCH_DOWN;
    mText1->instance->isRender=YZF_TRUE;

    if (view_params!=YZF_NULL_PTR)
    {
        view_params=YZF_NULL_PTR;
    }
    SelectIndex = 0;
}

void MainView::actP(void* actStructPointer)
{
    if(stage_act(yzf_gpio_get(GPIOB,YZF_PIN_7)==YZF_TRUE ? 1 : 0,armGroup->ProgramTick)){
        mText3->setText((utf8 *)map_position_stage_to_name(get_current_shape_ex()));
    }
    cameraInstance->act(); // 摄像头更新
    DistanceUpdate();
    ViewBase::actP(actStructPointer);
}

YZF_BOOL MainView::deInitP()
{
    ViewBase::deInitP();

    delete layout;

    return YZF_TRUE;
}

char* MainView::getViewName()
{
    return (char *)"MainView";
}

int MainView::getViewId()
{
    return 2026011701;
}

void MainView::resetBackground(UI_Rect* rect)
{
    yzl_ui_draw_565a((char *)this->backgroundPath,0,0,rect->x,rect->y,rect->width,rect->height);
}

void MainView::ButtonUp()
{
    if (view_params!=YZF_NULL_PTR)
    {
        view_params->ButtonUp();
    }else if(SelectIndex<5) {
        (*this)[SelectIndex]->status=YZL_UI_CTR_EVENT_TOUCH_UP;
        (*this)[SelectIndex]->isRender=YZF_TRUE;
        SelectIndex++;
        (*this)[SelectIndex]->status=YZL_UI_CTR_EVENT_TOUCH_DOWN;
        (*this)[SelectIndex]->isRender=YZF_TRUE;
    }
}

void MainView::ButtonDown()
{
    if (view_params!=YZF_NULL_PTR)
    {
        view_params->ButtonDown();
    }else if(SelectIndex>0) {
        (*this)[SelectIndex]->status=YZL_UI_CTR_EVENT_TOUCH_UP;
        (*this)[SelectIndex]->isRender=YZF_TRUE;
        SelectIndex--;
        (*this)[SelectIndex]->status=YZL_UI_CTR_EVENT_TOUCH_DOWN;
        (*this)[SelectIndex]->isRender=YZF_TRUE;
    }
}

void MainView::ButtonEnter()
{
    if (view_params!=YZF_NULL_PTR)
    {
        view_params->ButtonEnter();
    }else
    {
        actDataPtr->Event=YZL_UI_CTR_EVENT_SELECTED;
        actDataPtr->Data=&SelectIndex;
    }
}

void MainView::DistanceUpdate()
{
    YZF_RECV * recv = yz_uart_receive_IT_RECV_get(UART5);
    while (yzf_area_recv_get_size(recv)>0)
    {
        uint8_t dat = yzf_area_recv_get(recv);
        switch (distDecodeStatus)
        {
            case DIST_DECODE_STX:
                if (dat=='[')
                {
                    distDecodeStatus = DIST_DECODE_NE;
                    dist_data_prev=0;
                }
                break;
            case DIST_DECODE_NE:
                if (dat==']')
                {
                    dist_data = dist_data_prev;
                    mText5->setTextF((utf8 *)"%05d",(int)dist_data);
                    distDecodeStatus = DIST_DECODE_STX;
                }else if (dat >= '0' && dat <= '9')
                {
                    dist_data_prev= dist_data_prev*10+(dat - '0');
                }
                break;
            default:
                distDecodeStatus = DIST_DECODE_STX;
                break;
        }
    }
}

void MainView::ButtonEsc()
{
    //作为最外层控件不执行
    if (view_params!=YZF_NULL_PTR)
    {
        view_params->ButtonEsc();
    }
}

void MainView::Button1Listener(void* widget, YZL_UI_CTR_EVENT event)
{
    if(event==YZL_UI_CTR_EVENT_TOUCH_UP||event==YZL_UI_CTR_EVENT_SELECTED){
        WidgetTextButton *w=(WidgetTextButton *)widget;
        if(YZL_UI_WIDGET_PARENT_CPP_INSTANCE(MainView,w)->view_params==YZF_NULL_PTR){
            YZL_UI_ROOT *root=(YZL_UI_ROOT *) yzf_global_data_get(yz_status_sheet_get(YZ_SHEET_POINTER_FOR_UI_ROOT));
            YZL_UI_WIDGET_PARENT_CPP_INSTANCE(MainView,w)->view_params=new ViewParams(
                &YZL_NextTopView((*root))
                ,YZL_UI_WIDGET_PARENT_CPP_INSTANCE(MainView,w)->backgroundPath
                ,YZL_UI_WIDGET_PARENT_CPP_INSTANCE(MainView,w)->armGroup);
            yzf_global_handler_add(yzl_ui_push_view);
        }
    }
}

void MainView::Button2Listener(void *widget, YZL_UI_CTR_EVENT event)
{
    if(event==YZL_UI_CTR_EVENT_TOUCH_UP||event==YZL_UI_CTR_EVENT_SELECTED){
        yzf_global_handler_add(robot_main_program_thread);
    }
}
extern YZF_BOOL isRed;
void MainView::Button3Listener(void *widget, YZL_UI_CTR_EVENT event)
{
    if(event==YZL_UI_CTR_EVENT_TOUCH_UP||event==YZL_UI_CTR_EVENT_SELECTED){
        isRed=YZF_FALSE;
        yzf_global_handler_add(robot_main_program_thread);
        // WidgetTextButton *w=(WidgetTextButton *)widget;
        // YZL_UI_WIDGET_PARENT_CPP_INSTANCE(MainView,w)->cameraInstance->RequestCamera(0x46);
    }
}

void MainView::Button5Listener(void *widget, YZL_UI_CTR_EVENT event)
{
    if(event==YZL_UI_CTR_EVENT_TOUCH_UP||event==YZL_UI_CTR_EVENT_SELECTED){
        WidgetTextButton *w=(WidgetTextButton *)widget;
        YZL_UI_WIDGET_PARENT_CPP_INSTANCE(MainView,w)->cameraInstance->RequestCamera(0x53);
    }
}
YZF_RESULT CameraStatusListener(void *param, CameraStatus status){

    yz_info("[CAMERA] %s\n",(status==DISCONNECT)?(utf8 *)"DISCONNECT":(utf8 *)"CONNECT");

    ((MainView *)param)->mText4->setText(((status==DISCONNECT)?(utf8 *)"DISCONNECT":(utf8 *)"CONNECT"));
    return YZF_OK;
}
YZF_RESULT CameraRequestListener(void *viewMenu, CameraRequestStatus event,YZF_RECV_AREA *data){
    if(event==REQUEST_SUCCESS){
        memset(recv_temp,0,256);
        int len;
        len = yzf_area_recv_get_size(data);
        for (int i = 0; i < len; i++) {
            recv_temp[i]=yzf_area_recv_get(data);
        }
        recv_temp[len]='\0';
        request_lock.Result=((MainView *)viewMenu)->cameraInstance->CameraInfoDecode((char *)recv_temp,&request_lock.count);

        if (request_lock.Result==YZF_NULL_PTR) {
            request_lock.isSUCCUSS=YZF_FALSE;
            request_lock.isValid=YZF_TRUE;
            return YZF_OK;
        }

        request_lock.isSUCCUSS=YZF_TRUE;

        request_lock.Result->tag[19]='\0';

        yz_info("[CAMERA]Recv %s\n",recv_temp);

        ((MainView *)viewMenu)->mText6->setText((utf8 *) request_lock.Result->tag);
    }else{
        request_lock.isSUCCUSS=YZF_FALSE;
        ((MainView *)viewMenu)->mText6->setText((utf8 *) ("R_FAIL"));
    }
    request_lock.isValid=YZF_TRUE;
    return YZF_OK;
}