//
// Created by 32827 on 2024/2/1.
//

#ifndef YZL_UI_VIEW_PP_BASE_HPP
#define YZL_UI_VIEW_PP_BASE_HPP

#include "yzl_ui.h"

#include "widget/yzl_ui_widget_pp_base.hpp"
#include "yzl_ui_pp_def.hpp"


/**
 * View 销毁链图
 *   / 兼容C层 /                                             / 兼容C层 /
 * View::deInit() -> View::~View()  -> View::deInitP() -> Widget::dispose() -> Widget::~Widget() -> Widget::disposeP()
 *                -> del C++相关实例  -> free C相关                           -> del C++相关实例    -> free C相关
 * 所以无需手动销毁 View C++ 实例 和 Widget C++ 实例
 * */
class ViewBase{
private:
    YZL_VIEW *instance;
public:
    /**
     *  @param viewRam 数据构建的内存，用于保存C态的基本数据结构
     * */
    ViewBase(YZL_VIEW *viewRam);
    ///C++控件实例会由View进行销毁
    virtual ~ViewBase();
    constexpr static int VIEW_WIDGET_MAX_COUNT=YZL_UI_VIEW_WIDGET_MAX_COUNT;//允许的最大控件数

    uint8_t *softIndex; //软索引，用于快速查找控件绘制顺序 softIndex[0]即为第一个绘制的控件的索引
    uint16_t idCount;   //控件ID计数，为添加的ID计数，用于生成控件ID，判别ID

    /**
     * 完成界面的初始化，作为基类对空间的内存进行了分配
     * 重写该方法者应以一个标准的初始化：
     *     ViewBase::initP(initStructPointer, actStructPointer); //调用父类的初始化
     *     ...
     *     return YZF_TRUE;
     *
     *     @param actStructPointer YZL_ACTIVE_DATA类型结构体，拥有界面组织和交互的所有信息
     *     @param initStructPointer 由界面编写者自行定义，用于传递完成初始化的数据
     * */
    virtual YZF_BOOL initP(void *initStructPointer,void *actStructPointer);

    /**
     * 在界面活跃时会被不断的调用，更新界面数据
     * 重写该方法者应以一个标准的初始化：
     *  ...
     *  ViewBase::actP(actStructPointer);
     *
     *  @param actStructPointer YZL_ACTIVE_DATA类型结构体，拥有界面组织和交互的所有信息
     * */

    virtual void actP(void *actStructPointer);
    /**
     * 多界面调用时，用于休眠的代码写在此处
     * 重写该方法者应以一个标准的初始化：
     *     ViewBase::sleepP(actStructPointer);
     *     ...
     *
     *     @param actStructPointer YZL_ACTIVE_DATA类型结构体，拥有界面组织和交互的所有信息
     * */
    virtual void sleepP(void *actStructPointer);
    /**
     * 多界面调用时，用于唤醒的操作写在这，作为基类完成了控件的重绘请求
     * 重写该方法者应以一个标准的初始化：
     *     ViewBase::wakeupP(actStructPointer);
     *     ...
     *
     *     @param actStructPointer YZL_ACTIVE_DATA类型结构体，拥有界面组织和交互的所有信息
     * */
    virtual void wakeupP(void *actStructPointer);

    /**
     * 绘制界面，作为基类完成了控件的重绘请求
     * 重写该方法者应以一个标准的初始化：
     *     ViewBase::drawP(drawStructPointer);
     *     ...
     *
     *     @param drawStructPointer YZL_DRAW_DATA类型结构体，拥有界面组织和交互的所有信息
     * */
    virtual void drawP(void *drawStructPointer);
    /** 部分背景重绘制 */
    virtual void resetBackground(UI_Rect *rect);

    /**
     * 界面的逆初始化，基本为完成内存的释放，以及状态恢复
     * */
    virtual YZF_BOOL deInitP();

    /**
     * 你的界面应该有一个名字
     * */
    virtual char *getViewName();                    //获取视图名称

    /**
     * 你的界面应该有一个ID
     * */
    virtual int getViewId();                        //获取视图ID

    /**
     * 添加子控件，支持C构建的控件以及C++构建控件
     * YZL_WIDGET为C构建的控件，WidgetBase为C++构建的控件
     * */
    YZF_BOOL addChild(YZL_WIDGET *child);
    YZF_BOOL addChild(WidgetBase *child);
    YZF_BOOL delChild(YZL_WIDGET *child);
    YZF_BOOL delChild(WidgetBase *child);

    /**
     * 更新控件的绘制顺序，用于解决控件的重叠绘制，请设置控件的轴
     * */
    void updateSoftIndex();

    /**
     * @param i 子控件索引，如果为负数，则从最后一个子控件开始查找
     * @return 返回子控件YZL_WIDGET *
     * */
    YZL_WIDGET* operator [](int i);

    /**
     * 以下函数完成从C到C++的函数调用转换，请修改对应的xxxP函数
     * */
    static YZF_BOOL init(void *initStructPointer,void *actStructPointer); //初始化
    static void act(void *actStructPointer);              //活动
    static void sleep(void *actStructPointer);            //休眠
    static void wakeup(void *actStructPointer);           //唤醒
    ///@param drawStructPointer 保留参数
    static void draw(void *actStructPointer);             //绘制
    ///C++控件实例会由View进行销毁
    static YZF_BOOL deInit(void *actStructPointer);       //销毁
};
#endif //YZL_UI_VIEW_PP_BASE_HPP
