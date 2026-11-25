//
// Created by 32827 on 2026/3/20.
//

#include "RobotCupProgram.hpp"

#include "yzl_space_sensor_camera.hpp"


// ===== 抓取线程内部缓存（必须放在这里，且用 static）=====
static int g_viewImage = 0;   // 1大圆 2小圆 3大方 4小方
static int g_offsetX   = 0;
static int g_offsetY   = 0;

// 大小阈值（需实测修改）
#define BIG_CIRCLE_W_THRESHOLD   200
#define BIG_SQUARE_W_THRESHOLD  200

CameraDetectResult *result_bk = NULL;
YZF_BOOL isValid = YZF_FALSE;
YZF_BOOL ReqStatus = YZF_FALSE;
YZF_BOOL ting();
CameraInstance *g_CameraInstance;

float Startbefore[7]={46.0f,35.0f,39.0f,133.0f,84.0f,0.0f,0.0f};//初始位置
float Viewposition[7]={46.0f,35.0f,39.0f,133.0f,84.0f,0.0f,0.0f};//合适图像识别角度位置
/***************根据图像识别圆柱体机械臂抓取流程**************/
float Y_passingarrive[7]={46.0f,86.0f,58.0f,122.0f,130.0f,0.0f,0.0f};//传送带物块到位
float Y_passingcatch[7]={46.0f,86.0f,58.0f,122.0f,130.0f,162.0f,0.0f};//传送带抓取物块
float Y_midthroughnop[7]={46.0f,13.0f,58.0f,122.0f,130.0f,162.0f,0.0f};//抓取后中间传递无问题转移
float Y_midthrougharrive[7]={165.0f,15.0f,58.0f,122.0f,130.0f,162.0f,0.0f};//抓取后中间传递无问题转移
float Y_midstagearrive[7]={165.0f,27.0f,88.5f,91.5f,133.6f,162.0f,0.0f};//内部物料台到位测试
float Y_midstagestore[7]={165.0f,32.0f,81.0f,99.0f,113.0f,60.0f,0.0f};//内部物料台存放测试
float Y_midstagecatch[7]={165.0f,32.0f,81.0f,99.0f,113.0f,162.0f,0.0f};//内部物料台存放测试
float Y_midstagecatch1[7]={163.0f,7.3f,46.0f,134.0f,114.0f,162.0f,0.0f};//内部物料台离位测试
float Y_midstageleave[7]={163.0f,7.3f,46.0f,134.0f,128.7f,162.0f,0.0f};//内部物料台离位测试
float Y_outerstagearrive[7]={163.0f,75.0f,74.0f,106.0f,128.7f,162.0f,0.0f};//外部物料台存放到位测试
float Y_outerstagearrive1[7]={163.0f,75.0f,80.0f,100.0f,128.7f,162.0f,0.0f};//外部物料台存放到位测试
float Y_outerstagefinish[7]={163.0f,67.7f,80.0f,100.0f,128.7f,60.0f,0.0f};//外部物料台存放爪开测试
float Y_outerstageleave[7]={163.0f,67.7f,65.0f,115.0f,114.0f,0.0f,0.0f};//外部物料台存放离场测试
/***************根据图像识别正方柱体机械臂抓取流程**************/
float Z_passingarrive[7]={45.0f,80.0f,69.0f,0.0f,126.0f,0.0f,0.0f};//传送带物块到位
float Z_passingcatch[7]={45.0f,80.0f,69.0f,0.0f,126.0f,100.0f,0.0f};//传送带抓取物块
float Z_midthroughnop[7]={45.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float Z_midthrougharrive[7]={103.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float Z_midstagearrive[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台到位测试
float Z_midstagestore[7]={103.0f,27.0f,119.0f,0.0f,114.0f,0.0f,0.0f};//内部物料台存放测试
float Z_midstageleave[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台离位测试
float Z_outerstagearrive[7]={163.0f,95.0f,110.0f,0.0f,136.0f,100.0f,0.0f};//外部物料台存放到位测试
float Z_outerstagefinish[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放爪开测试
float Z_outerstageleave[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放离场测试
/***************根据图像识别三角体机械臂抓取流程**************/
float S_passingarrive[7]={45.0f,80.0f,69.0f,0.0f,126.0f,0.0f,0.0f};//传送带物块到位
float S_passingcatch[7]={45.0f,80.0f,69.0f,0.0f,126.0f,100.0f,0.0f};//传送带抓取物块
float S_midthroughnop[7]={45.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float S_midthrougharrive[7]={103.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float S_midstagearrive[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台到位测试
float S_midstagestore[7]={103.0f,27.0f,119.0f,0.0f,114.0f,0.0f,0.0f};//内部物料台存放测试
float S_midstageleave[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台离位测试
float S_outerstagearrive[7]={163.0f,95.0f,110.0f,0.0f,136.0f,100.0f,0.0f};//外部物料台存放到位测试
float S_outerstagefinish[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放爪开测试
float S_outerstageleave[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放离场测试
/***************根据图像识别梯形柱体机械臂抓取流程**************/
float T_passingarrive[7]={45.0f,80.0f,69.0f,0.0f,126.0f,0.0f,0.0f};//传送带物块到位
float T_passingcatch[7]={45.0f,80.0f,69.0f,0.0f,126.0f,100.0f,0.0f};//传送带抓取物块
float T_midthroughnop[7]={45.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float T_midthrougharrive[7]={103.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float T_midstagearrive[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台到位测试
float T_midstagestore[7]={103.0f,27.0f,119.0f,0.0f,114.0f,0.0f,0.0f};//内部物料台存放测试
float T_midstageleave[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台离位测试
float T_outerstagearrive[7]={163.0f,95.0f,110.0f,0.0f,136.0f,100.0f,0.0f};//外部物料台存放到位测试
float T_outerstagefinish[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放爪开测试
float T_outerstageleave[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放离场测试
/***************根据图像识别三角柱体外壳机械臂抓取流程**************/
float SK_passingarrive[7]={45.0f,80.0f,69.0f,0.0f,126.0f,0.0f,0.0f};//传送带物块到位
float SK_passingcatch[7]={45.0f,80.0f,69.0f,0.0f,126.0f,100.0f,0.0f};//传送带抓取物块
float SK_midthroughnop[7]={45.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float SK_midthrougharrive[7]={103.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float Sk_midstagearrive[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台到位测试
float SK_midstagestore[7]={103.0f,27.0f,119.0f,0.0f,114.0f,0.0f,0.0f};//内部物料台存放测试
float SK_midstageleave[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台离位测试
float SK_outerstagearrive[7]={163.0f,95.0f,110.0f,0.0f,136.0f,100.0f,0.0f};//外部物料台存放到位测试
float SK_outerstagefinish[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放爪开测试
float SK_outerstageleave[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放离场测试
/***************根据图像识别提醒柱体外壳机械臂抓取流程**************/
float TK_passingarrive[7]={45.0f,80.0f,69.0f,0.0f,126.0f,0.0f,0.0f};//传送带物块到位
float TK_passingcatch[7]={45.0f,80.0f,69.0f,0.0f,126.0f,100.0f,0.0f};//传送带抓取物块
float TK_midthroughnop[7]={45.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float TK_midthrougharrive[7]={103.0f,70.0f,0.0f,0.0f,126.0f,100.0f,0.0f};//抓取后中间传递无问题转移
float TK_midstagearrive[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台到位测试
float TK_midstagestore[7]={103.0f,27.0f,119.0f,0.0f,114.0f,0.0f,0.0f};//内部物料台存放测试
float TK_midstageleave[7]={103.0f,27.0f,119.0f,0.0f,114.0f,100.0f,0.0f};//内部物料台离位测试
float TK_outerstagearrive[7]={163.0f,95.0f,110.0f,0.0f,136.0f,100.0f,0.0f};//外部物料台存放到位测试
float TK_outerstagefinish[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放爪开测试
float TK_outerstageleave[7]={163.0f,95.0f,110.0f,0.0f,136.0f,0.0f,0.0f};//外部物料台存放离场测试

int Status_zq=0;//机械臂状态基
int Status_View=0;//摄像头状态基（0：关闭；1:启用）
int ViewImage=0;//1:圆柱体 2:正方体 3:三角体 4:梯形柱体 5:三角柱体外壳 6：梯形柱体外壳
ArmPoseRef *apr1=NULL_PTR;

void ARMSTART(float armSTART[5],int time,int Status_amount,int Status_finish){
    if(Status_zq == Status_amount){
        if(apr1==NULL_PTR){
            apr1 = new ArmPoseRef(armSTART,armGroup);
        }
    apr1->SetData(armSTART);//设置机械臂目标角度（5个关节的角度）
    apr1->Restore(time);//转到抓取前的位置,运动时间=time
    Status_zq=Status_finish;
    }
}

void Viewbegin(){

}

void VIEWSTART(float viewSTART[5],int time,int Status_amount,int Status_finish){
    if(Status_zq == Status_amount&&apr1->IsRestore()){
    apr1->SetData(viewSTART);
    apr1->Restore(time);//转到抓取前的位置
    Status_zq=Status_finish;
    }
}

void MOVEOBJECT(float objectSTART[5],int time,int Status_amount,int Status_finish){
    if(Status_zq == Status_amount&&apr1->IsRestore()){
        apr1->SetData(objectSTART);
        apr1->Restore(time);//转到抓取前的位置
        Viewbegin();
        Status_zq=Status_finish;
    }
}

extern uint32_t SystemTickCNT;
// int Status_reset=0;//抓取
// void armMoveInit()
// {
//     if(Status_reset==0&&armGroup->arm07->isFinish()){
//         armGroup->arm08->addLength(2);
//         Status_reset++;
//     }
// }
// void armMove(char arm,int movelength,int Status_now,int Status_finish,int delaytime)//封装
// {
//     if(Status_reset==1&&armGroup->arm07->isFinish()&&armGroup->arm09->isFinish())
//     {
//         armGroup->arm08->addLength(2);
//         armGroup->arm01->setAngle(60);
//         Status_reset++;
//     }
// }

int Status_reset=0;//抓取
YZF_BOOL ting()//判断arm都停止
{
    if (armGroup->arm07->isFinish()&&armGroup->arm08->isFinish()&&armGroup->arm09->isFinish()&&armGroup->arm10->isFinish()&&armGroup->arm01->isActionFinish())return YZF_TRUE;
    else return YZF_FALSE;
}

extern "C" YZF_RESULT robot_main_program_thread()
{
 // ========== 状态 0：走到观察位姿 ==========
    if (Status_zq == 0) {
        ARMSTART(Viewposition, 1500, 0, 1);
    }
    // ========== 状态 1：触发摄像头请求 ==========
    else if (Status_zq == 1) {
        if (apr1->IsRestore()) {
            if (g_CameraInstance != NULL) {
                isValid = YZF_FALSE;
                ReqStatus = YZF_FALSE;
                result_bk = NULL;
                g_CameraInstance->RequestCamera('r');   // 请求圆形模型
                Status_zq = 2;
            }
        }
    }
    // ========== 状态 2：等待并处理识别结果（锁 + 判断）==========
    else if (Status_zq == 2) {
        // ---------- 锁检查 ----------
        if (isValid == YZF_TRUE) {
            if (ReqStatus == YZF_TRUE) {
                if (result_bk != NULL) {
                    int obj_w = result_bk->w;   // 框宽，用于区分大小
                    int obj_x = result_bk->x;
                    int obj_y = result_bk->y;

                    // ---------- 判断形状和大小 ----------
                    if (strstr(result_bk->tag, "yuan") != NULL) {
                        if (obj_w > BIG_CIRCLE_W_THRESHOLD) {
                            g_viewImage = 1;   // 大圆
                        } else {
                            g_viewImage = 2;   // 小圆
                        }
                    } else if (strstr(result_bk->tag, "zfx") != NULL) {
                        if (obj_w > BIG_SQUARE_W_THRESHOLD) {
                            g_viewImage = 3;   // 大方
                        } else {
                            g_viewImage = 4;   // 小方
                        }
                    } else {
                        g_viewImage = 0;       // 未识别
                    }

                    // 保存位置偏移
                    if (g_viewImage != 0) {
                        g_offsetX = obj_x - 640;
                        g_offsetY = obj_y - 360;
                        Status_zq = 3;         // 进入抓取
                    } else {
                        goto GET_RESULT_ERROR;
                    }
                } else {
                    goto GET_RESULT_ERROR;
                }
            } else {
GET_RESULT_ERROR:
                isValid = YZF_FALSE;
                ReqStatus = YZF_FALSE;
                result_bk = NULL;
                g_CameraInstance->RequestCamera('r');   // 请求圆形模型
                return YZF_HANDLER_NOT_FINISH;
            }
        }else
        {
            return YZF_HANDLER_NOT_FINISH;
        }
        // 数据未到，保持状态等待
    }
    // ========== 状态 3：执行抓取 ==========
    else if (Status_zq == 3) {
        float *arrivePose = NULL;
        switch (g_viewImage) {
            case 1: arrivePose = Y_passingarrive; break;   // 大圆
            case 2: arrivePose = Y_passingarrive; break;   // 小圆
            case 3: arrivePose = Z_passingarrive; break;   // 大方
            case 4: arrivePose = Z_passingarrive; break;   // 小方
        }
        if (arrivePose != NULL) {
            float compensated[7];
            memcpy(compensated, arrivePose, sizeof(compensated));
            compensated[0] += g_offsetX * 0.02f;   // 偏移补偿
            MOVEOBJECT(compensated, 1500, 3, 4);
        } else {
            goto GET_RESULT_ERROR;
        }
    }
    // ========== 状态 4：后续抓取流程 ==========
    else if (Status_zq == 4) {
        Status_zq = 99;
    }

    // ========== 终止 ==========
    if (Status_zq >= 99) {
        Status_zq=0;
        return YZF_OK;
    }
    return YZF_HANDLER_NOT_FINISH;
}


    /*if(Status_reset==0&& ting())
    {
        armGroup->arm09->addLength(0);
        Status_reset++;
    }else if(Status_reset==1&& ting())
    {
        armGroup->arm10->setLength(35);//顺时针转
        Status_reset++;
    }else if(Status_reset==2&& ting())
    {
        if (ReqStatus == YZF_FALSE)
        {
            armGroup->arm08->addLength(7);
        }else if (ReqStatus == YZF_TRUE)
        {
            if ()
                isValid= FALSE
        }

*/
extern "C" YZF_RESULT robot_reset_program_thread()
{
    if(Status_reset==0&& ting())
    {
        armGroup->arm10->addLength(90);//逆时针转
        Status_reset++;
    }else if(Status_reset==1&& ting())
    {
        armGroup->arm07->addLength(2);//-x轴
        Status_reset++;
    }else if(Status_reset==2&& ting())
    {
        armGroup->arm08->addLength(5);//y轴
        Status_reset++;
    }else if(Status_reset==3&& ting())
    {
        armGroup->arm09->addLength(14);//z轴
        Status_reset++;
    }else if (Status_reset==4&& ting())
    {
        armGroup->arm01->setAngle(56);//夹子
    }else if (Status_reset==5&& ting())
    {
        return YZF_OK;
    }
    return YZF_HANDLER_NOT_FINISH;
}
