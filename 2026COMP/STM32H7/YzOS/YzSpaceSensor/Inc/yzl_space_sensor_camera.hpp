//
// Created by 32827 on 2025/5/11.
//

#ifndef STM32H723_LTS_BSP_YZL_SPACE_SENSOR_CAMERA_HPP
#define STM32H723_LTS_BSP_YZL_SPACE_SENSOR_CAMERA_HPP

#include "yzx_base.h"
#include "yzl.h"

#define ASCII_SYN 0x16
#define ASCII_ACK 0x06

#define ASCII_SOH 0x01 //标题开始
#define ASCII_STX 0x02 //正文开始
#define ASCII_ETX 0x03 //正文结束
#define ASCII_EOT 0x04 //传输结束

typedef enum{
    DISCONNECT,
    CONNECT_CHECKING,
    CONNECTING,
} CameraStatus;

typedef enum{
    REQUEST_SUCCESS,
    REQUEST_FAIL,
    REQUEST_TIMEOUT,
    REQUEST_IDLE,
    REQUEST_WAITING
} CameraRequestStatus;

typedef enum{
    RECEIVE_WAIT_START,
    RECEIVE_START,
    RECEIVE_END
} CameraReceiveStatus;

typedef struct {
    int x;
    int y;
    int w;
    int h;
    char tag[20];
    float confidence;
} CameraDetectResult;

typedef enum {
    DECODE_IDLE,
    DECODE_SUCCESS,
    DECODE_FAIL,
    DECODE_LEVEL_1,
    DECODE_LEVEL_2,
} CameraInfoDecodeStatus;


//TODO: 修改ArmGroup作为时钟参考
/**
 * 下位机发送
 *    请求格式: [0x02,ReqCode,0x03]
 *    同步请求格式: [0x16]
 * 上位机回复
 *    请求回复格式: [0x02, '[[x,y,w,h,tag:confidence][...]]', 0x03]
 *    同步回复格式: [0x06]
 *
 *    上位机 使用串口链接 com5 115200  n 1，完成上位机代买
 */
class CameraInstance{
public:
    CameraDetectResult result[10];
    char decodeBuf[100];
    int decodeBufIndex;

    CameraStatus cameraStatus;
    uint32_t CheckStartTick;
    void *ListenerOwner;
    YZF_RESULT (*CameraStatusListener)(void *viewMenu, CameraStatus event);

    CameraRequestStatus cameraRequestStatus;
    CameraReceiveStatus cameraReceiveStatus;
    uint32_t RequestStartTick;
    void *ListenerRequestOwner;
    YZF_RESULT (*CameraRequestListener)(void *viewMenu, CameraRequestStatus event,YZF_RECV_AREA *data);

    YZF_RECV_AREA *RecArea;
    YZF_RECV_AREA *BufArea;

    CameraInstance(YZF_RECV_AREA *recArea);
    ~CameraInstance();
    void act();
    void UpdateCamera();
    /**
     *
     * @param ListenerOwner 可以传入C++实体
     * @param pCameraStatusListener 摄像头连接状态更新监听器
     */
    void setStatusListener(void *ListenerOwner,YZF_RESULT (*pCameraStatusListener)(void *viewMenu, CameraStatus event));
    /**
     *
     * @param ListenerOwner 可以传入C++实体
     * @param pCameraRequestListener 摄像头请求状态更新监听器
     */
    void setRequestListener(void *ListenerOwner,YZF_RESULT (*pCameraRequestListener)(void *viewMenu, CameraRequestStatus event,YZF_RECV_AREA *data));
    void RequestCamera(int ReqCode);
    CameraDetectResult * CameraInfoDecode(char *data,int *count);

    static uint32_t CameraTick;
    static YZF_RESULT CameraTickHandler(uint32_t tick);
};

#endif //STM32H723_LTS_BSP_YZL_SPACE_SENSOR_CAMERA_HPP
