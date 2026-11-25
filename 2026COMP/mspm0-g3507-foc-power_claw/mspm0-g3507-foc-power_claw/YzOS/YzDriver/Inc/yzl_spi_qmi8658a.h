//
// Created by 32827 on 2026/4/3.
//

#ifndef MSPM0_YZL_SPI_QMI8658A_H
#define MSPM0_YZL_SPI_QMI8658A_H

#include "yzf.h"
#include "yzl_softport.h"
#include "yz_spi.h"

#define QMI8658_READ_REG_MASK 0x80
#define QMI8658_WRITE_REG_MASK 0x00

#define QMI8658_WHO_AM_I 0x00
#define QMI8658_REVISION_ID 0x01
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL5 0x06
#define QMI8658_CTRL7 0x08
#define QMI8658_CTRL8 0x09
#define QMI8658_CTRL9 0x0A

#define QMI8658_CAL1_L 0x0B
#define QMI8658_CAL1_H 0x0C
#define QMI8658_CAL2_L 0x0D
#define QMI8658_CAL2_H 0x0E
#define QMI8658_CAL3_L 0x0F
#define QMI8658_CAL3_H 0x10
#define QMI8658_CAL4_L 0x11
#define QMI8658_CAL4_H 0x12

#define QMI8658_FIFO_WTM_TH 0x13
#define QMI8658_FIFO_CTRL 0x14
#define QMI8658_FIFO_SMPL_CNT 0x15
#define QMI8658_FIFO_STATUS 0x16
#define QMI8658_FIFO_DATA 0x17

#define QMI8658_STATUS_INT 0x2D
#define QMI8658_STATUS0 0x2E
#define QMI8658_STATUS1 0x2F

#define QMI8658_TIMESTAMP_LOW 0x30
#define QMI8658_TIMESTAMP_MID 0x31
#define QMI8658_TIMESTAMP_HIGH 0x32

#define QMI8658_TEMP_L 0x33
#define QMI8658_TEMP_H 0x34
#define QMI8658_AX_L 0x35
#define QMI8658_AX_H 0x36
#define QMI8658_AY_L 0x37
#define QMI8658_AY_H 0x38
#define QMI8658_AZ_L 0x39
#define QMI8658_AZ_H 0x3A
#define QMI8658_GX_L 0x3B
#define QMI8658_GX_H 0x3C
#define QMI8658_GY_L 0x3D
#define QMI8658_GY_H 0x3E
#define QMI8658_GZ_L 0x3F
#define QMI8658_GZ_H 0x40

#define QMI8658_COD_STATUS 0x46
#define QMI8658_dQW_L 0x49
#define QMI8658_dQW_H 0x4A
#define QMI8658_dQX_L 0x4B
#define QMI8658_dQX_H 0x4C
#define QMI8658_dQY_L 0x4D
#define QMI8658_dQY_H 0x4E
#define QMI8658_dQZ_L 0x4F
#define QMI8658_dQZ_H 0x50
#define QMI8658_dVX_L 0x51
#define QMI8658_dVX_H 0x52
#define QMI8658_dVY_L 0x53
#define QMI8658_dVY_H 0x54
#define QMI8658_dVZ_L 0x55
#define QMI8658_dVZ_H 0x56

#define QMI8658_TAP_STATUS 0x59
#define QMI8658_STEP_CNT_LOW 0x5A
#define QMI8658_STEP_CNT_MIDL 0x5B
#define QMI8658_STEP_CNT_HIGH 0x5C

#define QMI8658_RESET 0x60

#define QMI8658_WHO_AM_I_DEFAULT 0x05
#define QMI8658_REVISION_ID_DEFAULT 0x7C
#define QMI8658_CTRL1_DEFAULT 0x20

typedef enum
{
    QMI8658_Accelerometer_2G = 0,
    QMI8658_Accelerometer_4G = 1,
    QMI8658_Accelerometer_8G = 2,
    QMI8658_Accelerometer_16G = 3
} QMI8658AccelerometerScaleType;

typedef enum
{
    QMI8658_AccelerometerSpeed_N_1000Hz = 3,
    QMI8658_AccelerometerSpeed_N_500Hz = 4,
    QMI8658_AccelerometerSpeed_N_250Hz = 5,
    QMI8658_AccelerometerSpeed_N_125Hz = 6,
    QMI8658_AccelerometerSpeed_N_62_5Hz = 7,
    QMI8658_AccelerometerSpeed_N_31_25Hz = 8,
    QMI8658_AccelerometerSpeed_L_128Hz = 12,
    QMI8658_AccelerometerSpeed_L_21Hz = 13,
    QMI8658_AccelerometerSpeed_L_11Hz = 14,
    QMI8658_AccelerometerSpeed_L_3Hz = 15,
} QMI8658AccelerometerSpeedType;

typedef enum
{
    QMI8658_Gyroscope_16DPS = 0,
    QMI8658_Gyroscope_32DPS = 1,
    QMI8658_Gyroscope_64DPS = 2,
    QMI8658_Gyroscope_128DPS = 3,
    QMI8658_Gyroscope_256DPS = 4,
    QMI8658_Gyroscope_512DPS = 5,
    QMI8658_Gyroscope_1024DPS = 6,
    QMI8658_Gyroscope_2048DPS = 7
} QMI8658GyroscopeScaleType;

typedef enum
{
    QMI8658_GyroscopeSpeed_7174_4 = 0,
    QMI8658_GyroscopeSpeed_3587_2 = 1,
    QMI8658_GyroscopeSpeed_1793_6 = 2,
    QMI8658_GyroscopeSpeed_896_8 = 3,
    QMI8658_GyroscopeSpeed_448_4 = 4,
    QMI8658_GyroscopeSpeed_224_2 = 5,
    QMI8658_GyroscopeSpeed_112_1 = 6,
    QMI8658_GyroscopeSpeed_56_05 = 7,
    QMI8658_GyroscopeSpeed_28_025 = 8
} QMI8658GyroscopeSpeedType;

typedef struct
{
    YzSPI spi;
    YZF_BOOL AccIsOn;
    YZF_BOOL GyroIsOn;
    QMI8658AccelerometerScaleType accelerometer_scale_type;
    QMI8658AccelerometerSpeedType accelerometer_speed_type;
    float acc_scale;
    QMI8658GyroscopeScaleType gyroscope_scale_type;
    QMI8658GyroscopeSpeedType gyroscope_speed_type;
    float gyro_scale;

    uint8_t user_id;
    uint8_t revision_id;

    uint8_t CTRL1;
    uint8_t CTRL2;
    uint8_t CTRL3;
    uint8_t CTRL5;
    uint8_t CTRL7;
    uint8_t CTRL8;

    uint16_t temp;
    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;

    float temp_f;
    float ax_f;
    float ay_f;
    float az_f;
    float gx_f;
    float gy_f;
    float gz_f;
} QMI8658;

YZF_RESULT yzl_qmi8658_check_id(QMI8658 *qmi8658);

YZF_RESULT yzl_qmi8658_ctrl_pull(QMI8658 *qmi8658);

YZF_RESULT yzL_qmi8658_ctrl_push(QMI8658 *qmi8658);

YZF_RESULT yzl_qmi8658_ctrl_print(QMI8658 *qmi8658);

YZF_RESULT yzl_qmi8658_update_data(QMI8658 *qmi8658);

YZF_RESULT yzl_qmi8658_print_data(QMI8658 *qmi8658);

YZF_RESULT yzl_qmi8658_sync_conf(QMI8658 *qmi8658);

YZF_RESULT yzl_qmi8658_reset(QMI8658 *qmi8658);

YZF_RESULT yzl_qmi8658_check_revision_id(QMI8658 *qmi8658);

#endif //MSPM0_YZL_SPI_QMI8658A_H