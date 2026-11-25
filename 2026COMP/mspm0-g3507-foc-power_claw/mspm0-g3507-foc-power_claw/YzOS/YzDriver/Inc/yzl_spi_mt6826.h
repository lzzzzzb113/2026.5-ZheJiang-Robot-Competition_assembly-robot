//
// Created by 32827 on 2026/2/22.
//

#ifndef MSPM0_YZL_SPI_MT6826_H
#define MSPM0_YZL_SPI_MT6826_H

#include "yzf.h"
#include "yz_spi.h"

#define MT6826S_READ_REG_MASK 0x300000
#define MT6826S_WRITE_REG_MASK 0x600000
#define MT6826S_EEPROM_MASK 0xC00000
#define MT6826S_READ_ANGLE_MASK  0xA00000

#define MT6826S_USER_ID_ADDRESS 0x001
#define MT6826S_ANGLE_HIGH_ADDRESS 0x003
#define MT6826S_ANGLE_LOW_ADDRESS 0x004
#define MT6826S_STATUS_ADDRESS 0x005
#define MT6826S_CRC_ADDRESS 0x006
#define MT6826S_ABZ_RES_HIGH_ADDRESS 0x007
#define MT6826S_ABZ_RES_LOW_ADDRESS 0x008
#define MT6826S_ZERO_POS_HIGH_ADDRESS 0x009
#define MT6826S_ZERO_POS_LOW_ADDRESS 0x00A
#define MT6826S_UVW_ADDRESS 0x00B
#define MT6826S_PWM_ADDRESS 0x00C
#define MT6826S_ANGLE_MOTION_ADDRESS 0x00D
#define MT6826S_AUTO_CAL_PARAM_ADDRESS 0x00E
#define MT6826S_BANDWIDTH_ADDRESS 0x011
#define MT6826S_POS_INIT_ADDRESS 0x0D8
#define MT6826S_AUTO_CAL_ADDRESS 0x155

#define MT6826S_ANGLE_STATUS_FAST_ERROR_BIT 0x1
#define MT6826S_ANGLE_STATUS_WEAK_ERROR_BIT 0x2
#define MT6826S_ANGLE_STATUS_INSUFFICIENT_ERROR_BIT 0x4

typedef struct
{
    uint8_t user_id;
    YzSPI spi;
    float angle;
    uint16_t ANGLE_DATA;
    uint8_t ANGLE_H;
    uint8_t ANGLE_L;
    uint8_t STATUS;
    uint8_t CRC;
} MT6826S;

YZF_RESULT yzl_mt6826_set_user_id(MT6826S *mt6826_s);

uint8_t yzl_mt6826_get_user_id(MT6826S *mt6826_s);

/**
 * @brief 未知bug废弃
 * @param angle_info 数据结构体
 * @return 成功返回 YZF_OK
 */
YZF_RESULT yzl_mt6826_get_angle_info(MT6826S *angle_info);

/**
 * @brief 计算公式 (角度值/2^15)*360
 * @return 返回角度值
 */
uint16_t yzl_mt6826_get_angle(MT6826S *mt6826_s);

YZF_BOOL yzl_mt6826_crc(MT6826S *mt6826_s);

#endif //MSPM0_YZL_SPI_MT6826_H