//
// Created by 32827 on 2026/2/22.
//

#include "yzl_spi_mt6826.h"

YZF_RESULT yzl_mt6826_set_user_id(MT6826S *mt6826_s)
{
    uint32_t data_temp = MT6826S_WRITE_REG_MASK;
    data_temp |= MT6826S_USER_ID_ADDRESS << 8;
    data_temp |= mt6826_s->user_id;

    yz_spi_full_lines_begin(&mt6826_s->spi);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>16&0xFF);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>8&0xFF);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp&0xFF);

    yz_spi_full_lines_end(&mt6826_s->spi);

    return YZF_OK;
}

uint8_t yzl_mt6826_get_user_id(MT6826S *mt6826_s)
{
    uint32_t data_temp = MT6826S_READ_REG_MASK;
    data_temp |= MT6826S_USER_ID_ADDRESS << 8;

    yz_spi_full_lines_begin(&mt6826_s->spi);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>16&0xFF);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>8&0xFF);

    data_temp = yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp&0xFF);

    yz_spi_full_lines_end(&mt6826_s->spi);

    return data_temp;
}

YZF_RESULT yzl_mt6826_get_angle_info(MT6826S *mt6826_s)
{
    uint32_t data_temp = MT6826S_READ_ANGLE_MASK;
    data_temp |= MT6826S_ANGLE_HIGH_ADDRESS << 8;

    yz_spi_full_lines_begin(&mt6826_s->spi);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>16&0xFF);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>8&0xFF);

    data_temp = yz_spi_full_lines_conv8(&mt6826_s->spi,0xFF);

    mt6826_s->ANGLE_DATA = data_temp<<7;

    data_temp = yz_spi_full_lines_conv8(&mt6826_s->spi,0xFF);

    mt6826_s->ANGLE_DATA |= data_temp>>1;

    mt6826_s->STATUS = yz_spi_full_lines_conv8(&mt6826_s->spi,0xFF);

    mt6826_s->CRC = yz_spi_full_lines_conv8(&mt6826_s->spi,0xFF);

    yz_spi_full_lines_end(&mt6826_s->spi);

    return YZF_OK;
}

YZF_BOOL yzl_mt6826_crc(MT6826S *mt6826_s) {
    const uint8_t poly = 0x07;
    uint8_t crc = 0x00;                // 初始值
    uint8_t data[3] = {mt6826_s->ANGLE_H, mt6826_s->ANGLE_L, mt6826_s->STATUS};    // 0x003, 0x004, 0x005 的顺序
    for (uint8_t i = 0; i < 3; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc==mt6826_s->CRC;   // 最终 CRC 值
}

uint16_t yzl_mt6826_get_angle(MT6826S *mt6826_s)
{
    uint32_t data_temp = MT6826S_READ_REG_MASK;
    data_temp |= MT6826S_ANGLE_HIGH_ADDRESS << 8;

    yz_spi_full_lines_begin(&mt6826_s->spi);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>16&0xFF);

    yz_spi_full_lines_conv8(&mt6826_s->spi,data_temp>>8&0xFF);

    mt6826_s->ANGLE_H = yz_spi_full_lines_conv8(&mt6826_s->spi, data_temp & 0xFF);
    mt6826_s->ANGLE_L = yz_spi_full_lines_conv8(&mt6826_s->spi, data_temp & 0xFF);
    mt6826_s->ANGLE_DATA = mt6826_s->ANGLE_H << 7 | mt6826_s->ANGLE_L >> 1;
    mt6826_s->STATUS = yz_spi_full_lines_conv8(&mt6826_s->spi,0xFF);
    mt6826_s->CRC = yz_spi_full_lines_conv8(&mt6826_s->spi,0xFF);
    yz_spi_full_lines_end(&mt6826_s->spi);

    return mt6826_s->ANGLE_DATA;
}