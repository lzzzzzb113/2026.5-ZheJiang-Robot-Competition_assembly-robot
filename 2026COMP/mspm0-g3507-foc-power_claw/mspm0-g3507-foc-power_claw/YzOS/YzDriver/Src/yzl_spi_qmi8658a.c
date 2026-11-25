//
// Created by 32827 on 2026/4/3.
//

#include "yzl_spi_qmi8658a.h"

#include "yz_log.h"

uint8_t yzl_qmi8658_conv_reg(QMI8658 *qmi8658,uint8_t reg,uint8_t data)
{
    // delay_us(5);
    yz_spi_full_lines_begin(&qmi8658->spi);
    delay_us(1);

    yz_spi_full_lines_conv8(&qmi8658->spi,reg);

    uint8_t re= yz_spi_full_lines_conv8(&qmi8658->spi,data);

    // delay_us(5);
    yz_spi_full_lines_end(&qmi8658->spi);
    delay_us(1);

    return re;
}

YZF_RESULT yzl_qmi8658_check_id(QMI8658 *qmi8658)
{
    uint8_t ADDRESS = QMI8658_WHO_AM_I | QMI8658_READ_REG_MASK;
    
    qmi8658->user_id = yzl_qmi8658_conv_reg(qmi8658,ADDRESS,0xFF);

    if(qmi8658->user_id == QMI8658_WHO_AM_I_DEFAULT)
    {
        return YZF_OK;
    }
    else
    {
        return YZF_ERROR;
    }
}

YZF_RESULT yzl_qmi8658_ctrl_pull(QMI8658 *qmi8658)
{
    
    qmi8658->CTRL1 = yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL1 | QMI8658_READ_REG_MASK,0xFF);
    
    qmi8658->CTRL2 = yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL2 | QMI8658_READ_REG_MASK,0xFF);
    
    qmi8658->CTRL3 = yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL3 | QMI8658_READ_REG_MASK,0xFF);
    
    qmi8658->CTRL5 = yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL5 | QMI8658_READ_REG_MASK,0xFF);
    
    qmi8658->CTRL7 = yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL7 | QMI8658_READ_REG_MASK,0xFF);
    
    qmi8658->CTRL8 = yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL8 | QMI8658_READ_REG_MASK,0xFF);
    return YZF_OK;
}
YZF_RESULT yzL_qmi8658_ctrl_push(QMI8658 *qmi8658)
{
    
    yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL1 | QMI8658_WRITE_REG_MASK,qmi8658->CTRL1);
    
    yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL2 | QMI8658_WRITE_REG_MASK,qmi8658->CTRL2);
    
    yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL3 | QMI8658_WRITE_REG_MASK,qmi8658->CTRL3);
    
    yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL5 | QMI8658_WRITE_REG_MASK,qmi8658->CTRL5);
    
    yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL7 | QMI8658_WRITE_REG_MASK,qmi8658->CTRL7);
    
    yzl_qmi8658_conv_reg(qmi8658,QMI8658_CTRL8 | QMI8658_WRITE_REG_MASK,qmi8658->CTRL8);
    return YZF_OK;
}

YZF_RESULT yzl_qmi8658_ctrl_print(QMI8658 *qmi8658)
{
    yz_printf("CTRL1: %d\n",qmi8658->CTRL1);
    yz_printf("CTRL2: %d\n",qmi8658->CTRL2);
    yz_printf("CTRL3: %d\n",qmi8658->CTRL3);
    yz_printf("CTRL5: %d\n",qmi8658->CTRL5);
    yz_printf("CTRL7: %d\n",qmi8658->CTRL7);
    yz_printf("CTRL8: %d\n",qmi8658->CTRL8);
    return YZF_OK;
}

YZF_RESULT yzl_qmi8658_update_data(QMI8658 *qmi8658)
{
    
    // qmi8658->temp = yzl_qmi8658_conv_reg(qmi8658,QMI8658_TEMP_L | QMI8658_READ_REG_MASK,0xFF);
    //
    // qmi8658->temp |= yzl_qmi8658_conv_reg(qmi8658,QMI8658_TEMP_H | QMI8658_READ_REG_MASK,0xFF) << 8;
    //
    // qmi8658->temp_f = ((float)(qmi8658->temp))/256.0f;
    //
    // qmi8658->ax = yzl_qmi8658_conv_reg(qmi8658,QMI8658_AX_L | QMI8658_READ_REG_MASK,0xFF);
    //
    // qmi8658->ax |= yzl_qmi8658_conv_reg(qmi8658,QMI8658_AX_H | QMI8658_READ_REG_MASK,0xFF) << 8;
    //
    // qmi8658->ax_f = ((float)((int16_t)qmi8658->ax))*qmi8658->acc_scale;
    //
    // qmi8658->ay = yzl_qmi8658_conv_reg(qmi8658,QMI8658_AY_L | QMI8658_READ_REG_MASK,0xFF);
    //
    // qmi8658->ay |= yzl_qmi8658_conv_reg(qmi8658,QMI8658_AY_H | QMI8658_READ_REG_MASK,0xFF) << 8;
    //
    // qmi8658->ay_f = ((float)((int16_t)qmi8658->ay))*qmi8658->acc_scale;
    //
    // qmi8658->az = yzl_qmi8658_conv_reg(qmi8658,QMI8658_AZ_L | QMI8658_READ_REG_MASK,0xFF);
    //
    // qmi8658->az |= yzl_qmi8658_conv_reg(qmi8658,QMI8658_AZ_H | QMI8658_READ_REG_MASK,0xFF) << 8;
    //
    // qmi8658->az_f = ((float)((int16_t)qmi8658->az))*qmi8658->acc_scale;
    //
    // qmi8658->gx = yzl_qmi8658_conv_reg(qmi8658,QMI8658_GX_L | QMI8658_READ_REG_MASK,0xFF);
    //
    // qmi8658->gx |= yzl_qmi8658_conv_reg(qmi8658,QMI8658_GX_H | QMI8658_READ_REG_MASK,0xFF) << 8;
    //
    // qmi8658->gx_f = ((float)((int16_t)qmi8658->gx))*qmi8658->gyro_scale;
    //
    // qmi8658->gy = yzl_qmi8658_conv_reg(qmi8658,QMI8658_GY_L | QMI8658_READ_REG_MASK,0xFF);
    //
    // qmi8658->gy |= yzl_qmi8658_conv_reg(qmi8658,QMI8658_GY_H | QMI8658_READ_REG_MASK,0xFF) << 8;
    //
    // qmi8658->gy_f = ((float)((int16_t)qmi8658->gy))*qmi8658->gyro_scale;

    qmi8658->gz = yzl_qmi8658_conv_reg(qmi8658,QMI8658_GZ_L | QMI8658_READ_REG_MASK,0xFF);

    qmi8658->gz |= yzl_qmi8658_conv_reg(qmi8658,QMI8658_GZ_H | QMI8658_READ_REG_MASK,0xFF) << 8;

    qmi8658->gz_f = ((float)((int16_t)qmi8658->gz))*qmi8658->gyro_scale;
    return YZF_OK;
}

YZF_RESULT yzl_qmi8658_print_data(QMI8658 *qmi8658)
{
    // yz_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",qmi8658->ax_f,qmi8658->ay_f,qmi8658->az_f,qmi8658->gx_f,qmi8658->gy_f,qmi8658->gz_f,qmi8658->temp_f);
    // yz_printf("%.3f\n",qmi8658->gz_f);
    return YZF_OK;
}

YZF_RESULT yzl_qmi8658_sync_conf(QMI8658 *qmi8658)
{
    if (qmi8658->AccIsOn)
    {
        qmi8658->CTRL7 |= 0x01;
        qmi8658->CTRL2 &= 0x80;
        qmi8658->CTRL2 |= qmi8658->accelerometer_speed_type | qmi8658->accelerometer_scale_type << 4;
        switch (qmi8658->accelerometer_scale_type)
        {
            case QMI8658_Accelerometer_2G:
                qmi8658->acc_scale = 2.0f/32768.0f;
                break;
            case QMI8658_Accelerometer_4G:
                qmi8658->acc_scale = 4.0f/32768.0f;
                break;
            case QMI8658_Accelerometer_8G:
                qmi8658->acc_scale = 8.0f/32768.0f;
                break;
            case QMI8658_Accelerometer_16G:
                qmi8658->acc_scale = 16.0f/32768.0f;
                break;
        }
    }else
    {
        qmi8658->CTRL7 &= 0xFE;
    }
    if (qmi8658->GyroIsOn)
    {
        qmi8658->CTRL7 |= 0x02;
        qmi8658->CTRL3 &= 0x80;
        qmi8658->CTRL3 |= qmi8658->gyroscope_speed_type | qmi8658->gyroscope_scale_type << 4;
        switch (qmi8658->gyroscope_scale_type)
        {
            case QMI8658_Gyroscope_16DPS:
                qmi8658->gyro_scale = 16.0f/32768.0f;
                break;
            case QMI8658_Gyroscope_32DPS:
                qmi8658->gyro_scale = 32.0f/32768.0f;
                break;
            case QMI8658_Gyroscope_64DPS:
                qmi8658->gyro_scale = 64.0f/32768.0f;
                break;
            case QMI8658_Gyroscope_128DPS:
                qmi8658->gyro_scale = 128.0f/32768.0f;
                break;
            case QMI8658_Gyroscope_256DPS:
                qmi8658->gyro_scale = 256.0f/32768.0f;
                break;
            case QMI8658_Gyroscope_512DPS:
                qmi8658->gyro_scale = 512.0f/32768.0f;
                break;
            case QMI8658_Gyroscope_1024DPS:
                qmi8658->gyro_scale = 1024.0f/32768.0f;
                break;
            case QMI8658_Gyroscope_2048DPS:
                qmi8658->gyro_scale = 2048.0f/32768.0f;
                break;
        }
    }else
    {
        qmi8658->CTRL7 &= 0xFD;
    }


    return YZF_OK;
}

YZF_RESULT yzl_qmi8658_reset(QMI8658 *qmi8658)
{
    
    uint8_t ADDRESS = QMI8658_RESET | QMI8658_WRITE_REG_MASK;
    uint8_t DATA = 0xB0;
    yzl_qmi8658_conv_reg(qmi8658,ADDRESS,DATA);
    return YZF_OK;
}

YZF_RESULT yzl_qmi8658_check_revision_id(QMI8658 *qmi8658)
{
    qmi8658->revision_id = yzl_qmi8658_conv_reg(qmi8658,QMI8658_REVISION_ID | QMI8658_READ_REG_MASK,0xFF);
    if ( qmi8658->revision_id == QMI8658_REVISION_ID_DEFAULT)
    {
        return YZF_OK;
    }else
    {
        return YZF_ERROR;
    }
}