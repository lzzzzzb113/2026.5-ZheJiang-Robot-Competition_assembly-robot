//
// Created by 32827 on 2026/4/3.
//

#include "yz_spi.h"


YZF_RESULT yz_spi_full_lines_begin(YzSPI *spi){
    YLGPIO(spi->define.cs)
    yzf_gpio_set_0_();
    return YZF_OK;
}

YZF_RESULT yz_spi_full_lines_end(YzSPI *spi){
    YLGPIO(spi->define.cs)
    yzf_gpio_set_1_();
    return YZF_OK;
}

uint8_t yz_spi_full_lines_conv8(YzSPI* spi,uint8_t data){
    if (spi->isHard)
    {
        DL_SPI_transmitDataBlocking8(spi->hard_spi,data);
        data = DL_SPI_receiveDataBlocking8(spi->hard_spi);
    }else
    {
        data = yzl_softport_spi_transmit_byte8(&spi->define,data);
    }
    return data;
}