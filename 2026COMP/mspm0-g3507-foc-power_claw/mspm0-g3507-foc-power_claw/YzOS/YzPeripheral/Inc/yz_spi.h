//
// Created by 32827 on 2026/4/3.
//

#ifndef MSPM0_YZ_SPI_H
#define MSPM0_YZ_SPI_H

#include "yzf.h"
#include "yzl_softport_spi.h"

typedef SPI_Regs* YZF_SPI_POINTER;

typedef struct
{
    YZF_BOOL isHard;
    YZF_SPI_POINTER hard_spi;
    YZL_Soft_SPI define;
} YzSPI;

YZF_RESULT yz_spi_full_lines_begin(YzSPI *spi);

YZF_RESULT yz_spi_full_lines_end(YzSPI *spi);

uint8_t yz_spi_full_lines_conv8(YzSPI* spi,uint8_t data);


#endif //MSPM0_YZ_SPI_H