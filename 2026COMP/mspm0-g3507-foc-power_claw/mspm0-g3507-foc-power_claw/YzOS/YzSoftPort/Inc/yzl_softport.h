//
// Created by 32827 on 2024/1/10.
//

#ifndef YZL_SOFTPORT_H
#define YZL_SOFTPORT_H
#include "yzl.h"
#if USE_SOFT_PORT

#include "yzf_gpio.h"
typedef struct{
    YZF_GPIO_PIN pin;
    YZF_GPIO_POINTER port;
}YZL_GPIO;
#define YLGPIO(x) YGPIO(x.port)YPIN(x.pin)
#define YLHIGH(x) yzf_gpio_set_1(x.port,x.pin)
#define YLLOW(x) yzf_gpio_set_0(x.port,x.pin)
#endif


#ifdef __cplusplus
extern "C" {
#endif

void yzl_softport_delayUS(__IO uint32_t ulCount);

void yzl_softport_delayMS(__IO uint32_t ulCount);

#ifdef __cplusplus
}
#endif

///@deprecated
#define delay_ms(x) yzl_softport_delayMS(x)
#define delay_us(x) yzl_softport_delayUS(x)

#endif

