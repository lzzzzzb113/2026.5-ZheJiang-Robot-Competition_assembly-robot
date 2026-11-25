//
// Created by 32827 on 2023/11/15.
//
#include "yzf_gpio.h"
uint8_t IOMUXA_MAP[]={1,2,7,8,9,10,11,14,19,20,21,22,34,35,36,37,38,39,40,41,42,
                           46,47,53,54,55,59,60,3,4,5,6};
uint8_t IOMUXB_MAP[]={12,13,15,16,17,18,23,24,25,26,27,28,29,30,31,32,33,
                       43,44,45,48,49,50,51,52,56,57,58};

YZF_RESULT yzf_gpio_open_clock(YZF_GPIO_POINTER gpio){
    gpio->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE);
}
YZF_RESULT yzf_gpio_close_clock(YZF_GPIO_POINTER gpio){
    gpio->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_DISABLE);
}

YZF_RESULT yzf_gpio_set_PP(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_initDigitalOutput(IOMUXA_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXA_MAP[pin], DL_GPIO_RESISTOR_NONE);
    }else if(gpio==GPIOB){
        DL_GPIO_initDigitalOutput(IOMUXB_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXB_MAP[pin], DL_GPIO_RESISTOR_NONE);
    }

    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_OD(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_initDigitalOutput(IOMUXA_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXA_MAP[pin], DL_GPIO_RESISTOR_NONE);
        DL_GPIO_enableHiZ(IOMUXA_MAP[pin]);
    }else if(gpio==GPIOB){
        DL_GPIO_initDigitalOutput(IOMUXB_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXB_MAP[pin], DL_GPIO_RESISTOR_NONE);
        DL_GPIO_enableHiZ(IOMUXB_MAP[pin]);
    }
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_IN_FLOAT(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_initDigitalInput(IOMUXA_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXA_MAP[pin], DL_GPIO_RESISTOR_NONE);
    }else if(gpio==GPIOB){
        DL_GPIO_initDigitalInput(IOMUXB_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXB_MAP[pin], DL_GPIO_RESISTOR_NONE);;
    }
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_IN_PULL_UP(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_initDigitalInput(IOMUXA_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXA_MAP[pin], DL_GPIO_RESISTOR_PULL_UP);
    }else if(gpio==GPIOB){
        DL_GPIO_initDigitalInput(IOMUXB_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXB_MAP[pin], DL_GPIO_RESISTOR_PULL_UP);;
    }
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_IN_PULL_DOWN(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_initDigitalInput(IOMUXA_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXA_MAP[pin], DL_GPIO_RESISTOR_PULL_DOWN);
    }else if(gpio==GPIOB){
        DL_GPIO_initDigitalInput(IOMUXB_MAP[pin]);
        DL_GPIO_setDigitalInternalResistor(IOMUXB_MAP[pin], DL_GPIO_RESISTOR_PULL_DOWN);
    }
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_IN_ANALOG(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_initDigitalInput(IOMUXA_MAP[pin]);
    }else if(gpio==GPIOB){
        DL_GPIO_initDigitalInput(IOMUXB_MAP[pin]);
    }
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_0(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    gpio->DOUTCLR31_0 = 0x1<<pin;
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_1(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    gpio->DOUTSET31_0 = 0x1<<pin;
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_Pull_Up(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_setDigitalInternalResistor(IOMUXA_MAP[pin], DL_GPIO_RESISTOR_PULL_UP);
    }else if(gpio==GPIOB){
        DL_GPIO_setDigitalInternalResistor(IOMUXB_MAP[pin], DL_GPIO_RESISTOR_PULL_UP);;
    }
    return YZF_OK;
}
YZF_RESULT yzf_gpio_set_Pull_Down(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    if(gpio==GPIOA){
        DL_GPIO_setDigitalInternalResistor(IOMUXA_MAP[pin], DL_GPIO_RESISTOR_PULL_DOWN);
    }else if(gpio==GPIOB){
        DL_GPIO_setDigitalInternalResistor(IOMUXB_MAP[pin], DL_GPIO_RESISTOR_PULL_DOWN);;
    }
    return YZF_OK;
}
YZF_BOOL yzf_gpio_get(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin){
    return (gpio->DIN31_0 & (0b1UL<<pin))?YZF_TRUE:YZF_FALSE;
}

YZF_GPIO_POINTER  mGpio= (YZF_GPIO_POINTER) YZF_NULL;
YZF_GPIO_PIN  mPin=YZF_NULL;
YZF_RESULT yzf_gpio(YZF_GPIO_POINTER gpio){
    mGpio=gpio;
    return YZF_OK;
}
YZF_RESULT yzf_gpio_pin(YZF_GPIO_PIN pin){
    mPin=pin;
    return YZF_OK;
}
YZF_RESULT yzf_gpio_open_clock_(){
    YZF_CHECK_PARAM(mGpio)
    return yzf_gpio_open_clock(mGpio);
}
YZF_RESULT yzf_gpio_close_clock_(){
    YZF_CHECK_PARAM(mGpio)
    return yzf_gpio_close_clock(mGpio);
}
YZF_RESULT yzf_gpio_set_PP_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_PP(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_Pull_Up_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_Pull_Up(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_Pull_Down_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_Pull_Down(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_OD_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_OD(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_IN_FLOAT_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_IN_FLOAT(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_IN_PULL_UP_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_IN_PULL_UP(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_IN_PULL_DOWN_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_IN_PULL_DOWN(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_IN_ANALOG_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_IN_ANALOG(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_0_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_0(mGpio,mPin);
}
YZF_RESULT yzf_gpio_set_1_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_set_1(mGpio,mPin);
}
YZF_BOOL yzf_gpio_get_(){
    YZF_CHECK_PARAM(mGpio)
    YZF_CHECK_PARAM(mPin)
    return yzf_gpio_get(mGpio,mPin);
}