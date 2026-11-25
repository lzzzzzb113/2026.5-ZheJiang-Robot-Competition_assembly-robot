//
// Created by 32827 on 2023/11/15.
//

#ifndef YZ_GPIO_H
#define YZ_GPIO_H
#include "yzf.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef GPIO_Regs* YZF_GPIO_POINTER;
typedef enum {
    YZF_PIN_0 = 0U,
    YZF_PIN_1 = 1U,
    YZF_PIN_2 = 2U,
    YZF_PIN_3 = 3U,
    YZF_PIN_4 = 4U,
    YZF_PIN_5 = 5U,
    YZF_PIN_6 = 6U,
    YZF_PIN_7 = 7U,
    YZF_PIN_8 = 8U,
    YZF_PIN_9 = 9U,
    YZF_PIN_10 = 10U,
    YZF_PIN_11 = 11U,
    YZF_PIN_12 = 12U,
    YZF_PIN_13 = 13U,
    YZF_PIN_14 = 14U,
    YZF_PIN_15 = 15U,
    YZF_PIN_UNKnow = 16U
} YZF_GPIO_PIN;
YZF_RESULT yzf_gpio_open_clock(YZF_GPIO_POINTER gpio);
YZF_RESULT yzf_gpio_close_clock(YZF_GPIO_POINTER gpio);
YZF_RESULT yzf_gpio_set_Pull_Up(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_Pull_Down(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_PP(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_OD(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_IN_FLOAT(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_IN_PULL_UP(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_IN_PULL_DOWN(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_IN_ANALOG(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_0(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_set_1(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
YZF_BOOL yzf_gpio_get(YZF_GPIO_POINTER gpio,YZF_GPIO_PIN pin);
/**
 * 便捷调用模式
 * 将通过yzf_gpio()指定后可调用便捷方式
 * */
YZF_RESULT yzf_gpio(YZF_GPIO_POINTER gpio);
YZF_RESULT yzf_gpio_pin(YZF_GPIO_PIN pin);
YZF_RESULT yzf_gpio_open_clock_();
YZF_RESULT yzf_gpio_close_clock_();
YZF_RESULT yzf_gpio_set_PP_();
YZF_RESULT yzf_gpio_set_OD_();
YZF_RESULT yzf_gpio_set_IN_FLOAT_();
YZF_RESULT yzf_gpio_set_IN_PULL_UP_();
YZF_RESULT yzf_gpio_set_Pull_Up_();
YZF_RESULT yzf_gpio_set_Pull_Down_();
YZF_RESULT yzf_gpio_set_IN_PULL_DOWN_();
YZF_RESULT yzf_gpio_set_IN_ANALOG_();
YZF_RESULT yzf_gpio_set_MIDDLE_();
YZF_RESULT yzf_gpio_set_LOW_();
YZF_RESULT yzf_gpio_set_HIGH_();
YZF_RESULT yzf_gpio_set_0_();
YZF_RESULT yzf_gpio_set_1_();
YZF_RESULT yzf_gpio_lock_();
YZF_RESULT yzf_gpio_unlock_();
YZF_RESULT yzf_gpio_set_af_(uint8_t type);
YZF_BOOL yzf_gpio_get_();
#define YGPIO(n) yzf_gpio(n);
#define YPIN(n) yzf_gpio_pin(n);

#ifdef __cplusplus
}
#endif
#endif //YZ_GPIO_H