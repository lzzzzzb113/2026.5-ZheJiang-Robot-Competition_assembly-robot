//
// Created by 32827 on 2026/4/11.
//

#ifndef STM32H723_LTS_BSP_YZF_RNG_H
#define STM32H723_LTS_BSP_YZF_RNG_H

#include "yzf.h"
#ifdef __cplusplus
extern "C" {
#endif

    YZF_RESULT yzf_rng_enable_clock();

    YZF_RESULT yzf_rng_enable_clock_err_detect();

    YZF_RESULT yzf_rng_disable_clock_err_detect();

    YZF_RESULT yzf_rng_interrupt_enable();

    YZF_RESULT yzf_rng_interrupt_disable();

    uint32_t yzf_rng_get_random_number();

    YZF_RESULT yzf_rng_enable();

    YZF_RESULT yzf_rng_disable();

#ifdef __cplusplus
}
#endif

#endif //STM32H723_LTS_BSP_YZF_RNG_H