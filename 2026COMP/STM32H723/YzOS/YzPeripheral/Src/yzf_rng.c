//
// Created by 32827 on 2026/4/11.
//

#include "yzf_rng.h"

YZF_RESULT yzf_rng_enable_clock()
{
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    return YZF_OK;
}

YZF_RESULT yzf_rng_enable_clock_err_detect()
{
    RNG->CR &= ~RNG_CR_CED;
    return YZF_OK;
}

YZF_RESULT yzf_rng_disable_clock_err_detect()
{
    RNG->CR |= RNG_CR_CED;
    return YZF_OK;
}

YZF_RESULT yzf_rng_interrupt_enable()
{
    RNG->CR |= RNG_CR_IE;
    return YZF_OK;
}

YZF_RESULT yzf_rng_interrupt_disable()
{
    RNG->CR &= ~RNG_CR_IE;
    return YZF_OK;
}

uint32_t yzf_rng_get_random_number()
{
    while(!(RNG->SR & RNG_SR_DRDY)) {}
    return RNG->DR;
}

YZF_RESULT yzf_rng_enable()
{
    RNG->CR |= RNG_CR_RNGEN;
    return YZF_OK;
}

YZF_RESULT yzf_rng_disable()
{
    RNG->CR &= ~RNG_CR_RNGEN;
    return YZF_OK;
}