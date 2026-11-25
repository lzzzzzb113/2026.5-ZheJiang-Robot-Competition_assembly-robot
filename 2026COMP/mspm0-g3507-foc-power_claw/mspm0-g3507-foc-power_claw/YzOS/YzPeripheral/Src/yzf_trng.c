//
// Created by 32827 on 2026/1/16.
//

#include "yzf_trng.h"

uint32_t yzf_trng_random(void){
    DL_TRNG_sendCommand(TRNG, DL_TRNG_CMD_NORM_FUNC);
    while (!DL_TRNG_isCaptureReady(TRNG));
    return DL_TRNG_getCapture(TRNG);
}