#ifndef STAGE_CONTROL_H
#define STAGE_CONTROL_H

#include "Motorclass.hpp"

/* 原有图形枚举 */
typedef enum {
    RED_CIRCLE,
    BLUE_TRAPEZOID,
    BLUE_TRIANGLE,
    RED_SQUARE,
    BLUE_CIRCLE,
    RED_TRAPEZOID,
    RED_TRIANGLE,
    BLUE_SQUARE
} Shape;
/* 状态 */
typedef enum {
    STOP,
    SWITCH
} State;

void start_mode_red_init(uint32_t time);

void start_mode_blue_init(uint32_t time);

YZF_BOOL stage_act(int ir, uint32_t time);

Shape get_current_shape(void);

position_stage get_current_shape_ex(void);

uint32_t get_current_index_duration(uint32_t now);

int is_ready(void);   /* 1 表示处于 STOP 状态 */

const char* map_position_stage_to_name(position_stage ps);

#endif