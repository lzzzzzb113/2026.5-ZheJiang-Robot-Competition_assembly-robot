#include "StageStauts.hpp"

#define QUEUE_SIZE 8

static const Shape queue[QUEUE_SIZE] = {
        RED_CIRCLE,       /* 0 */
        BLUE_TRAPEZOID,   /* 1 */
        BLUE_TRIANGLE,    /* 2 */
        RED_SQUARE,    /* 3 */
        BLUE_CIRCLE,      /* 4 */
        RED_TRAPEZOID,   /* 5 */
        RED_TRIANGLE,     /* 6 */
        BLUE_SQUARE       /* 7 */
};

static int currentIndex;
static State currentState;
static uint32_t last_index_change_time;   /* 索引最近一次改变时的系统时间 */

void start_mode_red_init(uint32_t time)
{
    currentIndex = 0;
    currentState = STOP;
    last_index_change_time = time;
}

void start_mode_blue_init(uint32_t time)
{
    currentIndex = 4;
    currentState = STOP;
    last_index_change_time = time;
}

YZF_BOOL stage_act(int ir, uint32_t time)
{
    switch (currentState) {
        case STOP:
            /* 常态 ir=0，上升沿时切换一次 */
            if (ir == 1) {
                currentIndex = (currentIndex + 1) % QUEUE_SIZE;
                currentState = SWITCH;
                last_index_change_time = time;   /* 记录切换时刻 */
                now_wheel = get_current_shape_ex();
                return YZF_TRUE;
            }
            break;

        case SWITCH:
            /* 等待 ir 回到 0，回到 STOP 为下一次上升沿做准备 */
            if (ir == 0) {
                currentState = STOP;
            }
            break;
    }
    return YZF_FALSE;
}

Shape get_current_shape(void)
{
    return queue[currentIndex];
}

position_stage get_current_shape_ex(void)
{
    switch (get_current_shape()) {
        case RED_CIRCLE:      return RED_YUAN;
        case BLUE_TRAPEZOID:  return BLUE_TI;
        case BLUE_TRIANGLE:   return BLUE_SAN;
        case RED_SQUARE:   return RED_SI;
        case BLUE_CIRCLE:     return BLUE_YUAN;
        case RED_TRAPEZOID:  return RED_TI;
        case RED_TRIANGLE:    return RED_SAN;
        case BLUE_SQUARE:     return BLUE_SI;
        default:              return RED_YUAN;
    }
}

uint32_t get_current_index_duration(uint32_t now)
{
    return now - last_index_change_time;
}

int is_ready(void)
{
    return currentState == STOP;
}

/* 源文件实现 */
const char* map_position_stage_to_name(position_stage ps)
{
    switch (ps) {
        case RED_YUAN:   return "R_YUAN";
        case BLUE_SI:    return "B_SI";
        case RED_SAN:    return "R_SAN";
        case RED_TI:     return "R_TI";
        case BLUE_YUAN:  return "B_YUAN";
        case RED_SI:     return "R_SI";
        case BLUE_SAN:   return "B_SAN";
        case BLUE_TI:    return "B_TI";
        default:         return "???";
    }
}