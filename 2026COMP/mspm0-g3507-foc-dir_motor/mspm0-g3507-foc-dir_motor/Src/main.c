#include "yzf_global_sheet.h"
#include "yz_time.h"
#include "yz_log.h"
#include "yz_status.h"
#include "yz_main.h"
#include "uart_package_connect.h"

UartConnect uart_connect;
void SystemTick_INST_IRQHandler(void){
    yz_heart_handler(1);
}

uint8_t uart0_rx_buf[256];
uint8_t uart0_r_index = 0;
uint8_t uart0_w_index = 0;
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
            {
                uart0_rx_buf[uart0_w_index++] = DL_UART_receiveData(UART_0_INST);
                if (uart0_w_index >= sizeof(uart0_rx_buf))
                    uart0_w_index = 0;
            }
        break;
        default:
            break;
    }
}
YZF_BOOL uart0_rx_buf_act()
{
    if (uart0_w_index != uart0_r_index)
    {
        package_put(&uart_connect, uart0_rx_buf[uart0_r_index++]);
        if (uart0_r_index >= sizeof(uart0_rx_buf))
        {
            uart0_r_index = 0;
        }
        return YZF_TRUE;
    }else
    {
        return YZF_FALSE;
    }
}
void yz_shell_send(void *file,uint8_t *str,  size_t len) {
    for(uint32_t i = 0; i < len; i++){
        DL_UART_transmitDataBlocking(UART_0_INST,str[i]);
    }
}
uint16_t adc_u[8];
uint16_t adc_v[8];
int main(){

    SYSCFG_DL_init();

    NVIC_EnableIRQ(SystemTick_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_Angle_INST_INT_IRQN);
    DL_Timer_startCounter(SystemTick_INST);
    DL_Timer_startCounter(TIMER_50US_INST);

    DL_DMA_setSrcAddr(DMA,DMA_CH0_CHAN_ID,DL_ADC12_getMemResultAddress(ADC_U_INST,ADC_U_ADCMEM_0));
    DL_DMA_setSrcAddr(DMA,DMA_CH1_CHAN_ID,DL_ADC12_getMemResultAddress(ADC_V_INST,ADC_V_ADCMEM_0));
    DL_DMA_setDestAddr(DMA,DMA_CH0_CHAN_ID,(uint32_t) adc_u);
    DL_DMA_setDestAddr(DMA,DMA_CH1_CHAN_ID,(uint32_t) adc_v);
    DL_DMA_enableChannel(DMA,DMA_CH0_CHAN_ID);
    DL_DMA_enableChannel(DMA,DMA_CH1_CHAN_ID);

    DL_Timer_startCounter(PWM_Motor_Control_INST);

    core_main();
}


YZF_BOOL OutError=YZF_FALSE;
//跳出错误函数
void user_OutError(){
    OutError=YZF_TRUE;
}

void user_error_handler(YZF_RESULT error){
    switch (error) {
        case YZF_NOT_FOUND:
            yz_error("[GlobalSheet: %hd] 参数未找到.\n",yz_status_sheet_get(YZ_FLAG_ERROR_HANDLER_INDEX));
            break;
        case YZF_ERROR:
            yz_error("[GlobalSheet: %hd] 未知错误.\n",yz_status_sheet_get(YZ_FLAG_ERROR_HANDLER_INDEX));
            break;
        case YZF_NOT_INIT:
            yz_error("[GlobalSheet: %hd] 使用的函数，所需对象未被初始化.\n",yz_status_sheet_get(YZ_FLAG_ERROR_HANDLER_INDEX));
            break;
        case YZF_CRC_ERROR:
            yz_error("[GlobalSheet: %hd] CRC校验错误.\n",yz_status_sheet_get(YZ_FLAG_ERROR_HANDLER_INDEX));
            break;
        case YZF_CLOCK_NOT_READY:
            yz_error("[GlobalSheet: %hd] 时钟未就绪.\n",yz_status_sheet_get(YZ_FLAG_ERROR_HANDLER_INDEX));
            break;
        case YZF_NOTHING:
#ifdef osCMSIS
            osDelay(1);
#endif
#ifdef GD32F470
            vTaskDelay(1);
#endif
            user_OutError();
            break;
        default:
            yz_error("[GlobalSheet: %hd] 未被识别的错误类型. Code: %d\n",yz_status_sheet_get(YZ_FLAG_ERROR_HANDLER_INDEX),error);
    }
    while (!OutError){
        __asm__("nop");
    }
    OutError=YZF_FALSE;
}