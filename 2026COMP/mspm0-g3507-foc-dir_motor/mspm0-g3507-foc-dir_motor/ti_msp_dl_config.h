/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_Motor_Control */
#define PWM_Motor_Control_INST                                             TIMA0
#define PWM_Motor_Control_INST_IRQHandler                        TIMA0_IRQHandler
#define PWM_Motor_Control_INST_INT_IRQN                         (TIMA0_INT_IRQn)
#define PWM_Motor_Control_INST_CLK_FREQ                                 32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_Motor_Control_C0_PORT                                     GPIOA
#define GPIO_PWM_Motor_Control_C0_PIN                             DL_GPIO_PIN_21
#define GPIO_PWM_Motor_Control_C0_IOMUX                          (IOMUX_PINCM46)
#define GPIO_PWM_Motor_Control_C0_IOMUX_FUNC             IOMUX_PINCM46_PF_TIMA0_CCP0
#define GPIO_PWM_Motor_Control_C0_IDX                        DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_Motor_Control_C1_PORT                                     GPIOA
#define GPIO_PWM_Motor_Control_C1_PIN                             DL_GPIO_PIN_22
#define GPIO_PWM_Motor_Control_C1_IOMUX                          (IOMUX_PINCM47)
#define GPIO_PWM_Motor_Control_C1_IOMUX_FUNC             IOMUX_PINCM47_PF_TIMA0_CCP1
#define GPIO_PWM_Motor_Control_C1_IDX                        DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_Motor_Control_C2_PORT                                     GPIOA
#define GPIO_PWM_Motor_Control_C2_PIN                              DL_GPIO_PIN_7
#define GPIO_PWM_Motor_Control_C2_IOMUX                          (IOMUX_PINCM14)
#define GPIO_PWM_Motor_Control_C2_IOMUX_FUNC             IOMUX_PINCM14_PF_TIMA0_CCP2
#define GPIO_PWM_Motor_Control_C2_IDX                        DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_Motor_Control_C3_PORT                                     GPIOA
#define GPIO_PWM_Motor_Control_C3_PIN                             DL_GPIO_PIN_23
#define GPIO_PWM_Motor_Control_C3_IOMUX                          (IOMUX_PINCM53)
#define GPIO_PWM_Motor_Control_C3_IOMUX_FUNC             IOMUX_PINCM53_PF_TIMA0_CCP3
#define GPIO_PWM_Motor_Control_C3_IDX                        DL_TIMER_CC_3_INDEX

/* Publisher defines */
#define PWM_Motor_Control_INST_PUB_0_CH                                      (2)
#define PWM_Motor_Control_INST_PUB_1_CH                                      (1)



/* Defines for CAPTURE_Angle */
#define CAPTURE_Angle_INST                                               (TIMG8)
#define CAPTURE_Angle_INST_IRQHandler                           TIMG8_IRQHandler
#define CAPTURE_Angle_INST_INT_IRQN                             (TIMG8_INT_IRQn)
#define CAPTURE_Angle_INST_LOAD_VALUE                                   (63999U)
/* GPIO defines for channel 0 */
#define GPIO_CAPTURE_Angle_C0_PORT                                         GPIOA
#define GPIO_CAPTURE_Angle_C0_PIN                                  DL_GPIO_PIN_1
#define GPIO_CAPTURE_Angle_C0_IOMUX                               (IOMUX_PINCM2)
#define GPIO_CAPTURE_Angle_C0_IOMUX_FUNC              IOMUX_PINCM2_PF_TIMG8_CCP0





/* Defines for SystemTick */
#define SystemTick_INST                                                  (TIMG0)
#define SystemTick_INST_IRQHandler                              TIMG0_IRQHandler
#define SystemTick_INST_INT_IRQN                                (TIMG0_INT_IRQn)
#define SystemTick_INST_LOAD_VALUE                                      (31999U)
/* Defines for TIMER_50US */
#define TIMER_50US_INST                                                  (TIMA1)
#define TIMER_50US_INST_IRQHandler                              TIMA1_IRQHandler
#define TIMER_50US_INST_INT_IRQN                                (TIMA1_INT_IRQn)
#define TIMER_50US_INST_LOAD_VALUE                                      (19999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART3
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART3_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_13
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_26
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM35)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM59)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM35_PF_UART3_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM59_PF_UART3_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_0_FBRD_32_MHZ_9600_BAUD                                        (21)




/* Defines for MT6826 */
#define MT6826_INST                                                        SPI0
#define MT6826_INST_IRQHandler                                  SPI0_IRQHandler
#define MT6826_INST_INT_IRQN                                      SPI0_INT_IRQn
#define GPIO_MT6826_PICO_PORT                                             GPIOA
#define GPIO_MT6826_PICO_PIN                                      DL_GPIO_PIN_9
#define GPIO_MT6826_IOMUX_PICO                                  (IOMUX_PINCM20)
#define GPIO_MT6826_IOMUX_PICO_FUNC                  IOMUX_PINCM20_PF_SPI0_PICO
#define GPIO_MT6826_POCI_PORT                                             GPIOA
#define GPIO_MT6826_POCI_PIN                                     DL_GPIO_PIN_10
#define GPIO_MT6826_IOMUX_POCI                                  (IOMUX_PINCM21)
#define GPIO_MT6826_IOMUX_POCI_FUNC                  IOMUX_PINCM21_PF_SPI0_POCI
/* GPIO configuration for MT6826 */
#define GPIO_MT6826_SCLK_PORT                                             GPIOA
#define GPIO_MT6826_SCLK_PIN                                      DL_GPIO_PIN_6
#define GPIO_MT6826_IOMUX_SCLK                                  (IOMUX_PINCM11)
#define GPIO_MT6826_IOMUX_SCLK_FUNC                  IOMUX_PINCM11_PF_SPI0_SCLK
/* Defines for QMI8658 */
#define QMI8658_INST                                                       SPI1
#define QMI8658_INST_IRQHandler                                 SPI1_IRQHandler
#define QMI8658_INST_INT_IRQN                                     SPI1_INT_IRQn
#define GPIO_QMI8658_PICO_PORT                                            GPIOA
#define GPIO_QMI8658_PICO_PIN                                    DL_GPIO_PIN_18
#define GPIO_QMI8658_IOMUX_PICO                                 (IOMUX_PINCM40)
#define GPIO_QMI8658_IOMUX_PICO_FUNC                 IOMUX_PINCM40_PF_SPI1_PICO
#define GPIO_QMI8658_POCI_PORT                                            GPIOA
#define GPIO_QMI8658_POCI_PIN                                    DL_GPIO_PIN_16
#define GPIO_QMI8658_IOMUX_POCI                                 (IOMUX_PINCM38)
#define GPIO_QMI8658_IOMUX_POCI_FUNC                 IOMUX_PINCM38_PF_SPI1_POCI
/* GPIO configuration for QMI8658 */
#define GPIO_QMI8658_SCLK_PORT                                            GPIOA
#define GPIO_QMI8658_SCLK_PIN                                    DL_GPIO_PIN_17
#define GPIO_QMI8658_IOMUX_SCLK                                 (IOMUX_PINCM39)
#define GPIO_QMI8658_IOMUX_SCLK_FUNC                 IOMUX_PINCM39_PF_SPI1_SCLK



/* Defines for ADC_V */
#define ADC_V_INST                                                          ADC0
#define ADC_V_INST_IRQHandler                                    ADC0_IRQHandler
#define ADC_V_INST_INT_IRQN                                      (ADC0_INT_IRQn)
#define ADC_V_ADCMEM_0                                        DL_ADC12_MEM_IDX_0
#define ADC_V_ADCMEM_0_REF                       DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_V_ADCMEM_0_REF_VOLTAGE_V                                         3.3
#define ADC_V_INST_SUB_CH                                                    (1)
#define GPIO_ADC_V_C0_PORT                                                 GPIOA
#define GPIO_ADC_V_C0_PIN                                         DL_GPIO_PIN_27
#define GPIO_ADC_V_IOMUX_C0                                      (IOMUX_PINCM60)
#define GPIO_ADC_V_IOMUX_C0_FUNC                  (IOMUX_PINCM60_PF_UNCONNECTED)

/* Defines for ADC_U */
#define ADC_U_INST                                                          ADC1
#define ADC_U_INST_IRQHandler                                    ADC1_IRQHandler
#define ADC_U_INST_INT_IRQN                                      (ADC1_INT_IRQn)
#define ADC_U_ADCMEM_0                                        DL_ADC12_MEM_IDX_0
#define ADC_U_ADCMEM_0_REF                       DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_U_ADCMEM_0_REF_VOLTAGE_V                                         3.3
#define ADC_U_INST_SUB_CH                                                    (2)
#define GPIO_ADC_U_C0_PORT                                                 GPIOA
#define GPIO_ADC_U_C0_PIN                                         DL_GPIO_PIN_15
#define GPIO_ADC_U_IOMUX_C0                                      (IOMUX_PINCM37)
#define GPIO_ADC_U_IOMUX_C0_FUNC                  (IOMUX_PINCM37_PF_UNCONNECTED)



/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (1)
#define ADC_V_INST_DMA_TRIGGER                        (DMA_ADC0_EVT_GEN_BD_TRIG)
/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define ADC_U_INST_DMA_TRIGGER                        (DMA_ADC1_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group SPI_CS */
#define SPI_CS_PORT                                                      (GPIOA)

/* Defines for QMI8658_CS: GPIOA.25 with pinCMx 55 on package pin 29 */
#define SPI_CS_QMI8658_CS_PIN                                   (DL_GPIO_PIN_25)
#define SPI_CS_QMI8658_CS_IOMUX                                  (IOMUX_PINCM55)
/* Defines for MT6826_CS: GPIOA.3 with pinCMx 8 on package pin 7 */
#define SPI_CS_MT6826_CS_PIN                                     (DL_GPIO_PIN_3)
#define SPI_CS_MT6826_CS_IOMUX                                    (IOMUX_PINCM8)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_Motor_Control_init(void);
void SYSCFG_DL_CAPTURE_Angle_init(void);
void SYSCFG_DL_SystemTick_init(void);
void SYSCFG_DL_TIMER_50US_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_MT6826_init(void);
void SYSCFG_DL_QMI8658_init(void);
void SYSCFG_DL_ADC_V_init(void);
void SYSCFG_DL_ADC_U_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
