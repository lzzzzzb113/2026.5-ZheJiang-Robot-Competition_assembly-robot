/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_Motor_ControlBackup;
DL_TimerA_backupConfig gTIMER_50USBackup;
DL_UART_Main_backupConfig gUART_0Backup;
DL_SPI_backupConfig gMT6826Backup;
DL_SPI_backupConfig gQMI8658Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_Motor_Control_init();
    SYSCFG_DL_CAPTURE_Angle_init();
    SYSCFG_DL_SystemTick_init();
    SYSCFG_DL_TIMER_50US_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_MT6826_init();
    SYSCFG_DL_QMI8658_init();
    SYSCFG_DL_ADC_V_init();
    SYSCFG_DL_ADC_U_init();
    SYSCFG_DL_DMA_init();
    /* Ensure backup structures have no valid state */
	gPWM_Motor_ControlBackup.backupRdy 	= false;

	gTIMER_50USBackup.backupRdy 	= false;
	gUART_0Backup.backupRdy 	= false;
	gMT6826Backup.backupRdy 	= false;
	gQMI8658Backup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_Motor_Control_INST, &gPWM_Motor_ControlBackup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_50US_INST, &gTIMER_50USBackup);
	retStatus &= DL_UART_Main_saveConfiguration(UART_0_INST, &gUART_0Backup);
	retStatus &= DL_SPI_saveConfiguration(MT6826_INST, &gMT6826Backup);
	retStatus &= DL_SPI_saveConfiguration(QMI8658_INST, &gQMI8658Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_Motor_Control_INST, &gPWM_Motor_ControlBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_50US_INST, &gTIMER_50USBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(UART_0_INST, &gUART_0Backup);
	retStatus &= DL_SPI_restoreConfiguration(MT6826_INST, &gMT6826Backup);
	retStatus &= DL_SPI_restoreConfiguration(QMI8658_INST, &gQMI8658Backup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_TimerA_reset(PWM_Motor_Control_INST);
    DL_TimerG_reset(CAPTURE_Angle_INST);
    DL_TimerG_reset(SystemTick_INST);
    DL_TimerA_reset(TIMER_50US_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_SPI_reset(MT6826_INST);
    DL_SPI_reset(QMI8658_INST);
    DL_ADC12_reset(ADC_V_INST);
    DL_ADC12_reset(ADC_U_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_TimerA_enablePower(PWM_Motor_Control_INST);
    DL_TimerG_enablePower(CAPTURE_Angle_INST);
    DL_TimerG_enablePower(SystemTick_INST);
    DL_TimerA_enablePower(TIMER_50US_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_SPI_enablePower(MT6826_INST);
    DL_SPI_enablePower(QMI8658_INST);
    DL_ADC12_enablePower(ADC_V_INST);
    DL_ADC12_enablePower(ADC_U_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motor_Control_C0_IOMUX,GPIO_PWM_Motor_Control_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motor_Control_C0_PORT, GPIO_PWM_Motor_Control_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motor_Control_C1_IOMUX,GPIO_PWM_Motor_Control_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motor_Control_C1_PORT, GPIO_PWM_Motor_Control_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motor_Control_C2_IOMUX,GPIO_PWM_Motor_Control_C2_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motor_Control_C2_PORT, GPIO_PWM_Motor_Control_C2_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motor_Control_C3_IOMUX,GPIO_PWM_Motor_Control_C3_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motor_Control_C3_PORT, GPIO_PWM_Motor_Control_C3_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_CAPTURE_Angle_C0_IOMUX,GPIO_CAPTURE_Angle_C0_IOMUX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);

    
	DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_MT6826_IOMUX_SCLK, GPIO_MT6826_IOMUX_SCLK_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
	DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_MT6826_IOMUX_PICO, GPIO_MT6826_IOMUX_PICO_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_MT6826_IOMUX_POCI, GPIO_MT6826_IOMUX_POCI_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    
	DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_QMI8658_IOMUX_SCLK, GPIO_QMI8658_IOMUX_SCLK_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
	DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_QMI8658_IOMUX_PICO, GPIO_QMI8658_IOMUX_PICO_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_QMI8658_IOMUX_POCI, GPIO_QMI8658_IOMUX_POCI_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(SPI_CS_QMI8658_CS_IOMUX);

    DL_GPIO_initDigitalOutput(SPI_CS_MT6826_CS_IOMUX);

    DL_GPIO_setPins(SPI_CS_PORT, SPI_CS_QMI8658_CS_PIN |
		SPI_CS_MT6826_CS_PIN);
    DL_GPIO_enableOutput(SPI_CS_PORT, SPI_CS_QMI8658_CS_PIN |
		SPI_CS_MT6826_CS_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_Motor_ControlClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_Motor_ControlConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_CENTER_ALIGN,
    .period = 1024,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_Motor_Control_init(void) {

    DL_TimerA_setClockConfig(
        PWM_Motor_Control_INST, (DL_TimerA_ClockConfig *) &gPWM_Motor_ControlClockConfig);

    DL_TimerA_initPWMMode(
        PWM_Motor_Control_INST, (DL_TimerA_PWMConfig *) &gPWM_Motor_ControlConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_Motor_Control_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_Motor_Control_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_Motor_Control_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_Motor_Control_INST, 512, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_Motor_Control_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_Motor_Control_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_Motor_Control_INST, 512, DL_TIMER_CC_1_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_Motor_Control_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_Motor_Control_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_Motor_Control_INST, 511, DL_TIMER_CC_2_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_Motor_Control_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_3_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_Motor_Control_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_Motor_Control_INST, 512, DL_TIMER_CC_3_INDEX);

    DL_TimerA_enableClock(PWM_Motor_Control_INST);
    DL_TimerA_enableEvent(PWM_Motor_Control_INST, DL_TIMERA_EVENT_ROUTE_1, (DL_TIMERA_EVENT_CC2_UP_EVENT));

    DL_TimerA_setPublisherChanID(PWM_Motor_Control_INST, DL_TIMERA_PUBLISHER_INDEX_0, PWM_Motor_Control_INST_PUB_0_CH);

    DL_TimerA_enableEvent(PWM_Motor_Control_INST, DL_TIMERA_EVENT_ROUTE_2, (DL_TIMERA_EVENT_CC2_UP_EVENT));

    DL_TimerA_setPublisherChanID(PWM_Motor_Control_INST, DL_TIMERA_PUBLISHER_INDEX_1, PWM_Motor_Control_INST_PUB_1_CH);

    
    DL_TimerA_setCCPDirection(PWM_Motor_Control_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT | DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gCAPTURE_AngleClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CAPTURE_Angle_INST_LOAD_VALUE = (2ms * 32000000 Hz) - 1
 */
static const DL_TimerG_CaptureConfig gCAPTURE_AngleCaptureConfig = {
    .captureMode    = DL_TIMER_CAPTURE_MODE_PULSE_WIDTH,
    .period         = CAPTURE_Angle_INST_LOAD_VALUE,
    .startTimer     = DL_TIMER_STOP,
    .edgeCaptMode   = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
    .inputChan      = DL_TIMER_INPUT_CHAN_0,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_CAPTURE_Angle_init(void) {

    DL_TimerG_setClockConfig(CAPTURE_Angle_INST,
        (DL_TimerG_ClockConfig *) &gCAPTURE_AngleClockConfig);

    DL_TimerG_initCaptureMode(CAPTURE_Angle_INST,
        (DL_TimerG_CaptureConfig *) &gCAPTURE_AngleCaptureConfig);
    DL_TimerG_enableInterrupt(CAPTURE_Angle_INST , DL_TIMERG_INTERRUPT_CC0_DN_EVENT);

    NVIC_SetPriority(CAPTURE_Angle_INST_INT_IRQN, 1);
    DL_TimerG_enableClock(CAPTURE_Angle_INST);

}


/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gSystemTickClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * SystemTick_INST_LOAD_VALUE = (1ms * 32000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gSystemTickTimerConfig = {
    .period     = SystemTick_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_SystemTick_init(void) {

    DL_TimerG_setClockConfig(SystemTick_INST,
        (DL_TimerG_ClockConfig *) &gSystemTickClockConfig);

    DL_TimerG_initTimerMode(SystemTick_INST,
        (DL_TimerG_TimerConfig *) &gSystemTickTimerConfig);
    DL_TimerG_enableInterrupt(SystemTick_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(SystemTick_INST_INT_IRQN, 1);
    DL_TimerG_enableClock(SystemTick_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   20000 Hz = 4000000 Hz / (8 * (199 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_50USClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 199U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_50US_INST_LOAD_VALUE = (1s * 20000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMER_50USTimerConfig = {
    .period     = TIMER_50US_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_50US_init(void) {

    DL_TimerA_setClockConfig(TIMER_50US_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_50USClockConfig);

    DL_TimerA_initTimerMode(TIMER_50US_INST,
        (DL_TimerA_TimerConfig *) &gTIMER_50USTimerConfig);
    DL_TimerA_enableClock(TIMER_50US_INST);





}


static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.24
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_32_MHZ_9600_BAUD, UART_0_FBRD_32_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_0_INST);
}

static const DL_SPI_Config gMT6826_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gMT6826_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_MT6826_init(void) {
    DL_SPI_setClockConfig(MT6826_INST, (DL_SPI_ClockConfig *) &gMT6826_clockConfig);

    DL_SPI_init(MT6826_INST, (DL_SPI_Config *) &gMT6826_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     4000000 = (32000000)/((1 + 3) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(MT6826_INST, 3);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(MT6826_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(MT6826_INST);
}
static const DL_SPI_Config gQMI8658_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gQMI8658_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_QMI8658_init(void) {
    DL_SPI_setClockConfig(QMI8658_INST, (DL_SPI_ClockConfig *) &gQMI8658_clockConfig);

    DL_SPI_init(QMI8658_INST, (DL_SPI_Config *) &gQMI8658_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     4000000 = (32000000)/((1 + 3) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(QMI8658_INST, 3);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(QMI8658_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_EMPTY);

    /* Enable module */
    DL_SPI_enable(QMI8658_INST);
}

/* ADC_V Initialization */
static const DL_ADC12_ClockConfig gADC_VClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC_V_init(void)
{
    DL_ADC12_setClockConfig(ADC_V_INST, (DL_ADC12_ClockConfig *) &gADC_VClockConfig);
    DL_ADC12_initSingleSample(ADC_V_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC_V_INST, ADC_V_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC_V_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC_V_INST,8);
    DL_ADC12_enableDMA(ADC_V_INST);
    DL_ADC12_setDMASamplesCnt(ADC_V_INST,1);
    DL_ADC12_enableDMATrigger(ADC_V_INST,(DL_ADC12_DMA_MEM0_RESULT_LOADED));
    DL_ADC12_setSubscriberChanID(ADC_V_INST,ADC_V_INST_SUB_CH);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC_V_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    DL_ADC12_enableInterrupt(ADC_V_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    DL_ADC12_enableConversions(ADC_V_INST);
}
/* ADC_U Initialization */
static const DL_ADC12_ClockConfig gADC_UClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC_U_init(void)
{
    DL_ADC12_setClockConfig(ADC_U_INST, (DL_ADC12_ClockConfig *) &gADC_UClockConfig);
    DL_ADC12_initSingleSample(ADC_U_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC_U_INST, ADC_U_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC_U_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC_U_INST,8);
    DL_ADC12_enableDMA(ADC_U_INST);
    DL_ADC12_setDMASamplesCnt(ADC_U_INST,1);
    DL_ADC12_enableDMATrigger(ADC_U_INST,(DL_ADC12_DMA_MEM0_RESULT_LOADED));
    DL_ADC12_setSubscriberChanID(ADC_U_INST,ADC_U_INST_SUB_CH);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC_U_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    DL_ADC12_enableInterrupt(ADC_U_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    DL_ADC12_enableConversions(ADC_U_INST);
}

static const DL_DMA_Config gDMA_CH1Config = {
    .transferMode   = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_HALF_WORD,
    .srcWidth       = DL_DMA_WIDTH_HALF_WORD,
    .trigger        = ADC_V_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH1_init(void)
{
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, 1);
    DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID , (DL_DMA_Config *) &gDMA_CH1Config);
}
static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_HALF_WORD,
    .srcWidth       = DL_DMA_WIDTH_HALF_WORD,
    .trigger        = ADC_U_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void)
{
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 1);
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID , (DL_DMA_Config *) &gDMA_CH0Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_CH1_init();
    SYSCFG_DL_DMA_CH0_init();
}


