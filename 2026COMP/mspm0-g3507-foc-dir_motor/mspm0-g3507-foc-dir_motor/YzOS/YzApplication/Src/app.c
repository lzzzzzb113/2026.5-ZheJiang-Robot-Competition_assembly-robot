//
// Created by 32827 on 2025/11/24.
//
#include "app.h"
#include "ti_msp_dl_config.h"
#include "IQmathLib.h"
#include "uart_package_connect.h"

#include "yzx_base.h"
#include "yzl_mem.h"
#include "yzl_pwm_foc.h"
#include "yzl_spi_mt6826.h"
#include "yzl_spi_qmi8658a.h"
#include "yz_system_scheduler.h"
#include "yz_system_type.h"

float update_angle(YZ_PWM_FOC *args);

/***    MEM FOR 8KB Keep    ***/
uint8_t CppRamRaw[8*1024]={0};

YZ_MemManger yzlMem={
    .baseAddress=(uint32_t) CppRamRaw,
    .size=8*1024,
    .pMalloc=YZF_NULL_PTR,
    .pFree=YZF_NULL_PTR
};

QMI8658 qmi8658={
    .user_id=0x00,
    .spi={
        .hard_spi = QMI8658_INST,
        .isHard = YZF_TRUE,
        .define = {
            .mode = YZL_SOFTPORT_SPI_MODE_0,
            .MSB = YZF_TRUE,
            .cs = {
                .pin = YZF_PIN_CONV_FROM(SPI_CS_QMI8658_CS_PIN),
                .port = SPI_CS_PORT,
            },
            .clk = {
                .pin = YZF_PIN_CONV_FROM(GPIO_QMI8658_SCLK_PIN),
                .port = GPIO_QMI8658_SCLK_PORT,
            },
            .mosi = {
                .pin = YZF_PIN_CONV_FROM(GPIO_QMI8658_PICO_PIN),
                .port = GPIO_QMI8658_PICO_PORT,
            },
            .miso ={
                .pin = YZF_PIN_CONV_FROM(GPIO_QMI8658_POCI_PIN),
                .port = GPIO_QMI8658_POCI_PORT,
            }
        }
    },
    .AccIsOn = YZF_TRUE,
    .GyroIsOn = YZF_TRUE,
    .accelerometer_scale_type = QMI8658_Accelerometer_2G,
    .accelerometer_speed_type = QMI8658_AccelerometerSpeed_N_1000Hz,
    .gyroscope_scale_type = QMI8658_Gyroscope_1024DPS,
    .gyroscope_speed_type = QMI8658_GyroscopeSpeed_896_8
};

MT6826S mt6826s={
    .user_id = 0xAA,
    .spi = {
        .hard_spi = MT6826_INST,
        .isHard = YZF_TRUE,
        .define = {
            .mode = YZL_SOFTPORT_SPI_MODE_3,
            .MSB = YZF_TRUE,
            .cs = {
                .pin = YZF_PIN_CONV_FROM(SPI_CS_MT6826_CS_PIN),
                .port = SPI_CS_PORT,
            },
            .clk = {
                .pin = YZF_PIN_CONV_FROM(GPIO_MT6826_SCLK_PIN),
                .port = GPIO_MT6826_SCLK_PORT,
            },
            .mosi = {
                .pin = YZF_PIN_CONV_FROM(GPIO_MT6826_PICO_PIN),
                .port = GPIO_MT6826_PICO_PORT
            },
            .miso ={
                .pin = YZF_PIN_CONV_FROM(GPIO_MT6826_POCI_PIN),
                .port = GPIO_MT6826_POCI_PORT
            },
        }
    }
};

YZ_PWM_FOC foc={
    .Freq = 31250,
    .MaxDuty = 510,
    .MinDuty = 2,

    .pwm_control1 = PWM_Motor_Control_INST,
    .ch1 = DL_TIMER_CC_0_INDEX,
    .pwm_control2 = PWM_Motor_Control_INST,
    .ch2 = DL_TIMER_CC_1_INDEX,
    .pwm_control3 = PWM_Motor_Control_INST,
    .ch3 = DL_TIMER_CC_3_INDEX,

    .PolarNum = 7,
    .SpeedUpdateTime = 0.01f,//10ms
    .zero_raw_rad = 0,
    .position_offset_rad = 0,

    .position_pid = {
        .kp = 10.0f,
        .ki = 0.4f,
        .kd = 0.0f,

        .max_sum = 6.2f,
        .min_sum = -6.2f,

        .max_out = 100.0f,
        .min_out = -100.0f,

        .k_neuro = 1.0f,
        .n_p = 0.001f,
        .n_i = 0.0008f,
        .n_d = 0.001f,

        .k_max = 50.0f,
        .k_min = -50.0f,
    },

    .speed_pid = {
        .kp = 0.02f,
        .ki = 0.0085f,
        .kd = 0.0f,

        .max_sum = 120.0f,
        .min_sum = -120.0f,

        .max_out = 1.8f,
        .min_out = -1.8f,

        .k_neuro = 1.0f,
        .n_p = 0.00005f,
        .n_i = 0.00000f,
        .n_d = 0.00000f,

        .k_max = 0.15f,
        .k_min = 0.00005f,
    },

    .id_pid = {
        .kp = 0.44f,
        .ki = 0.07f,
        .kd = 0.0f,

        .max_sum = 8.0f,
        .min_sum = -8.0f,

        .max_out = 0.9f,
        .min_out = -0.9f,
    },

    .iq_pid = {
        .kp = 0.44f,
        .ki = 0.07f,
        .kd = 0.0f,

        .max_sum = 8.0f,
        .min_sum = -8.0f,

        .max_out = 0.9f,
        .min_out = -0.9f,
    },

    .update_angle_p = &mt6826s,
    .update_angle = (float (*)(void *)) &update_angle,
};

YzOSGlobalData OSGlobal;

__attribute__ ((aligned (8))) YzStack TestStack;
YzThreadStartParams TestTaskStartParams;


#define PI 3.1415926f
#define RAW_RAD_K 0.00019174f
#define ANGLE_RAW_FULL 32768
#define ANGLE_RAW_FULL_2 16384
float update_angle(YZ_PWM_FOC *args)
{
    yzl_mt6826_get_angle(&mt6826s);

    while (!yzl_mt6826_crc(&mt6826s))
    {
        yzl_mt6826_get_angle(&mt6826s);
    }

    foc.cur_raw_rad = mt6826s.ANGLE_DATA;
    foc.last_real_raw_rad = foc.current_real_raw_rad;
    foc.current_real_raw_rad = foc.cur_raw_rad-foc.zero_raw_rad;//要转换到0 - 360
    if (foc.current_real_raw_rad < 0)
    {
        foc.current_real_raw_rad += ANGLE_RAW_FULL;
    }

    foc.current_sector_rad = (float)(foc.current_real_raw_rad*foc.PolarNum)*RAW_RAD_K;
    foc.current_sector_rad_q24 = _IQ24(foc.current_sector_rad);

    foc.current_speed_radi = (foc.current_real_raw_rad-foc.last_real_raw_rad);
    while (foc.current_speed_radi < -ANGLE_RAW_FULL_2 || foc.current_speed_radi > ANGLE_RAW_FULL_2)
    {
        if (foc.current_speed_radi < -ANGLE_RAW_FULL_2)
        {
            foc.current_speed_radi += ANGLE_RAW_FULL;
        }else if (foc.current_speed_radi > ANGLE_RAW_FULL_2)
        {
            foc.current_speed_radi -= ANGLE_RAW_FULL;
        }
    }

    foc.position_err_rad =  foc.current_real_raw_rad - foc.position_offset_rad;
    while (foc.position_err_rad < -ANGLE_RAW_FULL_2 || foc.position_err_rad > ANGLE_RAW_FULL_2)
    {
        if (foc.position_err_rad < -ANGLE_RAW_FULL_2)
        {
            foc.position_err_rad += ANGLE_RAW_FULL;
        }else if (foc.position_err_rad > ANGLE_RAW_FULL_2)
        {
            foc.position_err_rad -= ANGLE_RAW_FULL;
        }
    }

    int time_n = DL_TimerA_getTimerCount(TIMER_50US_INST);
    foc.AngleSampleRate= time_n - foc.SpeedUpdateTime;
    if (foc.AngleSampleRate<0)
    {
        foc.AngleSampleRate += TIMER_50US_INST_LOAD_VALUE;
    }
    foc.current_speed_rad = ((float)foc.current_speed_radi)*RAW_RAD_K/(((float)foc.AngleSampleRate)*0.00005f);//50us时间基准
    foc.SpeedUpdateTime = time_n;

    return foc.current_real_raw_rad*RAW_RAD_K;
}
int dut=120;
float gz_f_fix=-0.163259f;
int mode = 0;
extern UartConnect uart_connect;

void package_handler(UartConnect *connect)
{
    yz_printf("Package Data:\n");
    for (int i=0;i<connect->stx_index;i++)
    {
        yz_printf("%02x ",connect->rx_buf[i]);
        if (i%8==7)
        {
            yz_printf("\n");
        }
    }
    yz_printf("\n");

    if (connect->stx_index==2)
    {
        foc.position_offset_rad = ((int)connect->rx_buf[0]) << 8 | ((int)connect->rx_buf[1]);
    }
}

YZF_BOOL gCheckDutyAndPeriod = YZF_FALSE;
void CAPTURE_Angle_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CAPTURE_Angle_INST)) {
    case DL_TIMERG_IIDX_CC0_DN:
        gCheckDutyAndPeriod = YZF_TRUE;
        break;
    default:
        break;
    }
}

int32_t GetHostAngle()
{
    if (gCheckDutyAndPeriod== YZF_TRUE)
    {
        gCheckDutyAndPeriod = YZF_FALSE;
        int32_t count = CAPTURE_Angle_INST_LOAD_VALUE - DL_Timer_getCaptureCompareValue(CAPTURE_Angle_INST, DL_TIMER_CC_0_INDEX);
        return count;
    }
    return -1;
}

void AsyncAngle()
{
    DL_Timer_startCounter(CAPTURE_Angle_INST);
}

extern uint16_t adc_u[8];
extern uint16_t adc_v[8];
uint32_t last_log_time = 0;
uint32_t last_iu_time = 0;
uint32_t last_speed_time = 0;
uint32_t last_position_time = 0;
YzPID iu_pid={
    .kp = 0.12f,
    .ki = 0.08f,
    .kd = 0.0f,

    .max_sum = 8.0f,
    .min_sum = -8.0f,

    .max_out = 0.9f,
    .min_out = -0.9f,
};
YzPID speed_pid={
    .kp = 0.45f,
    .ki = 0.0f,
    .kd = 0.0f,

    .max_sum = 12.0f,
    .min_sum = -12.0f,

    .max_out = 1.2f,
    .min_out = -1.2f,
};
YzPID position_pid={
    .kp = 16.5f,
    .ki = 0.5f,
    .kd = 0.0f,

    .max_sum = 3.0f,
    .min_sum = -3.0f,

    .max_out = 12.0f,
    .min_out = -12.0f,
};
YZF_RESULT testTask(void *args){
    int i = 300000;//angle check
    foc.update_angle(&foc);
    yzl_pid_default_init(&iu_pid,0.0f);
    yzl_pid_default_init(&speed_pid,0.0f);
    yzl_pid_default_init(&position_pid,0.0f);
    yz_time_block_delay_tick(1000);
    while(1){
        if (mode)
        {
            switch (mode)
            {
                case 1:
                    //Stop Status...
                    break;
                case 2:
                    //Fixed GyroZ
                    while (i)
                    {
                        yz_time_block_delay_tick(2);
                        yzl_qmi8658_update_data(&qmi8658);
                        gz_f_fix += qmi8658.gz_f;
                        i--;
                        if (i%100==0)
                        {
                            yz_printf("%d/300000\n",300000-i);
                        }
                    }
                    gz_f_fix /= 300000.0f;
                    yz_printf("gz_f_fix:%f\n",gz_f_fix);
                    mode = 0;
                    break;
                default:
                    mode = 1;
                    break;
            }
        }else
        {
            // yzl_foc_update(&foc);
            foc.iu = -((((float)adc_u[0])/4095.0f - 0.5f) * 3.3f);
            foc.iv = -((((float)adc_v[0])/4095.0f - 0.5f) * 3.3f);

            if (last_position_time + 9 <yz_time_get()->SYStemOSTime)
            {
                last_position_time = yz_time_get()->SYStemOSTime;
                yzl_pid_default_pos(&position_pid,foc.position_err_rad*RAW_RAD_K);
                speed_pid.SetValue = position_pid.out;
            }
            if (last_speed_time + 3 < yz_time_get()->SYStemOSTime)
            {
                last_speed_time = yz_time_get()->SYStemOSTime;
                yzl_pid_default_pos(&speed_pid,foc.current_speed_rad);
                iu_pid.SetValue = speed_pid.out;
            }
            if (last_iu_time+1 < yz_time_get()->SYStemOSTime)
            {
                last_iu_time = yz_time_get()->SYStemOSTime;
                foc.update_angle(&foc);
                yzl_pid_default_pos(&iu_pid,foc.iu);
            }

            if (-iu_pid.out < 0)
            {
                __disable_irq();
                DL_Timer_setCaptureCompareValue(foc.pwm_control1, (foc.MaxDuty-foc.MinDuty)*fabs(iu_pid.out)+foc.MinDuty,foc.ch1);
                DL_Timer_setCaptureCompareValue(foc.pwm_control2, (foc.MaxDuty-foc.MinDuty)*0.0f+foc.MinDuty,foc.ch2);
                DL_Timer_setCaptureCompareValue(foc.pwm_control3, (foc.MaxDuty-foc.MinDuty)*0.0f+foc.MinDuty,foc.ch3);
                __enable_irq();
            }else
            {
                __disable_irq();
                DL_Timer_setCaptureCompareValue(foc.pwm_control1, (foc.MaxDuty-foc.MinDuty)*0.0f+foc.MinDuty,foc.ch1);
                DL_Timer_setCaptureCompareValue(foc.pwm_control2, (foc.MaxDuty-foc.MinDuty)*0.0f+foc.MinDuty,foc.ch2);
                DL_Timer_setCaptureCompareValue(foc.pwm_control3, (foc.MaxDuty-foc.MinDuty)*fabs(iu_pid.out)+foc.MinDuty,foc.ch3);
                __enable_irq();
            }


            if (last_log_time + 20 < yz_time_get()->SYStemOSTime)
            {
                last_log_time = yz_time_get()->SYStemOSTime;
                //yz_printf("%.3f,%.3f,%.3f,%.3f\n",iu_pid.ActualValue,speed_pid.ActualValue,position_pid.ActualValue,position_pid.SetValue);
            }
        }
        package_check(&uart_connect);
    }
    return YZF_OK;
}

_iq24 x, y, z;

int user_main(){
    yzl_mem_init(&yzlMem);
    yz_time_block_init();   //时钟模块初始化

    x = _IQ16(1.5);
    y = _IQ16(3.2);

    z = _IQ24mpy(x, y);
    yz_info("Enter Init\n");
    int init_flag = 0;
    yz_system_thread_init(&TestStack,&TestTaskStartParams,testTask,10,YZF_NULL_PTR);

SYSTEM_CHECK:
    if (yzl_qmi8658_check_id(&qmi8658) == YZF_OK)
    {
        if (yzl_qmi8658_check_revision_id(&qmi8658) == YZF_OK)
        {
            yz_info("QMI8658 Check OK\n");
            yzl_qmi8658_reset(&qmi8658);
            yz_time_block_delay_tick(3000);
            yzl_qmi8658_ctrl_pull(&qmi8658);
            yz_time_block_delay_tick(3);
            yzl_qmi8658_sync_conf(&qmi8658);
            yz_time_block_delay_tick(3);
            yzL_qmi8658_ctrl_push(&qmi8658);
            yz_time_block_delay_tick(3);
            yzl_qmi8658_ctrl_pull(&qmi8658);
            init_flag++;
        }else
        {
            yz_error("QMI8658 VID Check Error %d\n",qmi8658.revision_id);
        }
    }
    else
    {
        yz_error("QMI8658 ID Check Error %d\n",qmi8658.user_id);
    }

    yzl_mt6826_set_user_id(&mt6826s);

    if (yzl_mt6826_get_user_id(&mt6826s) == mt6826s.user_id)
    {
        yz_info("MT6826 Check OK\n");
        init_flag++;
    }else
    {
        yz_error("MT6826 ID Check Error\n");
    }

    if (init_flag == 2)
    {
        yzl_foc_init(&foc);
        uart_connect.package_handler = (void (*)(void *))package_handler;
        yz_system_start_scheduler();
    }else
    {
        while (1)
        {
            yz_time_block_delay_tick(3000);
            yz_error("Init Error\n");
            goto SYSTEM_CHECK;
        }
    }

    return 0;
}