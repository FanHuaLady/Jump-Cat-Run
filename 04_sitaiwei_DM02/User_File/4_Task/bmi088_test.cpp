#define BMI088_TEST

#ifdef BMI088_TEST

#include "dvc_vofa.h"
#include "bsp_w25q64jv.h"
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"
#include "bsp_power.h"
#include "bsp_key.h"
#include "alg_filter_kalman.h"
#include "alg_matrix.h"
#include "drv_wdg.h"
#include "sys_timestamp.h"
#include "dvc_serialplot.h"
#include "bmi088_test.h"
#include "balance_bmi088_service.h"
#include "bsp_bmi088.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern bool init_finished;

static void Task1ms_Callback(void)
{
    static int mod128 = 0;
    mod128++;
    if (mod128 == 128)
    {
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;

        mod128 = 0;
        yaw = BSP_BMI088.Get_Euler_Angle()[0][0] / BASIC_MATH_DEG_TO_RAD;
        pitch = BSP_BMI088.Get_Euler_Angle()[1][0] / BASIC_MATH_DEG_TO_RAD;
        roll = BSP_BMI088.Get_Euler_Angle()[2][0] / BASIC_MATH_DEG_TO_RAD;

        if (!Basic_Math_Is_Invalid_Float(yaw) &&
            !Basic_Math_Is_Invalid_Float(pitch) &&
            !Basic_Math_Is_Invalid_Float(roll))
        {
            char buffer[128];
            int yaw_int = (int) (yaw * 100);
            int pitch_int = (int) (pitch * 100);
            int roll_int = (int) (roll * 100);
            int len = snprintf(buffer,
                               sizeof(buffer),
                               "yaw:%d.%02d pitch:%d.%02d roll:%d.%02d\r\n",
                               yaw_int / 100,
                               abs(yaw_int % 100),
                               pitch_int / 100,
                               abs(pitch_int % 100),
                               roll_int / 100,
                               abs(roll_int % 100));
            HAL_UART_Transmit(&huart7, (uint8_t *) buffer, len, HAL_MAX_DELAY);
        }
        else
        {
            const char *err_msg = "Euler angle: Invalid\r\n";
            HAL_UART_Transmit(&huart7, (uint8_t *) err_msg, strlen(err_msg), HAL_MAX_DELAY);
        }
    }
}

void BalanceBmi088Service_Timer1msUserCallback(void)
{
    if (!init_finished)
    {
        return;
    }

    Task1ms_Callback();
}

void Bmi088_test()
{
    BalanceBmi088Service_Init();

    ADC_Init(&hadc1, 1);
    BSP_Power.Init();

    init_finished = true;
}

#endif
