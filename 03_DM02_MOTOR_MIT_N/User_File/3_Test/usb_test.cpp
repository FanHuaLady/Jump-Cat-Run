// #define USB_TEST

#ifdef USB_TEST

#include "usb_test.h"
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
#include "gimbal_yaw_pitch_direct.h"
#include "jy61p_test.h"
#include "bsp_jy61p.h"
#include <stdio.h>

int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

bool init_finished = false;

void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

void Task1ms_Callback()
{
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }
        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }
        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        if (red_minus_flag)
        {
            red--;
        }
        else
        {
            red++;
        }
        if (green_minus_flag)
        {
            green--;
        }
        else
        {
            green++;
        }
        if (blue_minus_flag)
        {
            blue--;
        }
        else
        {
            blue++;
        }

        BSP_WS2812.Set_RGB(red, green, blue);
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
    }
}

void USB_RxCallback(uint8_t *Buffer, uint16_t Length)
{
    HAL_UART_Transmit(&huart7, Buffer, Length, HAL_MAX_DELAY);
}

void USB_test()
{
    SYS_Timestamp.Init(&htim5);                                     // 初始化时间戳
    SPI_Init(&hspi6, nullptr);                                      // WS2812的SPI
    BSP_WS2812.Init(0, 0, 0);                                       // 初始化彩灯

    USB_Init(USB_RxCallback);                                       // 初始化USB虚拟串口

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim7);                                  // 开启定时器中断
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim8);

    init_finished = true;                                           // 标记初始化完成
}

void USB_test_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
}

#endif