// #define CONTROL_TEST

#ifdef CONTROL_TEST

#include "control_test.h"

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

// 新增：遥控器 BSP 头文件
#include "bsp_control.h"
#include <stdio.h>

// 新增：声明外部全局对象（在 bsp_control.cpp 中定义）
extern Class_Control Control;

// 新增：声明外部串口句柄（用于打印）
extern UART_HandleTypeDef huart7;

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

    Control.TIM_1ms_Process_PeriodElapsedCallback();
}

void Control_test()
{
    SYS_Timestamp.Init(&htim5);                                     // 初始化时间戳
    SPI_Init(&hspi6, nullptr);                                      // WS2812的SPI
    BSP_WS2812.Init(0, 0, 0);                                       // 初始化彩灯

    Control.Init(&huart5, 100);

    HAL_TIM_Base_Start_IT(&htim7);                                  // 开启定时器中断
    HAL_TIM_Base_Start_IT(&htim5);

    init_finished = true;                                           // 标记初始化完成
}

void Control_test_Loop()
{
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();

    if (!Control.IsOffline() && (now - last_print > 100)) 
    {
        last_print = now;

        // 获取原始数据
        const auto &raw = Control.GetData();

        // 打印开关值、四个摇杆通道值、以及两个旋钮通道值
        char debug[192];  // 增大缓冲区
        int dlen = snprintf(debug, sizeof(debug), 
            "s=[%d,%d,%d,%d] ch=[%d,%d,%d,%d] vr=[%d,%d]\r\n",
            raw.rc.s[0], raw.rc.s[1], raw.rc.s[2], raw.rc.s[3],
            raw.rc.ch[0], raw.rc.ch[1], raw.rc.ch[2], raw.rc.ch[3],
            raw.rc.ch[4], raw.rc.ch[5]);  // 旋钮 VrA 和 VrB
        HAL_UART_Transmit(&huart7, (uint8_t*)debug, dlen, HAL_MAX_DELAY);
    }

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
