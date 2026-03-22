// #define MOTOR_TEST

#ifdef MOTOR_TEST

#include "dvc_motor_dji.h"
#include "bsp_bmi088.h"
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
#include "motor_test.h"

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

Class_Motor_DM_Normal motor_dm;

bool init_finished = false;

void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    case (0x000): // MasterID
        motor_dm.CAN_RxCpltCallback();
        break;
    }
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

    // 这里要说明，电机的一些状态并不是自己回复得来
    // 而是根据一些信息判断出来
    // 这里的函数会每隔100ms进行判断
    // 这里的motor_dm.TIM_Send_PeriodElapsedCallback()
    // 如果发现电机一直没有回复消息
    // 就会Motor_DM_Status = Motor_DM_Status_DISABLE;
    // 并且再发送一次使能帧
    static int mod100 = 0;
    mod100++;
    if (mod100 == 100)
    {
        mod100 = 0;
        motor_dm.TIM_100ms_Alive_PeriodElapsedCallback();
    }

    static uint32_t tick_dm = 0;
    tick_dm++;
    float target_angle = 1.0f * sinf(2.0f * 3.14159f * tick_dm / 2000.0f);
    motor_dm.Set_Control_Angle(target_angle);
    motor_dm.Set_K_P(0.0f);
    motor_dm.Set_K_D(1.0f);
    motor_dm.Set_Control_Omega(5.0f);
    motor_dm.Set_Control_Torque(0.0f);

    // 这个函数会对各种数据进行限幅
    // 并且在断线时自动进行使能
    // 在错误时自动清除错误并使能
    motor_dm.TIM_Send_PeriodElapsedCallback();
}

void Motor_test()
{
    SYS_Timestamp.Init(&htim5);                                     // 初始化时间戳
    CAN_Init(&hfdcan1, CAN1_Callback);                              // 电机的CAN
    SPI_Init(&hspi6, nullptr);                                      // WS2812的SPI
    BSP_WS2812.Init(0, 0, 0);                                       // 初始化彩灯

    // 初始化电机
    motor_dm.Init(&hfdcan1, 0x00, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f, 10.261194f);
    motor_dm.CAN_Send_Enter();                                      // 使能电机

    HAL_TIM_Base_Start_IT(&htim7);                                  // 开启定时器中断

    init_finished = true;                                           // 标记初始化完成
}

void Motor_test_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }
    if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
}

#endif