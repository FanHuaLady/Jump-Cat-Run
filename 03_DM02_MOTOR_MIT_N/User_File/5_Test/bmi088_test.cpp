#define BMI088_TEST

#ifdef BMI088_TEST

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
#include "bmi088_test.h"
#include <stdio.h>

int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

bool init_finished = false;

// ----------------------------------------------------------------------- // 关于陀螺仪的
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }

    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

void Task125us_Callback()
{
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

void Task10us_Callback()
{
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}

void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    if (SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin || SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

// -------------------------------------------------------------------------- // 
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

        /*
        float yaw = BSP_BMI088.Get_Euler_Angle()[0][0] / BASIC_MATH_DEG_TO_RAD;
        float pitch = BSP_BMI088.Get_Euler_Angle()[1][0] / BASIC_MATH_DEG_TO_RAD;
        float roll = BSP_BMI088.Get_Euler_Angle()[2][0] / BASIC_MATH_DEG_TO_RAD;

        char buffer[128];
        int yaw_int = (int)(yaw * 100);
        int pitch_int = (int)(pitch * 100);
        int roll_int = (int)(roll * 100);
        int len = snprintf(buffer, sizeof(buffer), "yaw:%d.%02d pitch:%d.%02d roll:%d.%02d\r\n",
        yaw_int/100, abs(yaw_int%100),
        pitch_int/100, abs(pitch_int%100),
        roll_int/100, abs(roll_int%100));
        
        HAL_UART_Transmit(&huart7, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        */
    }

    static int mod128 = 0;
    mod128++;
    if (mod128 == 128)
    {
        mod128 = 0;

        BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();     // 每128ms，调一下温度

        // ----- 打印温度 -----
        float temp = BSP_BMI088.BMI088_Accel.Get_Now_Temperature();
        if (!Basic_Math_Is_Invalid_Float(temp))
        {
            int temp_int = (int)(temp * 100);
            char temp_buf[64];
            int len = snprintf(temp_buf, sizeof(temp_buf), "Temperature: %d.%02d C\r\n",
                            temp_int / 100, abs(temp_int % 100));
            HAL_UART_Transmit(&huart7, (uint8_t*)temp_buf, len, HAL_MAX_DELAY);
        }
        else
        {
            const char* err_msg = "Temperature: Invalid\r\n";
            HAL_UART_Transmit(&huart7, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
        }

        // ----- 打印电压 -----
        float temp_v = BSP_Power.Get_Power_Voltage();
        if (!Basic_Math_Is_Invalid_Float(temp_v))
        {
            int voltage_int = (int)(temp_v * 100);
            char volt_buf[64];
            int len = snprintf(volt_buf, sizeof(volt_buf), "Voltage: %d.%02d V\r\n",
                            voltage_int / 100, abs(voltage_int % 100));
            HAL_UART_Transmit(&huart7, (uint8_t*)volt_buf, len, HAL_MAX_DELAY);
        }
        else
        {
            const char* err_msg = "Voltage: Invalid\r\n";
            HAL_UART_Transmit(&huart7, (uint8_t*)err_msg, strlen(err_msg), HAL_MAX_DELAY);
        }
    }
}

void Bmi088_test()
{
    SYS_Timestamp.Init(&htim5);                                     // 初始化时间戳

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim7);                                  // 开启定时器中断
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim8);

    SPI_Init(&hspi6, nullptr);                                      // WS2812的SPI
    BSP_WS2812.Init(0, 0, 0);                                       // 初始化彩灯
    SPI_Init(&hspi2, SPI2_Callback);                                // 陀螺仪的SPI
    BSP_BMI088.Init();

    ADC_Init(&hadc1,1);                                             // !!!
    BSP_Power.Init();                                               // 初始化连接电源的ADC

    SPI_Init(&hspi6, nullptr);                                      // WS2812的SPI

    init_finished = true;                                           // 标记初始化完成
}

void Bmi088_test_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

#endif
