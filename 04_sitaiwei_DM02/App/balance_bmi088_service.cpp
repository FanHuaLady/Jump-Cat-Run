#include "balance_bmi088_service.h"

#include "bsp_bmi088.h"
#include "sys_timestamp.h"

namespace
{
    static bool g_bmi088_inited = false;
    static uint16_t g_tim7_divider = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!g_bmi088_inited)
    {
        return;
    }

    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    (void) Tx_Buffer;
    (void) Rx_Buffer;
    (void) Tx_Length;
    (void) Rx_Length;

    if ((SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port &&
         SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin) ||
        (SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port &&
         SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin))
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!g_bmi088_inited)
    {
        return;
    }

    if (htim->Instance == TIM4)
    {
        BalanceBmi088Service_Timer10usCallback();
    }
    else if (htim->Instance == TIM5)
    {
        BalanceBmi088Service_Timer3600sCallback();
    }
    else if (htim->Instance == TIM7)
    {
        g_tim7_divider++;
        if (g_tim7_divider >= 128)
        {
            g_tim7_divider = 0;
            BalanceBmi088Service_Timer128msCallback();
        }
    }
    else if (htim->Instance == TIM8)
    {
        BalanceBmi088Service_Timer125usCallback();
    }
}

void BalanceBmi088Service_Timer10usCallback(void)
{
    if (!g_bmi088_inited)
    {
        return;
    }

    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}

void BalanceBmi088Service_Timer125usCallback(void)
{
    if (!g_bmi088_inited)
    {
        return;
    }

    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

void BalanceBmi088Service_Timer128msCallback(void)
{
    if (!g_bmi088_inited)
    {
        return;
    }

    BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
}

void BalanceBmi088Service_Timer3600sCallback(void)
{
    if (!g_bmi088_inited)
    {
        return;
    }

    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

void BalanceBmi088Service_Init(void)
{
    if (g_bmi088_inited)
    {
        return;
    }

    SYS_Timestamp.Init(&htim5);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    SPI_Init(&hspi2, SPI2_Callback);
    BSP_BMI088.Init();

    g_tim7_divider = 0;
    g_bmi088_inited = true;
}
