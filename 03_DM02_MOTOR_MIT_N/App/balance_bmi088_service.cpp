#include "balance_bmi088_service.h"
#include "bsp_bmi088.h"

namespace
{
    static bool g_bmi088_inited = false;
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
    (void)Tx_Buffer;
    (void)Rx_Buffer;
    (void)Tx_Length;
    (void)Rx_Length;

    if ((SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port &&
         SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin) ||
        (SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port &&
         SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin))
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!g_bmi088_inited)
    {
        return;
    }
   
    if (htim->Instance == TIM4)
    {
        BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        static int mod128 = 0;
        mod128++;
        if (mod128 == 128)
        {
            mod128 = 0;
            BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
        }
    }
    else if (htim->Instance == TIM8)
    {
        BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
    }
}
*/

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

void BalanceBmi088Service_Init(void)
{
    if (g_bmi088_inited)
    {
        return;
    }
    
    g_bmi088_inited = true;
    SPI_Init(&hspi2, SPI2_Callback);
    BSP_BMI088.Init();
}
