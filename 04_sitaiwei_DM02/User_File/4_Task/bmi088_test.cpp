#define BMI088_TEST

#ifdef BMI088_TEST

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_power.h"
#include "sys_timestamp.h"
#include "bmi088_test.h"
#include "balance_bmi088_service.h"
#include "bsp_bmi088.h"
#include "balance_tool.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern bool init_finished;
static TaskHandle_t xImuPrintTaskHandle = NULL;

static void vImuPrintTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1200));

    for (;;)
    {
        auto euler = BSP_BMI088.Get_Euler_Angle();

        float roll  = euler[2][0];
        float pitch = euler[1][0];
        float yaw   = euler[0][0];   
                                    
        BalanceTool_PrintFloat4Line("pitch",
                                    pitch,
                                    "roll",
                                    roll);
        BalanceTool_PrintFloat4Line("yaw",
                                    yaw,
                                    "safe",
                                    1.0f);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void Bmi088_test()
{
    BalanceBmi088Service_Init();

    ADC_Init(&hadc1, 1);
    BSP_Power.Init();
    /*
    BaseType_t ret;
    
    ret = xTaskCreate(vImuPrintTask,
                      "vImuPrintTask",
                      512,
                      NULL,
                      1,
                      &xImuPrintTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }
    */
    init_finished = true;
}

#endif
