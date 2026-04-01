#include "balance_test_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "dvc_motor_dji.h"

Class_Motor_DJI_C620 motor_test_dji;

static TaskHandle_t xBalanceTestTaskHandle = NULL;

static bool g_balance_test_enable = false;
static bool g_balance_test_inited = false;

static float g_target_vel = 0.0f;
static uint32_t g_alive_tick = 0;

static constexpr float kBalanceTestLoopDt = 0.002f;
static constexpr float kBalanceTestOmegaKp = 0.30f;
static constexpr float kBalanceTestOmegaKi = 0.00f;
static constexpr float kBalanceTestOmegaKd = 0.00f;
static constexpr float kBalanceTestTorqueMax = 3.0f;

static inline float Balance_Test_Clamp(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static void Balance_Test_CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    switch (Header.Identifier)
    {
    case 0x201:
        motor_test_dji.CAN_RxCpltCallback();
        break;

    default:
        break;
    }
}

void Balance_Test_Motor_Init(void)
{
    if (g_balance_test_inited)
    {
        return;
    }

    CAN_Init(&hfdcan1, Balance_Test_CAN1_Callback);

    motor_test_dji.Init(&hfdcan1,
                        Motor_DJI_ID_0x201,
                        Motor_DJI_Control_Method_OMEGA,
                        19.0f);

    // Without PID_Omega gains, omega mode keeps output at 0.
    motor_test_dji.PID_Omega.Init(kBalanceTestOmegaKp,
                                  kBalanceTestOmegaKi,
                                  kBalanceTestOmegaKd,
                                  0.0f,
                                  0.0f,
                                  kBalanceTestTorqueMax,
                                  kBalanceTestLoopDt);

    motor_test_dji.Set_Target_Omega(0.0f);
    motor_test_dji.Set_Target_Torque(0.0f);
    motor_test_dji.Set_Feedforward_Omega(0.0f);
    motor_test_dji.Set_Feedforward_Torque(0.0f);

    g_balance_test_inited = true;
}

static void Balance_Test_Motor_Apply(void)
{
    if (!g_balance_test_enable)
    {
        motor_test_dji.Set_Target_Omega(0.0f);
        motor_test_dji.Set_Target_Torque(0.0f);
        motor_test_dji.Set_Feedforward_Omega(0.0f);
        motor_test_dji.Set_Feedforward_Torque(0.0f);
        return;
    }

    const float cmd_vel = Balance_Test_Clamp(g_target_vel, -30.0f, 30.0f);

    motor_test_dji.Set_Target_Omega(cmd_vel);
    motor_test_dji.Set_Feedforward_Omega(0.0f);
    motor_test_dji.Set_Feedforward_Torque(0.0f);

    motor_test_dji.TIM_Calculate_PeriodElapsedCallback();
}

void Balance_Test_Motor_Start(float target_vel)
{
    if (!g_balance_test_inited)
    {
        Balance_Test_Motor_Init();
    }

    g_target_vel = target_vel;
    g_balance_test_enable = true;
}

void Balance_Test_Motor_Stop(void)
{
    g_balance_test_enable = false;

    motor_test_dji.Set_Target_Omega(0.0f);
    motor_test_dji.Set_Target_Torque(0.0f);
    motor_test_dji.Set_Feedforward_Omega(0.0f);
    motor_test_dji.Set_Feedforward_Torque(0.0f);

    motor_test_dji.TIM_Calculate_PeriodElapsedCallback();
}

void Balance_Test_Motor_SetVelocity(float target_vel)
{
    g_target_vel = target_vel;
}

static void Balance_Test_DJI_SendCommand(void)
{
    extern uint8_t CAN1_0x200_Tx_Data[8];

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = 0x200;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CAN1_0x200_Tx_Data);
}

static void vBalanceTestTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));

    Balance_Test_Motor_Init();
    Balance_Test_Motor_Start(10.0f);

    while (1)
    {
        Balance_Test_Motor_Apply();
        Balance_Test_DJI_SendCommand();

        g_alive_tick += 2;
        if (g_alive_tick >= 100)
        {
            g_alive_tick = 0;
            motor_test_dji.TIM_100ms_Alive_PeriodElapsedCallback();
        }

        volatile float now_angle = motor_test_dji.Get_Now_Angle();
        volatile float now_omega = motor_test_dji.Get_Now_Omega();
        volatile float now_torque = motor_test_dji.Get_Now_Torque();
        volatile float now_temp = motor_test_dji.Get_Now_Temperature();

        (void)now_angle;
        (void)now_omega;
        (void)now_torque;
        (void)now_temp;

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void Balance_Test_Task_Create(void)
{
    BaseType_t ret = xTaskCreate(vBalanceTestTask,
                                 "vBalanceTestTask",
                                 256,
                                 NULL,
                                 1,
                                 &xBalanceTestTaskHandle);

    if (ret != pdPASS)
    {
        while (1)
        {
        }
    }
}
