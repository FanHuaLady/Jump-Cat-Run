#include "balance_imu_if.h"

#include <math.h>
#include <stddef.h>

#if BALANCE_IMU_BACKEND_JY61P
#include "bsp_jy61p.h"
#endif

#if BALANCE_IMU_BACKEND_BMI088
#include "balance_bmi088_service.h"
#include "bsp_bmi088.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)

namespace
{
    static bool g_balance_imu_if_inited = false;

    static inline float DegToRad(float deg)
    {
        return deg * DEG_TO_RAD;
    }
}

void BalanceImuIf_Init(void)
{
#if BALANCE_IMU_BACKEND_JY61P
    BSP_JY61P.Init(&huart10);
#endif

#if BALANCE_IMU_BACKEND_BMI088
    BalanceBmi088Service_Init();
#endif

    g_balance_imu_if_inited = true;
}

void BalanceImuIf_Update(BalanceImuData* imu)
{
    if (imu == nullptr)
    {
        return;
    }

    if (!g_balance_imu_if_inited)
    {
        BalanceImuIf_Init();
    }

#if BALANCE_IMU_BACKEND_JY61P
    // JY61P:
    // angle -> deg
    // gyro  -> deg/s
    // accel -> m/s^2

    imu->roll  = DegToRad(BSP_JY61P.GetRoll());
    imu->pitch = DegToRad(BSP_JY61P.GetPitch());
    imu->yaw   = DegToRad(BSP_JY61P.GetYaw());

    imu->roll_dot  = DegToRad(BSP_JY61P.GetGyroX());
    imu->pitch_dot = DegToRad(BSP_JY61P.GetGyroY());
    imu->yaw_dot   = DegToRad(BSP_JY61P.GetGyroZ());

    imu->ax = BSP_JY61P.GetAccelX();
    imu->ay = BSP_JY61P.GetAccelY();
    imu->az = BSP_JY61P.GetAccelZ();

    imu->online = true;
#endif

#if BALANCE_IMU_BACKEND_BMI088
    // BMI088:
    // Euler angle order in BSP_BMI088 is Yaw-Pitch-Roll
    // Gyro_Body / Accel_Body are body-frame values

    auto euler = BSP_BMI088.Get_Euler_Angle();
    auto gyro  = BSP_BMI088.Get_Gyro_Body();
    auto accel = BSP_BMI088.Get_Accel_Body();

    imu->roll  = euler[2][0];
    imu->pitch = euler[1][0];
    imu->yaw   = euler[0][0];

    imu->roll_dot  = gyro[0][0];
    imu->pitch_dot = gyro[1][0];
    imu->yaw_dot   = gyro[2][0];

    imu->ax = accel[0][0];
    imu->ay = accel[1][0];
    imu->az = accel[2][0];

    imu->online = true;
#endif
}