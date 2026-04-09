#pragma once

#include "balance_types.h"

// =====================================================
// IMU backend selection
// 只能选一个为 1
// =====================================================
#define BALANCE_IMU_BACKEND_JY61P   0
#define BALANCE_IMU_BACKEND_BMI088  1

#if (BALANCE_IMU_BACKEND_JY61P + BALANCE_IMU_BACKEND_BMI088) != 1
#error "Exactly one IMU backend must be enabled."
#endif

void BalanceImuIf_Init(void);
void BalanceImuIf_Update(BalanceImuData* imu);