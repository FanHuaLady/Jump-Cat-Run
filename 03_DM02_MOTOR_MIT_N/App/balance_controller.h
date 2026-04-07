#pragma once

#include "balance_types.h"

// 初始化控制器
void BalanceController_Init(BalanceRobot* robot);

// 设置参考值（第一版先写死）
void BalanceController_SetRef(BalanceRobot* robot);

// 腿长控制（第一版只做腿长P/PD）
void BalanceController_LegLength(BalanceRobot* robot);

// 原来的虚拟杆角度控制
// 现在保留接口，但主流程里先不调用
void BalanceController_LegAngle(BalanceRobot* robot);

// 新增：LQR 平衡控制
// 输出：wheel_t + rod_tp
void BalanceController_LqrBalance(BalanceRobot* robot);

// 输出分配（rod_f + rod_tp -> joint_t -> motor_cmd）
void BalanceController_Output(BalanceRobot* robot);

// 急停/清零
void BalanceController_Stop(BalanceRobot* robot);