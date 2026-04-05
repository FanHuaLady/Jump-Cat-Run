/**
 * @file alg_pid.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID初始化
 *
 * @param __K_P P值
 * @param __K_I I值
 * @param __K_D D值
 * @param __K_F 前馈
 * @param __I_Out_Max 积分限幅
 * @param __Out_Max 输出限幅
 * @param __D_T 时间片长度
 * @param __Dead_Zone 死区误差阈值
 * @param __I_Variable_Speed_A 变速积分误差阈值A
 * @param __I_Variable_Speed_B 变速积分误差阈值B
 * @param __I_Separate_Threshold 积分分离误差阈值
 * @param __D_First 是否开启微分先行
 */
void Class_PID::Init(const float &__K_P, const float &__K_I, const float &__K_D, const float &__K_F, const float &__I_Out_Max, const float &__Out_Max, const float &__D_T, const float &__Dead_Zone, const float &__I_Variable_Speed_A, const float &__I_Variable_Speed_B, const float &__I_Separate_Threshold, const Enum_PID_D_First &__D_First)
{
    K_P = __K_P;
    K_I = __K_I;
    K_D = __K_D;
    K_F = __K_F;
    I_Out_Max = __I_Out_Max;
    Out_Max = __Out_Max;
    D_T = __D_T;
    Dead_Zone = __Dead_Zone;
    I_Variable_Speed_A = __I_Variable_Speed_A;
    I_Variable_Speed_B = __I_Variable_Speed_B;
    I_Separate_Threshold = __I_Separate_Threshold;
    D_First = __D_First;
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 */
void Class_PID::TIM_Calculate_PeriodElapsedCallback()
{
    float p_out = 0.0f;                                             // P输出
    float i_out = 0.0f;                                             // I输出
    float d_out = 0.0f;                                             // D输出
    float f_out = 0.0f;                                             // F输出
    float error;                                                    // 误差
    float abs_error;                                                // 绝对值误差
    float speed_ratio;                                              // 线性变速积分

    error = Target - Now;                                           // 获取误差值
    abs_error = Basic_Math_Abs(error);                              // 获得误差绝对值

    // 作用就是让PID系统忽略误差
    if (abs_error < Dead_Zone)                                      // 判断死区
    {
        Target = Now;                                               // 视为已经达到目标
        error = 0.0f;                                               // 设置误差为0
        abs_error = 0.0f;                                           // 设置绝对误差为0
    }
    else if (error > 0.0f && abs_error > Dead_Zone)
    {
        error -= Dead_Zone;                                         // 在误差超出死区后能够平滑过渡
    }
    else if (error < 0.0f && abs_error > Dead_Zone)
    {
        error += Dead_Zone;
    }
    p_out = K_P * error;                                            // 计算p项


    // 当用户没有启用变速积分功能时，将积分速度系数设为固定值 1.0
    // “非变速积分”指的是传统的积分方式，即积分项的计算速度（积分系数）是恒定不变的
    // 在代码中表现为 speed_ratio = 1.0f，也就是说积分累加的速率一直保持不变
    if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f)   // 计算i项
    {
        speed_ratio = 1.0f;                                         // 非变速积分
    }
    else
    {
        // 变速积分
        if (abs_error <= I_Variable_Speed_A)
        {
            speed_ratio = 1.0f;
        }
        else if (I_Variable_Speed_A < abs_error && abs_error < I_Variable_Speed_B)
        {
            speed_ratio = (I_Variable_Speed_B - abs_error) / (I_Variable_Speed_B - I_Variable_Speed_A);
        }
        else if (abs_error >= I_Variable_Speed_B)
        {
            speed_ratio = 0.0f;
        }
    }
    if (I_Out_Max != 0.0f)                                          // 积分限幅
    {
        Basic_Math_Constrain(&Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
    }

    if (I_Separate_Threshold == 0.0f)                               // 积分分离阈值，需为正数, 0为不限制
    {
        // 没有积分分离
        // DT是PID计时器周期, s
        Integral_Error += speed_ratio * D_T * error;                // 误差累积
        i_out = K_I * Integral_Error;                               // I的输出
    }
    else
    {
        // 有积分分离
        if (abs_error < I_Separate_Threshold)
        {
            // 不在积分分离区间上
            Integral_Error += speed_ratio * D_T * error;
            i_out = K_I * Integral_Error;
        }
        else
        {
            // 在积分分离区间上
            Integral_Error = 0.0f;
            i_out = 0.0f;
        }
    }

    // 计算d项

    if (D_First == PID_D_First_DISABLE)
    {
        // 没有微分先行
        d_out = K_D * (error - Pre_Error) / D_T;
    }
    else
    {
        // 微分先行使能
        d_out = -K_D * (Now - Pre_Now) / D_T;
    }

    // 计算前馈

    f_out = K_F * (Target - Pre_Target);

    Out = p_out + i_out + d_out + f_out;                            // 计算输出

    // 输出限幅
    if (Out_Max != 0.0f)
    {
        Basic_Math_Constrain(&Out, -Out_Max, Out_Max);
    }

    // 善后工作
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = Out;
    Pre_Error = error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/