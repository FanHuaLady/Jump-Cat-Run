#include "balance_controller.h"

#include <stddef.h>
#include <string.h>

#include "balance_config.h"
#include "balance_kinematics.h"
#include "balance_tool.h"

namespace
{
    // =========================
    // 腿长控制参数
    // =========================
    static constexpr float k_leg_len_kp = 4.0f;
    static constexpr float k_leg_len_kd = 0.8f;

    // =========================
    // 虚拟腿角度控制参数
    // 参考角不再写死在这里
    // 只保留控制器增益
    // =========================
    static constexpr float k_leg_ang_kp = 4.0f;
    static constexpr float k_leg_ang_kd = 0.8f;

    // =========================
    // LQR 增益（leg_length = 0.3844）
    // 状态顺序：
    // [theta, theta_dot, x, x_dot, phi, phi_dot]
    //
    // 输入顺序：
    // [T, Tp]
    //
    // T  -> wheel_t
    // Tp -> rod_tp
    // =========================
    static constexpr float k_lqr_K[2][6] =
    {
        { -82.0714f, -11.5743f, -0.2701f, -0.4922f, -134.2395f, -15.1839f },
        {  73.3496f,   9.4385f,  1.9724f,  2.3286f,  137.6689f,  16.7321f }
    };

    static inline float BalanceClamp(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }

    static inline void BalanceClearMotorCmd(BalanceMotorCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->pos = 0.0f;
        cmd->vel = 0.0f;
        cmd->tor = 0.0f;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->enable = false;
    }

    static inline void BalanceClearLegCmd(BalanceLegCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->rod_f = 0.0f;
        cmd->rod_tp = 0.0f;
        cmd->joint_t[0] = 0.0f;
        cmd->joint_t[1] = 0.0f;
        cmd->wheel_t = 0.0f;
    }

    // =========================
    // 模型关节力矩 -> 真实电机力矩
    // 左腿不翻转
    // 右腿翻回真实方向
    // =========================
    static inline float BalanceMapJointTorqueToMotor(bool is_right_leg, float tau_model)
    {
        return is_right_leg ? (-tau_model) : tau_model;
    }
}

void BalanceController_Init(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        BalanceClearLegCmd(&robot->cmd[i]);
    }

    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }
}

// 左右腿目标参考
void BalanceController_SetRef(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 默认站立腿长
    robot->ref.target_leg_length[0] = BALANCE_DEFAULT_LEG_LEN_STAND;
    robot->ref.target_leg_length[1] = BALANCE_DEFAULT_LEG_LEN_STAND;

    // 默认虚拟腿角
    // 这里先沿用你之前写死的参考角
    robot->ref.target_leg_angle[0] = -1.5708f;
    robot->ref.target_leg_angle[1] = -1.39626f;

    // 这些目前先保留为 0，后面如果你要做速度跟踪/转向/横滚控制再接进去
    robot->ref.target_vx = 0.0f;
    robot->ref.target_wz = 0.0f;
    robot->ref.target_roll = 0.0f;
}

void BalanceController_LegLength(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        const float l_ref  = robot->ref.target_leg_length[i];
        const float l_now  = robot->leg[i].rod.l0;
        const float dl_now = robot->leg[i].rod.dl0;

        const float err_l = l_ref - l_now;

        float rod_f = k_leg_len_kp * err_l - k_leg_len_kd * dl_now;

        rod_f = BalanceClamp(rod_f,
                             -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                              BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);

        robot->cmd[i].rod_f = rod_f;
    }
}

void BalanceController_LegAngle(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        const float phi0_now  = robot->leg[i].rod.phi0;
        const float dphi0_now = robot->leg[i].rod.dphi0;

        // 改成从 ref 读取，而不是使用写死常量
        const float phi0_ref = robot->ref.target_leg_angle[i];
        const float err_phi0 = BalanceTool_AngleDiffRad(phi0_ref, phi0_now);

        float rod_tp = k_leg_ang_kp * err_phi0 - k_leg_ang_kd * dphi0_now;

        rod_tp = BalanceClamp(rod_tp,
                              -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                               BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);

        robot->cmd[i].rod_tp = rod_tp;
    }
}

// =========================
// LQR 平衡控制
// 当前先单腿测试：直接使用右腿 theta / theta_dot
// =========================
void BalanceController_LqrBalance(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    const float theta = robot->leg_state[1].theta;
    const float theta_dot = robot->leg_state[1].theta_dot;

    const float x       = robot->body.x;
    const float x_dot   = robot->body.x_dot;
    const float phi     = robot->body.phi;
    const float phi_dot = robot->body.phi_dot;

    float wheel_t =
        (k_lqr_K[0][0] * theta +
          k_lqr_K[0][1] * theta_dot +
          k_lqr_K[0][2] * x +
          k_lqr_K[0][3] * x_dot +
          k_lqr_K[0][4] * phi +
          k_lqr_K[0][5] * phi_dot);

    float rod_tp =
        (k_lqr_K[1][0] * theta +
          k_lqr_K[1][1] * theta_dot +
          k_lqr_K[1][2] * x +
          k_lqr_K[1][3] * x_dot +
          k_lqr_K[1][4] * phi +
          k_lqr_K[1][5] * phi_dot);

    wheel_t = BalanceClamp(wheel_t,
                           -BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT,
                            BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT);

    rod_tp = BalanceClamp(rod_tp,
                          -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                           BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        robot->cmd[i].wheel_t = wheel_t;
        robot->cmd[i].rod_tp  = rod_tp;
    }
}

// 这个函数会利用 rod_f, rod_tp 和 VMC 得到真正的控制力矩
// 然后填充 joint_motor_cmd 和 wheel_motor_cmd
void BalanceController_Output(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 先清空所有输出
    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }

    // =========================
    // 左腿
    // rod_f + rod_tp -> joint_t[2]
    // =========================
    {
        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(robot->leg[0].joint.phi1, robot->leg[0].joint.phi4, J);

        BalanceCalcVmc(robot->cmd[0].rod_f,
                       robot->cmd[0].rod_tp,
                       J,
                       robot->cmd[0].joint_t);

        robot->joint_motor_cmd[BAL_JOINT_L_0].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_0].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_0].tor =
            BalanceClamp(BalanceMapJointTorqueToMotor(false, robot->cmd[0].joint_t[0]),
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                          BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_L_0].kp = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_0].kd = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_0].enable = true;

        robot->joint_motor_cmd[BAL_JOINT_L_1].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_1].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_1].tor =
            BalanceClamp(BalanceMapJointTorqueToMotor(false, robot->cmd[0].joint_t[1]),
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                          BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_L_1].kp = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_1].kd = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_1].enable = true;
    }

    // =========================
    // 右腿
    // 右腿输入已经按“左腿视角”做了翻面
    // 所以这里模型输出回真实右腿电机时，需要再反号
    // =========================
    {
        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(robot->leg[1].joint.phi1, robot->leg[1].joint.phi4, J);

        BalanceCalcVmc(robot->cmd[1].rod_f,
                       robot->cmd[1].rod_tp,
                       J,
                       robot->cmd[1].joint_t);

        robot->joint_motor_cmd[BAL_JOINT_R_0].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_0].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_0].tor =
            BalanceClamp(BalanceMapJointTorqueToMotor(true, robot->cmd[1].joint_t[0]),
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                          BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_R_0].kp = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_0].kd = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_0].enable = true;

        robot->joint_motor_cmd[BAL_JOINT_R_1].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_1].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_1].tor =
            BalanceClamp(BalanceMapJointTorqueToMotor(true, robot->cmd[1].joint_t[1]),
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                          BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_R_1].kp = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_1].kd = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_1].enable = true;
    }

    // =========================
    // 轮子输出
    // 第一版：左右轮同向同值
    // =========================
    const float wheel_t =
        BalanceClamp(0.5f * (robot->cmd[0].wheel_t + robot->cmd[1].wheel_t),
                     -BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT,
                      BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT);

    robot->wheel_motor_cmd[BAL_WHEEL_L].pos = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_L].vel = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_L].tor = wheel_t;
    robot->wheel_motor_cmd[BAL_WHEEL_L].kp = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_L].kd = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_L].enable = true;

    robot->wheel_motor_cmd[BAL_WHEEL_R].pos = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_R].vel = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_R].tor = wheel_t;
    robot->wheel_motor_cmd[BAL_WHEEL_R].kp = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_R].kd = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_R].enable = true;
}

void BalanceController_Stop(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        BalanceClearLegCmd(&robot->cmd[i]);
    }

    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }
}