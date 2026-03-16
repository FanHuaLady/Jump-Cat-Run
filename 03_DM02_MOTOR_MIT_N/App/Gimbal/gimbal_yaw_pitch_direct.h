#ifndef GIMBAL_YAW_PITCH_H
#define GIMBAL_YAW_PITCH_H

#include "1_Middleware/Algorithm/PID/alg_pid.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/BSP/BMI088/bsp_bmi088.h"

typedef enum {
    GIMBAL_ZERO_FORCE,                                          // 云台无力，所有控制量置0
    GIMBAL_IMU,                                                 // 云台陀螺仪控制(角度控制)
    GIMBAL_INIT,                                                // 云台矫正模式
    GIMBAL_DBUS_ERR,                                            // 遥控器断联相关处理任务
    GIMBAL_GAP,                                                 // 跳出矫正进入手动或自动模式之前的存储数据模式
    GIMBAL_AUTO_AIM,                                            // 自瞄模式
} GimbalMode_e;

typedef struct
{
    float pitch;
    float yaw;
} Values_t;

typedef struct 
{
    Class_PID yaw_angle_pid;
    Class_PID yaw_velocity_pid;
    Class_PID pitch_angle_pid;
    Class_PID pitch_velocity_pid;
}PID_t;

class Gimbal
{
public:
    // const RC_ctrl_t * rc;                                    // 遥控器指针
    GimbalMode_e mode,last_mode,mode_before_rc_err;             // 模式
    Class_Motor_DM_Normal yaw_motor,pitch_motor;                // 电机对象
    Values_t reference;                                         // 期望值
    Values_t feedback_pos,feedback_vel;                         // 状态值(目前专供给IMU数据)
    Values_t upper_limit;                                       // 上限值
    Values_t lower_limit;                                       // 下限值
    float angle_zero_for_imu;                                   // pitch电机处于中值时imupitch的角度
    unsigned int init_start_time,init_timer;
    bool init_continue;                                         // 是否继续进行校准模式

    PID_t pid;                                                  // PID控制器
    Class_BMI088 BMI088;                                        // 陀螺仪

    void Init(void);                                     // 云台初始化
    void GimbalObserver(void);                                  // 云台状态观察
    void GimbalSetMode(void);                                   // 云台模式切换
    void GimbalReference(void);                                 // 更新目标量
};

#endif
