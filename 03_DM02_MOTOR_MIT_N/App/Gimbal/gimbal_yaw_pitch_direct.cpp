#include "gimbal_yaw_pitch_direct.h"
#include "gimbal_param.h"                                               // 存很多宏

void Gimbal::Init(void)
{
    // rc = Get_RC_Pointer();                                           // 获取遥控器指针

    reference.pitch = 0.0f;                                             // 期望值pitch初始化
    reference.yaw = 0.0f;                                               // 期望值yaw初始化
    feedback_pos.pitch = 0.0f;
    feedback_pos.yaw = 0.0f;
    feedback_vel.pitch = 0.0f;
    feedback_vel.yaw = 0.0f;

    upper_limit.pitch = GIMBAL_UPPER_LIMIT_PITCH;
    upper_limit.yaw = GIMBAL_LOWER_LIMIT_PITCH;

    const static float gimbal_yaw_angle[3] = {KP_GIMBAL_YAW_ANGLE,KI_GIMBAL_YAW_ANGLE,KD_GIMBAL_YAW_ANGLE};
    const static float gimbal_yaw_velocity[3]={KP_GIMBAL_YAW_VELOCITY,KI_GIMBAL_YAW_VELOCITY,KD_GIMBAL_YAW_VELOCITY};
    const static float gimbal_pitch_angle[3]={KP_GIMBAL_PITCH_ANGLE,KI_GIMBAL_PITCH_ANGLE,KD_GIMBAL_PITCH_ANGLE};
    const static float gimbal_pitch_velocity[3]={KP_GIMBAL_PITCH_VELOCITY,KI_GIMBAL_PITCH_VELOCITY,KD_GIMBAL_PITCH_VELOCITY};

    pid.yaw_angle_pid.Init(gimbal_yaw_angle[0],gimbal_yaw_angle[1],gimbal_yaw_angle[2]);
    pid.yaw_velocity_pid.Init(gimbal_yaw_velocity[0],gimbal_yaw_velocity[1],gimbal_yaw_velocity[2]);
    pid.pitch_angle_pid.Init(gimbal_pitch_angle[0],gimbal_pitch_angle[1],gimbal_pitch_angle[2]);
    pid.pitch_velocity_pid.Init(gimbal_pitch_velocity[0],gimbal_pitch_velocity[1],gimbal_pitch_velocity[2]);

    yaw_motor.Init(&hfdcan1, 0x000, 0x001, Motor_DM_Control_Method_NORMAL_MIT);
    // pitch_motor.Init(&hfdcan1, 0x02, 0x02, Motor_DM_Control_Method_NORMAL_MIT);

    BMI088.Init();                                                      // 初始化陀螺仪，但我不确定是否正确

    init_start_time=0;                                                  // 云台校准开始时间
    init_timer=0;
    init_continue=false;

    mode = GIMBAL_ZERO_FORCE;                                           // 云台初始模式
    last_mode = GIMBAL_ZERO_FORCE;
    mode_before_rc_err = GIMBAL_ZERO_FORCE;
}

void Gimbal::GimbalObserver(void)
{
    // yaw_motor.                                                       // 获取电机当前的yaw角度和角速度
    // pitch_motor.                                                     // 获取电机当前的pitch角度和角速度

    Class_Matrix_f32<3, 1> euler_angles = BSP_BMI088.Get_Euler_Angle();
    Class_Matrix_f32<3, 1> euler_gyro = BSP_BMI088.Get_Gyro();

    feedback_pos.pitch = *euler_angles[0];                              // 更新状态量pitch
    feedback_pos.yaw = *euler_angles[1];                                // 更新状态量yaw
    feedback_vel.pitch = *euler_gyro[0];                                // 更新状态量pitch角速度
    feedback_vel.yaw = *euler_gyro[1];                                  // 更新状态量yaw角速度

    if (this->mode == GIMBAL_INIT)                                      // 如果是云台矫正模式
    {
        if (this->last_mode != GIMBAL_INIT)                             // 如果上一次不是云台矫正模式，重置计时器
        {
            // 计时器记录当前时间
        }
        // 计时器记录当前时间与开始时间的差值，作为云台矫正模式的运行时间
    }
    else                                                                // 不是云台矫正模式，重置计时器
    {
        this->init_timer=0;
    }

    // 保存进入遥控器错误模式之前的云台工作模式，以便在遥控器恢复后能够自动恢复到原来的模式
    if (this->mode == GIMBAL_DBUS_ERR && this->last_mode != GIMBAL_DBUS_ERR )
    {
        this->mode_before_rc_err=this->last_mode;
    }

    this->last_mode=this->mode;                                         // 更新"上一次"运行模式
}

void Gimbal::GimbalSetMode(void)
{

}

void Gimbal::GimbalReference(void)
{
    if (this->mode == GIMBAL_INIT)                                      // 云台矫正模式
    {
        // 进行云台校准，获取零点位置，调整陀螺仪数据
    }
    else if (this->mode == GIMBAL_GAP)                                  // 跳出矫正进入手动或自动模式之前的存储数据模式
    {
        this->reference.pitch = this->feedback_pos.pitch;
        this->reference.yaw = this->feedback_pos.yaw;
    }
    else if (this->mode==GIMBAL_IMU)                                    // 云台陀螺仪控制(角度控制)
    {
        if (this->last_mode != GIMBAL_IMU)                              // 如果上一次不是云台陀螺仪控制模式，重置目标量为当前的状态量，防止突然跳变
        {
            this->reference.pitch=this->feedback_pos.pitch;
            this->reference.yaw=this->feedback_pos.yaw;
        }
        else 
        {
            // 读取遥感器数据，更新目标量
        }
  }
  else if (this->mode == GIMBAL_AUTO_AIM)                               // 自瞄模式
  {
    // 进行目标检测，获取目标位置，更新目标量
  }
}
