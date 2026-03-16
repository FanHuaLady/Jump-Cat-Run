#include "gimbal_task.h"

#include "gimbal_yaw_pitch_direct.h"

#define GIMBAL_TASK_INIT_TIME 200

void gimbal_task(void const * pvParameters)
{

    // 云台发布数据
    // GimbalPublish();                                                    // 不确定作用
    // 等待陀螺仪任务更新陀螺仪数据
    // vTaskDelay(GIMBAL_TASK_INIT_TIME);                                  // 等待一段时间
    // 云台初始化
    // GimbalInit();                                                       // 初始化PID还有电机

    // Gimbal gimbal;
    // gimbal.Gimbal_Init();                                                  // 初始化云台

    while (1) 
    {
        // 更新状态量
        // 电机数据来自CAN通信的CallBack
        // IMU数据来自IMU_task
        // GimbalObserver();                                               // 获取角度和速度反馈值

        // 处理异常
        // GimbalHandleException();                                        // 找不到解决方案

        // 设置云台模式
        // 遥控器开关控制模式切换，优先级：安全档 > 遥控器断连 > 其他模式
        // GimbalSetMode();
        
        // 更新目标量
        // GimbalReference();
        // 计算控制量
        // GimbalConsole();
        // 发送控制量
        // GimbalSendCmd();
        // 系统延时
        // vTaskDelay(GIMBAL_CONTROL_TIME);
    }
}