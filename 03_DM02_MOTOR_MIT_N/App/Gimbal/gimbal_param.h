#ifndef GIMBAL_PARAM_H
#define GIMBAL_PARAM_H

// 云台限制参数
#define GIMBAL_UPPER_LIMIT_PITCH (0.3f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.5f)

#define KP_GIMBAL_YAW_ANGLE (20.00f)
#define KI_GIMBAL_YAW_ANGLE (0.003f)
#define KD_GIMBAL_YAW_ANGLE (0.35f)

#define KP_GIMBAL_YAW_VELOCITY (20000.0f)
#define KI_GIMBAL_YAW_VELOCITY (1.0f)
#define KD_GIMBAL_YAW_VELOCITY (0.0f)

#define KP_GIMBAL_PITCH_ANGLE (25.0f)                           // KP参数
#define KI_GIMBAL_PITCH_ANGLE (0.0f)                            // KI参数        
#define KD_GIMBAL_PITCH_ANGLE (3.0f)                            // KD参数 

#define KP_GIMBAL_PITCH_VELOCITY (3500.0f)                      // KP参数
#define KI_GIMBAL_PITCH_VELOCITY (30.0f)                        // KI参数
#define KD_GIMBAL_PITCH_VELOCITY (50.0f)                        // KD参数     

#endif