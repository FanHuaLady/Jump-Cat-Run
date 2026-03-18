#ifndef BSP_CONTROL_H
#define BSP_CONTROL_H

#include "stm32h7xx_hal.h"
#include <cstdint>      // 推荐使用 C++ 标准头文件
#include <cstring>

#define RC_CH_VALUE_OFFSET      1024                    // 通道值偏移量（SBUS 中值 1024）

class Class_Control 
{
public:
    // 遥控器数据结构
    struct Data_t 
    {
        struct {
            int16_t ch[6];      // 通道 0~5（0右平，1右竖，2左平，3左竖，4旋钮A，5旋钮B）
            int8_t  s[4];       // 开关 0~3（值 -1/0/1）
        } rc;
        uint8_t failsafe;       // 失控保护标志（1=激活）
        uint8_t frame_lost;     // 丢帧标志（1=丢帧）
    };

    void Init(UART_HandleTypeDef *huart, uint32_t offline_ms = 100);
    void TIM_1ms_Process_PeriodElapsedCallback();           // 定时器中断处理函数（用于掉线检测）

    inline const Data_t &GetData() const { return Data; }   // 获取最新遥控器数据（只读）
    inline bool IsOffline() const { return OfflineFlag; }   // 遥控器掉线标志

private:
    UART_HandleTypeDef *UART_Handler;                       // 串口句柄
    Data_t              Data;                               // 解析后的数据
    uint32_t            LastReceiveTick;                    // 最后接收时间戳 (ms)
    uint32_t            OfflineThreshold;                   // 掉线阈值 (ms)
    bool                OfflineFlag;                        // 当前掉线标志
    
    void ParseI6x(uint8_t *buf);                            // i6x 解析函数

    static void UART_RxCallback(uint8_t *Buffer, uint16_t Length);
};

#endif // BSP_CONTROL_H
