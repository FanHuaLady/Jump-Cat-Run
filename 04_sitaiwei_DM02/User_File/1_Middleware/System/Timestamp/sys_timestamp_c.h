#ifndef SYS_TIMESTAMP_C_H
#define SYS_TIMESTAMP_C_H

#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

void SysTimestamp_Init(TIM_HandleTypeDef *htim);
void SysTimestamp_TIM_3600s_PeriodElapsedCallback(void);

#ifdef __cplusplus
}
#endif

#endif
