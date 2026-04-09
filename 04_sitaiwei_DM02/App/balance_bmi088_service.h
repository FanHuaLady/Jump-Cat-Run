#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BalanceBmi088Service_Init(void);

void BalanceBmi088Service_Timer10usCallback(void);
void BalanceBmi088Service_Timer125usCallback(void);
void BalanceBmi088Service_Timer128msCallback(void);

#ifdef __cplusplus
}
#endif