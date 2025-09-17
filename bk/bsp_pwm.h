#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void BSP_PWM_Init();

void BSP_PWM_SetDuty(float duty);

void BSP_PWM_IMUHeat_SetPower(float power);

#ifdef __cplusplus
}
#endif
