#pragma once
#include "stm32f4xx.h"
inline constexpr unsigned HAL_TIM_STATE_RESET = 0, TIM_EGR_UG = 1, RCC_HCLK_DIV1 = 1;
struct TimerRegisters {
    std::uint32_t PSC = 0, ARR = 65535, CCR[4]{}, EGR = 0;
};
struct TIM_HandleTypeDef {
    TimerRegisters* Instance;
    unsigned State = 1;
};
inline TimerRegisters reg1, reg4, reg5, reg8, reg10;
inline TIM_HandleTypeDef htim1{&reg1}, htim4{&reg4}, htim5{&reg5}, htim8{&reg8}, htim10{&reg10};
struct RCC_ClkInitTypeDef {
    std::uint32_t APB1CLKDivider = 4, APB2CLKDivider = 2;
};
inline void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef*, std::uint32_t*) {}
inline std::uint32_t HAL_RCC_GetPCLK1Freq() { return 42000000; }
inline std::uint32_t HAL_RCC_GetPCLK2Freq() { return 84000000; }
inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, unsigned) { return HAL_OK; }
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_SET_AUTORELOAD(h, value) ((h)->Instance->ARR = (value))
#define __HAL_TIM_SET_PRESCALER(h, value) ((h)->Instance->PSC = (value))
#define __HAL_TIM_SET_COMPARE(h, channel, value) ((h)->Instance->CCR[(channel) / 4] = (value))
