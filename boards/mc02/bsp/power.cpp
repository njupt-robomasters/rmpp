/*
 * 可控电源使能：PC13 = POWER_OUT1_EN，PC14 = POWER_OUT2_EN，PC15 = POWER_5V_EN
 * GPIO_Output，推挽，初始低电平（关闭），高电平打开
 */

#include "board/power.hpp"
#include "main.h"

void set_power(const PowerOutput output, const bool enabled) {
    const GPIO_PinState state = enabled ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (output == PowerOutput::OUT1)
        HAL_GPIO_WritePin(POWER_OUT1_EN_GPIO_Port, POWER_OUT1_EN_Pin, state);
    else if (output == PowerOutput::OUT2)
        HAL_GPIO_WritePin(POWER_OUT2_EN_GPIO_Port, POWER_OUT2_EN_Pin, state);
    else if (output == PowerOutput::OUT_5V)
        HAL_GPIO_WritePin(POWER_5V_EN_GPIO_Port, POWER_5V_EN_Pin, state);
}
