#include "app_init.h"

#include "bsp_led.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_cdc.h"
#include "bsp_imuheat.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"

#include "app_variable.hpp"

void app_init() {
    // bsp init
    BSP_LED_Init();
    BSP_DWT_Init();
    BSP_CAN_Init();
    BSP_CDC_Init();
    BSP_IMUHeat_Init();
    BSP_UART_Init();
    BSP_PWM_Init();
}
