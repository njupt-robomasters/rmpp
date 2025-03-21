#include "app_init.h"
#include "bsp_led.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_cdc.h"
#include "bsp_pwm.h"

void app_init() {
    BSP_LED_Init();
    BSP_DWT_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_CDC_Init();
    BSP_PWM_Init();
}
