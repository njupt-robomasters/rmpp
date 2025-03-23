#include "app_init.h"
#include "cmsis_os.h"

#include "bsp_led.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_cdc.h"
#include "bsp_pwm.h"

#include "task_led.h"
#include "task_chassis.h"
#include "task_gimbal.h"
#include "task_imu.h"
#include "task_protocol.h"

void app_init() {
    BSP_LED_Init();
    BSP_DWT_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_CDC_Init();
    BSP_PWM_Init();

    osThreadDef(task_led, task_led_entry, osPriorityIdle, 0, 128);
    osThreadCreate(osThread(task_led), NULL);

    osThreadDef(task_chassis, task_chassis_entry, osPriorityHigh, 0, 1024);
    osThreadCreate(osThread(task_chassis), NULL);

    osThreadDef(task_gimbal, task_gimbal_entry, osPriorityHigh, 0, 1024);
    osThreadCreate(osThread(task_gimbal), NULL);

    osThreadDef(task_imu, task_imu_entry, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(task_imu), NULL);

    osThreadDef(task_protocol, task_protocol_entry, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(task_protocol), NULL);
}
