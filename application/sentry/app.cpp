#include "app.hpp"

Settings settings;
DJ6 dj6;
ChassisSteer chassis(settings.servo_pid, settings.wheel_pid);
Referee referee;
NAV nav;

void app_init() {
    BSP_LED_Init();
    BSP_DWT_Init();
    BSP_CAN_Init();
    BSP_PWM_Init();
    BSP_UART_Init();
    BSP_CDC_Init();

    // 设置中断回调函数
    BSP_CAN_SetCallback(task_chassis_callback);
    BSP_UART_RC_SetCallback(task_protocol_rc_callback);
    BSP_UART_Referee_SetCallback(task_protocol_referee_callback);
    BSP_CDC_SetCallback(task_protocol_cdc_callback);

    osThreadDef(task_led, task_led_entry, osPriorityIdle, 0, 128);
    osThreadCreate(osThread(task_led), NULL);

    osThreadDef(task_chassis, task_chassis_entry, osPriorityHigh, 0, 128);
    osThreadCreate(osThread(task_chassis), NULL);

    osThreadDef(task_protocol_nav, task_protocol_nav_entry, osPriorityHigh, 0, 128);
    osThreadCreate(osThread(task_protocol_nav), NULL);
}
