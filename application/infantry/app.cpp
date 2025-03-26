#include "app.hpp"

Settings settings;
Status status;
DJ6 dj6;
IMU imu(settings.imu_param, settings.imu_calib);
Chassis chassis(settings.wheel_pid);
Gimbal gimbal(imu, settings.pitch_pid, settings.yaw_pid, settings.shoot_pid);
Referee referee;
UI ui;
RV2 rv2;
SuperCapacity superCapacity;

static void can_callback(const uint32_t id, uint8_t data[8]) {
    task_chassis_callback(id, data);
    task_gimbal_callback(id, data);
}

void app_init() {
    BSP_LED_Init();
    BSP_DWT_Init();
    BSP_CAN_Init();
    BSP_PWM_Init();
    BSP_UART_Init();
    BSP_CDC_Init();

    // 设置中断回调函数
    BSP_CAN_SetCallback(can_callback);
    BSP_UART_RC_SetCallback(task_protocol_rc_callback);
    BSP_UART_Referee_SetCallback(task_protocol_referee_callback);
    BSP_CDC_SetCallback(task_protocol_cdc_callback);

    osThreadDef(task_led, task_led_entry, osPriorityIdle, 0, 128);
    osThreadCreate(osThread(task_led), NULL);

    osThreadDef(task_chassis, task_chassis_entry, osPriorityHigh, 0, 256);
    osThreadCreate(osThread(task_chassis), NULL);

    osThreadDef(task_imu, task_imu_entry, osPriorityNormal, 0, 256);
    osThreadCreate(osThread(task_imu), NULL);

    osThreadDef(task_gimbal, task_gimbal_entry, osPriorityHigh, 0, 256);
    osThreadCreate(osThread(task_gimbal), NULL);

    osThreadDef(task_protocol_ui, task_protocol_ui_entry, osPriorityNormal, 0, 256);
    osThreadCreate(osThread(task_protocol_ui), NULL);

    osThreadDef(task_protocol_rv2, task_protocol_rv2_entry, osPriorityNormal, 0, 256);
    osThreadCreate(osThread(task_protocol_rv2), NULL);
}
