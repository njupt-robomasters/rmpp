#include "app.hpp"

Settings settings;
IMU imu(settings.imu_dir, settings.imu_calib);
DJ6 dj6;
Chassis chassis(settings.m6020_pid_param, settings.m3508_pid_param);
Shooter shooter(settings.m2006_pid_param);

extern "C" void app_init() {
    BSP::Init();

    extern void task_led_entry(const void* argument);
    BSP::OS::Create("task_led", task_led_entry, osPriorityIdle, 128);

    extern void task_imu_entry(const void* argument);
    BSP::OS::Create("task_imu", task_imu_entry, osPriorityRealtime, 128);

    extern void task_can_entry(const void *argument);
    BSP::OS::Create("task_can", task_can_entry, osPriorityHigh, 128);

    extern void task_chassis_entry(const void *argument);
    BSP::OS::Create("task_chassis", task_chassis_entry, osPriorityNormal, 128);

    extern void task_shooter_entry(const void *argument);
    BSP::OS::Create("task_shooter", task_shooter_entry, osPriorityNormal, 128);
}
