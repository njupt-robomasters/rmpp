#include "app.hpp"

Settings settings;
IMU imu(settings.imu_dir, settings.imu_calib);
DJ6 dj6;
Chassis chassis(settings.servo_pid, settings.wheel_pid);

extern "C" void app_init() {
    BSP::Init();

    extern void task_led_entry(const void* argument);
    BSP::OS::Create("task_led", task_led_entry, osPriorityIdle, 128);

    extern void task_imu_entry(const void* argument);
    BSP::OS::Create("task_imu", task_imu_entry, osPriorityRealtime, 128);

    extern void task_chassis_entry(const void *argument);
    BSP::OS::Create("task_chassis", task_chassis_entry, osPriorityHigh, 128);
}
