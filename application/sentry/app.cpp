#include "app.hpp"

ModuleParams module_params;
AppParams app_params;
IMU imu(module_params.imu_dir, module_params.imu_calib);
DJ6 dj6;
Referee referee;
Chassis chassis(&module_params.servo_pid_param, &module_params.wheel_pid_param);
Gimbal gimbal(imu, &module_params.yaw1_pid_param, &module_params.yaw2_pid_param, &module_params.pitch_pid_param);
Shooter shooter(&module_params.m2006_pid_param);

extern "C" void app_init() {
    BSP::Init();

    extern void task_led_entry(const void* argument);
    BSP::OS::Create("task_led", task_led_entry, osPriorityIdle, 128);

    extern void task_imu_entry(const void* argument);
    BSP::OS::Create("task_imu", task_imu_entry, osPriorityRealtime, 128);

    extern void task_can_entry(const void* argument);
    BSP::OS::Create("task_can", task_can_entry, osPriorityHigh, 128);

    extern void task_chassis_entry(const void* argument);
    BSP::OS::Create("task_chassis", task_chassis_entry, osPriorityNormal, 128);

    extern void task_gimbal_entry(const void* argument);
    BSP::OS::Create("task_gimbal", task_gimbal_entry, osPriorityNormal, 128);

    extern void task_shooter_entry(const void* argument);
    BSP::OS::Create("task_shooter", task_shooter_entry, osPriorityNormal, 128);
}
