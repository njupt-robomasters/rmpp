#include "app.hpp"

Parameter param;
Config cfg;
Variable var;

DJ6 dj6;
VT13 vt13;
IMU imu(param.imu.dir, param.imu.calib);
Chassis chassis(&param.chassis.servo_pid_param, &param.chassis.wheel_pid_param);
Gimbal gimbal(imu, &param.gimbal.yaw1_pid_param, &param.gimbal.yaw2_pid_param, &param.gimbal.pitch_pid_param);
Shooter shooter(&param.shooter.shoot_pid_param);

extern "C" void app_init() {
    BSP::Init();

    extern void task_led_entry(void* argument);
    BSP::OS::TaskCreate(task_led_entry, "task_led", 1024, osPriorityIdle);

    extern void task_imu_entry(void* argument);
    BSP::OS::TaskCreate(task_imu_entry, "task_imu", 1024, osPriorityRealtime);

    extern void task_can_entry(void* argument);
    BSP::OS::TaskCreate(task_can_entry, "task_can", 1024, osPriorityHigh);

    extern void task_remote_entry(void* argument);
    BSP::OS::TaskCreate(task_remote_entry, "task_remote", 1024, osPriorityHigh);

    extern void task_chassis_entry(void* argument);
    BSP::OS::TaskCreate(task_chassis_entry, "task_chassis", 1024, osPriorityNormal);

    extern void task_gimbal_entry(void* argument);
    BSP::OS::TaskCreate(task_gimbal_entry, "task_gimbal", 1024, osPriorityNormal);

    extern void task_shooter_entry(void* argument);
    BSP::OS::TaskCreate(task_shooter_entry, "task_shooter", 1024, osPriorityNormal);
}