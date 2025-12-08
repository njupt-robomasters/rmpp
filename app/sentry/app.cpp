#include "app.hpp"

Parameter param;
Config cfg;
Variable var;

Scheduler scheduler;
DJ6 dj6;
VT13 vt13;
IMU imu(param.imu.dir, param.imu.calib);
Chassis chassis(&param.chassis.servo_pid_param, &param.chassis.wheel_pid_param);
Gimbal gimbal(imu, &param.gimbal.yaw1_pid_param, &param.gimbal.yaw2_pid_param, &param.gimbal.pitch_pid_param);
Shooter shooter(&param.shooter.shoot_pid_param);

extern void loop_led();
extern void loop_remote();
extern void loop_imu();
extern void loop_chassis();
extern void loop_gimbal();
extern void loop_shooter();
extern void loop_can();

extern "C" void app_main() {
    BSP::Init();

    imu.Init();
    // imu.Calibrate();

    scheduler.RegisterLoop("led", loop_led, 1000 * Hz);
    scheduler.RegisterLoop("remote", loop_remote, 1000 * Hz);
    scheduler.RegisterLoop("imu", loop_imu, 1000 * Hz);
    scheduler.RegisterLoop("chassis", loop_chassis, 1000 * Hz);
    scheduler.RegisterLoop("gimbal", loop_gimbal, 1000 * Hz);
    scheduler.RegisterLoop("shooter", loop_shooter, 1000 * Hz);
    scheduler.RegisterLoop("can", loop_can, 1000 * Hz);

    while (true) {
        scheduler.OnLoop();
    }
}