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

extern void loop_led();
extern void loop_remote();
extern void loop_imu();
extern void loop_chassis();
extern void loop_gimbal();
extern void loop_shooter();
extern void loop_can();

void setup() {
    BSP::Init();

    imu.Init();
    // imu.Calibrate();
}

void loop() {
    loop_led();
    loop_remote();
    loop_imu();
    loop_chassis();
    loop_gimbal();
    loop_shooter();
    loop_can();
}

extern "C" void app_main() {
    setup();

    BSP::Dwt dwt;
    while (true) {
        if (dwt.GetDT() >= 0.001f) {
            dwt.UpdateDT();
            loop();
        }
    }
}