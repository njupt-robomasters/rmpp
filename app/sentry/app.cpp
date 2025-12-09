#include "app.hpp"

Parameter param;
Config cfg;
Variable var;

LED led;
DJ6 dj6;
VT13 vt13;
Referee referee;
IMU imu(param.imu.dir, param.imu.calib);
Chassis chassis(&param.chassis.servo_pid_param, &param.chassis.wheel_pid_param);
Gimbal gimbal(imu, &param.gimbal.yaw1_pid_param, &param.gimbal.yaw2_pid_param, &param.gimbal.pitch_pid_param);
Shooter shooter(&param.shooter.shoot_pid_param);

extern void loop_control();
extern void loop_can();

void setup() {
    BSP::Init();

    imu.Init();
    // imu.Calibrate();

    shooter.SetBulletSpeed(18.0f * m_s); // 设置弹速
    shooter.SetShootFreq(5.0f * Hz); // 设置弹频
}

void loop() {
    loop_control();

    led.OnLoop();
    dj6.OnLoop();
    vt13.OnLoop();
    referee.OnLoop();
    imu.OnLoop();
    chassis.OnLoop();
    gimbal.OnLoop();

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