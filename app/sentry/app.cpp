#include "config.hpp"

// bsp
#include "bsp/bsp.hpp"

// lib
#include "template/led.hpp"
#include "rc/dj6.hpp"
#include "referee/vt13.hpp"
#include "referee/referee.hpp"
#include "imu/imu.hpp"
#include "template/control.hpp"

// app
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

LED led;
// 控制器
DJ6 dj6;
VT13 vt13;
Referee referee;
NUC nuc;
// 传感器
IMU imu(config.imu.dir, config.imu.calib);
// 执行器
Chassis chassis(&config.chassis.servo_pid_param, &config.chassis.wheel_pid_param);
Gimbal gimbal(imu, &config.gimbal.yaw1_pid_param, &config.gimbal.yaw2_pid_param, &config.gimbal.pitch_pid_param);
Shooter shooter(&config.shooter.shoot_pid_param);

Control control(config.speed,
                dj6, vt13, referee, nuc, // 控制器
                imu,                     // 传感器
                chassis, gimbal, shooter // 执行器
);

void handle_can() {
    // 底盘
    const int16_t cmd7 = chassis.m_servo1.GetVoltageCmd();
    const int16_t cmd8 = chassis.m_servo2.GetVoltageCmd();
    const int16_t cmd1 = chassis.m_wheel1.GetCurrentCmd();
    const int16_t cmd2 = chassis.m_wheel2.GetCurrentCmd();

    // 云台
    const int16_t cmd5 = gimbal.m_yaw2.GetVoltageCmd();

    // 发射机构
    const int16_t cmd6 = shooter.m_shoot.GetCurrentCmd();

    uint8_t data[8];

    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x200, data);

    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = cmd6 >> 8;
    data[3] = cmd6;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = cmd8 >> 8;
    data[7] = cmd8;
    BSP::CAN::TransmitStd(1, 0x1FF, data);
}

void setup() {
    BSP::Init();

    imu.Init();
    // imu.Calibrate();
}

void loop() {
    led.OnLoop();
    control.OnLoop();

    handle_can();
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