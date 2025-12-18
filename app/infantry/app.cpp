#include "config.hpp"
#include "lib.hpp"

// app
#include "chassis.hpp"
#include "gimbal.hpp"
#include "shooter.hpp"

LED led;
// 控制器
DJ6 dj6;
VT13 vt13;
Mavlink mavlink;
// 传感器
IMU imu(config.imu.dir, config.imu.calib);
// 执行器
Chassis chassis(&config.chassis.wheel_pid, &config.chassis.follow_pid);
Gimbal gimbal(imu, &config.gimbal.yaw_pid, &config.gimbal.pitch_pid);
Shooter shooter(&config.shooter.shoot_pid);
// 裁判系统交互
Referee referee;
UI ui;

Robot robot(config.config,
            dj6, vt13, mavlink,       // 控制器
            imu,                      // 传感器
            chassis, gimbal, shooter, // 执行器
            referee, ui               // 裁判系统交互
);

void handle_can() {
    // 底盘
    const int16_t cmd1 = chassis.m_wheel1.GetCurrentCmd();
    const int16_t cmd2 = chassis.m_wheel2.GetCurrentCmd();
    const int16_t cmd3 = chassis.m_wheel3.GetCurrentCmd();
    const int16_t cmd4 = chassis.m_wheel4.GetCurrentCmd();
    // 云台
    const int16_t cmd5 = gimbal.m_yaw.GetVoltageCmd();
    // 发射机构
    const int16_t cmd7 = shooter.m_shoot.GetCurrentCmd();

    uint8_t data[8];

    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = cmd3 >> 8;
    data[5] = cmd3;
    data[6] = cmd4 >> 8;
    data[7] = cmd4;
    BSP::CAN::TransmitStd(1, 0x200, data);

    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = 0;
    data[3] = 0;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = 0;
    data[7] = 0;
    BSP::CAN::TransmitStd(1, 0x1FF, data);
}

void setup() {
    BSP::Init();

    imu.Init();
    // imu.Calibrate();
}

void loop() {
    led.OnLoop();
    robot.OnLoop();

    handle_can();
}

static BSP::Dwt dwt;
static UnitFloat<pct> cpu_usage;

extern "C" void app_main() {
    setup();

    while (true) {
        if (dwt.GetDT() >= 0.001f) {
            const float interval_time = dwt.UpdateDT();
            loop();
            const float running_time = dwt.GetDT();
            cpu_usage = running_time / interval_time * ratio;
        }
    }
}

extern "C" void print_message(const uint8_t* message, const int length) {
    referee.AddCanData(message, length);
}