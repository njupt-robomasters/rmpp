#include "led.hpp"
#include "robot.hpp"

void send_can_cmd() {
    // CAN1
    ui.SendCanCmd();

    const int16_t cmd1 = w1.GetCanCmd();
    const int16_t cmd2 = w2.GetCanCmd();
    const int16_t cmd3 = w3.GetCanCmd();
    const int16_t cmd4 = w4.GetCanCmd();
    const int16_t cmd5 = yaw.GetCanCmd();
    const int16_t cmd6 = 0;
    const int16_t cmd7 = shoot.GetCanCmd();
    const int16_t cmd8 = 0;

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
    data[2] = cmd6 >> 8;
    data[3] = cmd6;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = cmd8 >> 8;
    data[7] = cmd8;
    BSP::CAN::TransmitStd(1, 0x1FF, data);

    // CAN2
    pitch.SendCanCmd();
    rub_left.SendCanCmd();
    rub_right.SendCanCmd();
}

void handle_ui_robot() {
    ui.robot.w1 = w1.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.w2 = w2.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.w3 = w3.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.w4 = w4.is_connect ? UI::GREEN : UI::PINK;

    ui.robot.cap = UI::PINK;

    ui.robot.yaw1 = yaw.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.pitch = pitch.is_connect ? UI::GREEN : UI::PINK;

    ui.robot.rub_left1 = rub_left.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.rub_right1 = rub_right.is_connect ? UI::GREEN : UI::PINK;

    if (rub_left.is_connect == false) {
        ui.robot.rub_left1 = UI::PINK;
    } else if (unit::abs(shooter.bullet_speed.ref - shooter.bullet_speed.measure_left1) < 1) {
        ui.robot.rub_left1 = UI::GREEN;
    } else {
        ui.robot.rub_left1 = UI::ORANGE;
    }

    if (rub_right.is_connect == false) {
        ui.robot.rub_right1 = UI::PINK;
    } else if (unit::abs(shooter.bullet_speed.ref - shooter.bullet_speed.measure_right1) < 1) {
        ui.robot.rub_right1 = UI::GREEN;
    } else {
        ui.robot.rub_right1 = UI::ORANGE;
    }

    ui.robot.shoot = shoot.is_connect ? UI::GREEN : UI::PINK;

    ui.robot.aim = mavlink.is_connect ? UI::GREEN : UI::PINK;
}

void setup() {
    BSP::Init();
    // imu.Calibrate();
}

void loop() {
    led.OnLoop();
    robot.OnLoop();
    handle_ui_robot();
    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
        }
    }
}