#include "misc.hpp"
#include "robot.hpp"

void send_can_cmd() {
    // CAN1
    ui.SendCanCmd();

    const int16_t cmd1 = w1.GetCanCmd();
    const int16_t cmd2 = w2.GetCanCmd();
    const int16_t cmd3 = 0;
    const int16_t cmd4 = 0;
    const int16_t cmd5 = yaw2.GetCanCmd();
    const int16_t cmd6 = shoot.GetCanCmd();
    const int16_t cmd7 = s1.GetCanCmd();
    const int16_t cmd8 = s2.GetCanCmd();

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

    yaw1.SendCanCmd();

    // CAN2
    pitch.SendCanCmd();
    rub1.SendCanCmd();
    rub2.SendCanCmd();
}

void handle_yaw_offset() {
    static uint8_t flag = 0;

    if (flag == 0) {
        flashdb.Init("sentry");
        yaw1.config.offset = flashdb.Read("yaw_offset") * deg;
        flag = 1;
    } else if (flag == 1) {
        if (rc.vt13.mode == VT13::C && rc.vt13.fn_left && rc.vt13.fn_right) { // 等待按下
            flag = 2;
        }
    } else if (flag == 2) {
        if (!(rc.vt13.mode == VT13::C && rc.vt13.fn_left && rc.vt13.fn_right)) { // 等待松开
            yaw1.config.offset += yaw1.angle.measure;
            flashdb.Write("yaw_offset", yaw1.config.offset.toFloat(deg));
            buzzer.Play(Buzzer::G5G5G5);
            flag = 1;
        }
    }
}

void setup() {
    BSP::Init();
    // imu.Calibrate();
    buzzer.Play(Buzzer::C5D5G5);
}

void loop() {
    handle_yaw_offset();
    led.OnLoop();
    buzzer.OnLoop();
    robot.OnLoop();
    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    static UnitFloat<pct> cpu_usage;
    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
            const UnitFloat<ms> running_time = dwt.GetDT();
            cpu_usage = running_time / (1 * ms);
        }
    }
}