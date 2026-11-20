#include "../../jelly_sentry/gimbal.hpp"
#include "bsp/timer.hpp"
#include "rc/fsi6x.hpp"
#include "rc/vt13.hpp"
#include <algorithm>

// pitch2 单轴测试：云台只控制俯仰轴 pitch2，CAN 只发送 pitch2（CAN2，CAN ID 0x11 / Master ID 0x21），用于排查 pitch2 震荡与断联
// 大 yaw、小 yaw、pitch1 不发送任何指令：达妙与 7025 收不到命令不会应答，保持离线，云台对它们输出零，不会动作
// 电机、PID、限位、IMU 安装方向直接使用整车 app/jelly_sentry/gimbal.hpp，调好后无需搬运
// 使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；左手竖直摇杆控制 pitch 转速
// is_imu_mode：true 用 IMU 闭环（与整车一致），false 只用编码器闭环；调试器修改后下一周期生效
// 观察：pitch2.is_connect、pitch2.err_code、disconnect_count、pitch2.angle、pitch2.speed、pitch2.torque_ref、
//      gimbal.pitch.ecd.measure、gimbal.pitch.imu.measure、pitch2_speed_pid.ff

static constexpr float WPITCH_MAX = 360 * deg_s; // pitch 最大角速度，rad/s

volatile bool is_imu_mode = true; // true IMU 模式，false 编码器模式

static VT13 vt13({.uart = Uart(1)});
static FSi6X fsi6x({.uart = Uart(5)});

// 跨周期状态：pitch2 从在线变为离线的次数，用于统计断联
static uint32_t disconnect_count = 0;
static bool last_connect = false;

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
static bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}

extern "C" void app_main() {
    Timer loop_timer;
    uint32_t pitch2_divider = 0;
    while (true) {
        if (loop_timer.elapsed() < 1 * ms)
            continue;
        loop_timer.restart();

        vt13.update();
        fsi6x.update();

        // 只给 pitch 转速，yaw 速度为零
        gimbal.set_mode(is_imu_mode ? GimbalFold::IMU_MODE : GimbalFold::ECD_MODE);
        gimbal.set_enable(rc_enable());
        const float pitch_speed = std::clamp((vt13.pitch + fsi6x.pitch) * WPITCH_MAX, -WPITCH_MAX, WPITCH_MAX);
        gimbal.set_speed(0, pitch_speed);
        attitude.update();
        gimbal.update(attitude.yaw, attitude.pitch);

        // 断联计数：在线变离线时加一
        if (last_connect && !pitch2.is_connect)
            disconnect_count++;
        last_connect = pitch2.is_connect;

        // CAN 只发送 pitch2
        if (++pitch2_divider >= pitch2_send_period_ms) {
            pitch2_divider = 0;
            pitch2.send();
        }
    }
}
