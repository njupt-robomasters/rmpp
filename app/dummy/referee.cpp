#include "referee.hpp"
#include "chassis.hpp"
#include <algorithm>

// 模拟裁判系统底盘功率检测：10 Hz 结算，缓冲能量按（上限 − 裁判功率）·周期增减，限幅 [0, 60] J
// 裁判功率取超电实测电池侧功率乘偏差比例，比例 > 1 模拟我们测量偏小
static constexpr float POWER_SCALE = 1.0f; // 裁判功率 / 超电实测
static constexpr float BUFFER_MAX = 60;    // 缓冲能量上限，J
static constexpr float REFEREE_DT = 0.1f;  // 结算周期，s

static uint32_t referee_count = 0; // 结算分频计数

void referee_loop() {
    referee_count++;
    if (referee_count % 100 == 0) {
        referee_power = super_cap.power.battery * POWER_SCALE;
        referee_energy = std::clamp(referee_energy + (REFEREE_LIMIT - referee_power) * REFEREE_DT, 0.0f, BUFFER_MAX);
    }
}
