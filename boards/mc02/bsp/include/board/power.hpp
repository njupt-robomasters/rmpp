#pragma once

// mc02 可控电源，上电默认全部关闭；只有 mc02 提供此头文件，其他板卡引用时编译报错
enum class PowerOutput {
    OUT1,   // XT30 可控输出 1，输入电压，持续 5 A
    OUT2,   // XT30 可控输出 2，输入电压，持续 5 A
    OUT_5V, // 可控 5 V，供 4 路 PWM、一路串口和一路 CAN 接口电源脚，持续 2 A
};

void set_power(PowerOutput output, bool enabled);
