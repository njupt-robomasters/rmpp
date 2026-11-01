/*
 * 蜂鸣器沙沙声定位：依次播放以下段落，每段 3 s，段间 1 s 静音（占空比 0），循环
 * 1. 占空比固定 50%，无信号：仍有沙沙声说明来自载波或驱动
 * 2. A4 方波，Timer 轮询 40 kHz 写 CCR（与 buzzer_duo_v1 相同）
 * 3. A4 方波，与载波同步 80 kHz；与 2 对比判断时间抖动
 * 4. C7 方波，同步、未限带
 * 5. C7 方波，同步、PolyBLEP 限带；与 4 对比判断混叠
 * 信号幅度均为占空比 0.5 ± 0.25，与 2 声部测试的单声部幅度相当
 */

#include "bsp/timer.hpp"
#include "pwm_dac.hpp"

static constexpr float SEGMENT = 3.0f; // 每段时长 s
static constexpr float GAP = 1.0f;     // 段间静音 s
static constexpr float FADE = 0.02f;   // 斜坡时间 s
static constexpr float AMP = 0.25f;    // 信号幅度，占空比单位
static constexpr float A4 = 440.0f;
static constexpr float C7 = 2093.0f;

// 与载波同步播放 hz 方波；hz 为 0 时只输出 50%
static void play_sync(const float hz, const bool is_blep) {
    const float step = hz * SAMPLE_DT;
    float phase = 0;
    const int count = static_cast<int>(SEGMENT / SAMPLE_DT);
    for (int i = 0; i < count; i++) {
        phase += step;
        if (phase >= 1)
            phase -= 1;

        float x = 0;
        if (hz > 0)
            x = is_blep ? square_blep(phase, step) : (phase < 0.5f ? 1.0f : -1.0f);
        wait_sample();
        set_duty(0.5f + AMP * x);
    }
}

// 不同步：用 Timer 按 40 kHz 轮询写 CCR，相位按实际 dt 推进
static void play_timer(const float hz) {
    Timer timer;
    timer.restart();
    float phase = 0, time = 0;
    while (time < SEGMENT) {
        if (timer.elapsed() < 1.0f / 40000)
            continue;
        const float dt = timer.restart();
        time += dt;
        phase += hz * dt;
        phase -= static_cast<float>(static_cast<int>(phase));
        set_duty(0.5f + AMP * (phase < 0.5f ? 1.0f : -1.0f));
    }
}

extern "C" void app_main() {
    start_carrier();

    while (true) {
        for (int segment = 1; segment <= 5; segment++) {
            ramp_duty(0, 0.5f, FADE);
            if (segment == 1)
                play_sync(0, false);
            else if (segment == 2)
                play_timer(A4);
            else if (segment == 3)
                play_sync(A4, false);
            else if (segment == 4)
                play_sync(C7, false);
            else
                play_sync(C7, true);
            ramp_duty(0.5f, 0, FADE);
            Timer::delay(GAP);
        }
    }
}
