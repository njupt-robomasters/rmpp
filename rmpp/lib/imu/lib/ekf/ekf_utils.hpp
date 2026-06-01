#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

// 以下是一些简单的数学运算工具
namespace MathUtils {

    // // 硬件加速的快速反平方根 (开启 -Ofast 后会编译为单指令)
    // inline float invSqrt(float x) {
    //     if (x <= 1e-6f) return 0.0f; // 防除零和负数崩溃
    //     return 1.0f / std::sqrt(x);
    // }
    // 上面的快速反平方跟计算需要硬件支持
    inline float invSqrt(float x) {
        // 防止除以零或者负数
        if (x<= 1e-6f) return 0.0f;

        float halfx = 0.5f * x;
        float y = x;
        uint32_t i;
        // 把浮点数的内存二进制位复制给整型变量
        std::memcpy(&i,&y,sizeof(i));
        // 魔数和位运算
        i = 0x5f3759df - (i >> 1);
        // 把修改后的二进制位复制回浮点数
        std::memcpy(&y,&i,sizeof(y));
        // 牛顿迭代法(一次就够)
        y = y * (1.5f - (halfx * y * y));
        return y;
    }

    // 绝对值限幅
    constexpr float abs_limit(float num, float Limit) {
        if (num > Limit) return Limit;
        if (num < -Limit) return -Limit;
        return num;
    }

    // 浮点限幅
    constexpr float float_constrain(float Value, float minValue, float maxValue) {
        if (Value < minValue) return minValue;
        if (Value > maxValue) return maxValue;
        return Value;
    }

    // 循环限幅
    constexpr float loop_float_constrain(float Input, float minValue, float maxValue) {
        if (maxValue < minValue) return Input;
        float len = maxValue - minValue;
        while (Input > maxValue) Input -= len;
        while (Input < minValue) Input += len;
        return Input;
    }

    /**
     * @brief 静态一阶最小二乘法平滑/求导器 （未使用 malloc）
     */
    template <uint16_t ORDER>
    class OrdinaryLeastSquares {
    private:
        uint32_t count = 0;
        float x[ORDER] = {0};
        float y[ORDER] = {0};
        float t[4] = {0};

        float k = 0;
        float b = 0;

    public:
        OrdinaryLeastSquares() = default;
        // deltax
        void Update(float deltax, float input_y) {
            float temp = x[1];
            for (uint16_t i = 0; i < ORDER - 1; ++i) {
                x[i] = x[i + 1] - temp;
                y[i] = y[i + 1];
            }
            x[ORDER - 1] = x[ORDER - 2] + deltax;
            y[ORDER - 1] = input_y;

            if (count < ORDER) count++;
            // 防除零与单点死锁
            if (count < 2) {
                k = 0.0f;
                b = input_y;
                return;
            }

            std::memset(t, 0, sizeof(t));
            for (uint16_t i = ORDER - count; i < ORDER; ++i) {
                t[0] += x[i] * x[i];              // t[0] = ∑xi^2
                t[1] += x[i];                     // t[1] = ∑xi
                t[2] += x[i] * y[i];              // t[2] = ∑xi*yi
                t[3] += y[i];                     // t[3] = ∑yi
            }

            // 把原代码 k 里乘的ORDER改为了count 防止数据未满时出错
            float denominator = (t[0] * count - t[1] * t[1]);
            if (std::abs(denominator) > 1e-6f) {
                k = (t[2] * count - t[1] * t[3]) / denominator;
                b = (t[0] * t[3] - t[1] * t[2]) / denominator;
            }
        }
        // 偏导计算出k
        float GetDerivative() const { return k; }
        // 拟合出的平滑值
        float GetSmooth() const { return k * x[ORDER - 1] + b; }
    };
}