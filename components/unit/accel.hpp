#pragma once

#include "utils.hpp"

class Accel {
private:
    // 实际存储的成员变量
    float m_rpss = 0; // 弧度/s²
    float m_apss = 0; // 度/s²
    float m_tpss = 0; // 圈/s²

    // 弧度/s²代理类
    class RpssProxy {
    private:
        Accel &m_accel;

    public:
        explicit RpssProxy(Accel &a) : m_accel(a) {
        }

        operator float() const {
            return m_accel.m_rpss;
        }

        RpssProxy &operator=(const float rpss) {
            m_accel.m_rpss = rpss;
            m_accel.m_tpss = rpss / (2.0f * PI); // 弧度转圈数
            m_accel.m_apss = m_accel.m_tpss * 360.0f;
            return *this;
        }
    };

    // 度/s²代理类
    class ApssProxy {
    private:
        Accel &m_accel;

    public:
        explicit ApssProxy(Accel &a) : m_accel(a) {
        }

        operator float() const {
            return m_accel.m_apss;
        }

        ApssProxy &operator=(const float apss) {
            m_accel.m_apss = apss;
            m_accel.m_tpss = apss / 360.0f; // 角度转圈数
            m_accel.m_rpss = m_accel.m_tpss * 2.0f * PI;
            return *this;
        }
    };

    // 圈/s²代理类
    class TpssProxy {
    private:
        Accel &m_accel;

    public:
        explicit TpssProxy(Accel &a) : m_accel(a) {
        }

        operator float() const {
            return m_accel.m_tpss;
        }

        TpssProxy &operator=(float tpss) {
            m_accel.m_tpss = tpss;
            m_accel.m_rpss = tpss * 2.0f * PI; // 圈数转弧度
            m_accel.m_apss = tpss * 360.0f; // 圈数转角度
            return *this;
        }
    };

public:
    // 代理成员声明
    RpssProxy rpss;
    ApssProxy apss;
    TpssProxy tpss;

    // 构造函数
    explicit Accel() : rpss(*this), apss(*this), tpss(*this) {
    }

    // 拷贝赋值运算符
    Accel &operator=(const Accel &other) {
        if (this != &other) {
            m_rpss = other.m_rpss;
            m_apss = other.m_apss;
            m_tpss = other.m_tpss;
        }
        return *this;
    }

    // 一元负号运算符重载：取相反数
    Accel operator-() const {
        Accel result;
        result.rpss = -rpss;
        result.apss = -apss;
        result.tpss = -tpss;
        return result;
    }
};
