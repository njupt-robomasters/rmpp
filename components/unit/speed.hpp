#pragma once

#include "utils.hpp"

class Speed {
private:
    // 实际存储的成员变量
    float m_rpm = 0; // 转/分钟
    float m_tps = 0; // 转/秒
    float m_aps = 0; // 角度/秒
    float m_rps = 0; // 弧度/秒

    // RPM代理类
    class RpmProxy {
    private:
        Speed &m_speed;

    public:
        explicit RpmProxy(Speed &speed) : m_speed(speed) {
        }

        operator float() const {
            return m_speed.m_rpm;
        }

        RpmProxy &operator=(const float rpm) {
            m_speed.m_rpm = rpm;
            m_speed.m_tps = rpm / 60.0f;
            m_speed.m_aps = m_speed.m_tps * 360.0f;
            m_speed.m_rps = m_speed.m_tps * 2.0f * PI;
            return *this;
        }
    };

    // TPS代理类
    class TpsProxy {
    private:
        Speed &m_speed;

    public:
        explicit TpsProxy(Speed &speed) : m_speed(speed) {
        }

        operator float() const {
            return m_speed.m_tps;
        }

        TpsProxy &operator=(const float tps) {
            m_speed.m_tps = tps;
            m_speed.m_rpm = tps * 60.0f;
            m_speed.m_aps = tps * 360.0f;
            m_speed.m_rps = tps * 2.0f * PI;
            return *this;
        }
    };

    // APS代理类
    class ApsProxy {
    private:
        Speed &m_speed;

    public:
        explicit ApsProxy(Speed &speed) : m_speed(speed) {
        }

        operator float() const {
            return m_speed.m_aps;
        }

        ApsProxy &operator=(const float aps) {
            m_speed.m_aps = aps;
            m_speed.m_tps = aps / 360.0f;
            m_speed.m_rpm = m_speed.m_tps * 60.0f;
            m_speed.m_rps = m_speed.m_tps * 2.0f * PI;
            return *this;
        }
    };

    // RPS代理类
    class RpsProxy {
    private:
        Speed &m_speed;

    public:
        explicit RpsProxy(Speed &speed) : m_speed(speed) {
        }

        operator float() const {
            return m_speed.m_rps;
        }

        RpsProxy &operator=(const float rps) {
            m_speed.m_rps = rps;
            m_speed.m_tps = rps / (2.0f * PI);
            m_speed.m_rpm = m_speed.m_tps * 60.0f;
            m_speed.m_aps = m_speed.m_tps * 360.0f;
            return *this;
        }
    };

public:
    // 代理成员声明
    RpmProxy rpm;
    TpsProxy tps;
    ApsProxy aps;
    RpsProxy rps;

    // 构造函数
    explicit Speed() : rpm(*this), tps(*this), aps(*this), rps(*this) {
    }

    // 拷贝赋值运算符
    Speed& operator=(const Speed& other) {
        if (this != &other) {
            m_rpm = other.m_rpm;
            m_tps = other.m_tps;
            m_aps = other.m_aps;
            m_rps = other.m_rps;
        }
        return *this;
    }

    // 一元负号运算符重载：取相反数
    Speed operator-() const {
        Speed result;
        result.rpm = -rpm;
        result.tps = -tps;
        result.aps = -aps;
        result.rps = -rps;
        return result;
    }

    Speed& operator*=(const float ratio) {
        m_rpm *= ratio;
        m_tps *= ratio;
        m_aps *= ratio;
        m_rps *= ratio;
        return *this;
    }

    Speed& operator/=(const float ratio) {
        m_rpm /= ratio;
        m_tps /= ratio;
        m_aps /= ratio;
        m_rps /= ratio;
        return *this;
    }
};
