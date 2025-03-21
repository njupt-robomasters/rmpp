#pragma once

#include "utils.hpp"

class Angle {
private:
    // 实际存储的成员变量
    float m_rad = 0; // 存储弧度值
    float m_degree = 0; // 存储角度值

    // 弧度代理类
    class RadProxy {
    private:
        Angle &m_angle;

    public:
        explicit RadProxy(Angle &angle) : m_angle(angle) {
        }

        operator float() const {
            return m_angle.m_rad; // 直接返回存储值
        }

        RadProxy &operator=(const float rad) {
            m_angle.m_rad = rad;
            m_angle.m_degree = rad / PI * 180.0f; // 更新角度存储值
            return *this;
        }
    };

    // 角度代理类
    class DegreeProxy {
    private:
        Angle &m_angle;

    public:
        explicit DegreeProxy(Angle &angle) : m_angle(angle) {
        }

        operator float() const {
            return m_angle.m_degree; // 直接返回存储值
        }

        DegreeProxy &operator=(const float degree) {
            m_angle.m_degree = degree;
            m_angle.m_rad = degree / 180.0f * PI; // 更新弧度存储值
            return *this;
        }
    };

public:
    // 代理成员声明
    RadProxy rad;
    DegreeProxy degree;

    // 构造函数
    explicit Angle() : rad(*this), degree(*this) {
    }

    // 拷贝赋值运算符
    Angle &operator=(const Angle &other) {
        if (this != &other) {
            m_rad = other.m_rad;
            m_degree = other.m_degree;
        }
        return *this;
    }

    // 一元负号运算符重载：取相反数
    Angle operator-() const {
        Angle result;
        result.m_rad = -m_rad; // 取弧度的相反数
        result.m_degree = -m_degree; // 取角度的相反数
        return result;
    }
};
