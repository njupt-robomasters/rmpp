#pragma once

#include "bsp/bsp.hpp"

template <typename T>
class TimeoutWatch {
public:
    TimeoutWatch(const float timeout, T timeout_value) : timeout(timeout), timeout_value(timeout_value), m_value(timeout_value) {}

    T value() {
        if (BSP::Dwt::GetTime() - m_last_time_f64 > timeout) {
            m_value = timeout_value;
        }
        return m_value;
    }

    // 取值运算
    operator T() {
        return value();
    }

    // 赋值运算
    TimeoutWatch& operator=(const T& value) {
        m_value = value;
        m_last_time_f64 = BSP::Dwt::GetTime();
        return *this;
    }

    TimeoutWatch& operator=(const TimeoutWatch& other) {
        m_value = other.m_value;
        m_last_time_f64 = BSP::Dwt::GetTime();
        return *this;
    }

private:
    const float timeout;
    const T timeout_value;

    T m_value;
    double m_last_time_f64 = 0;
};