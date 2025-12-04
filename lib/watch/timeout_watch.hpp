#pragma once

#include "bsp/bsp.hpp"

template <typename T>
class TimeoutWatch {
public:
    TimeoutWatch(const float timeout, T timeout_value) : timeout(timeout), timeout_value(timeout_value), m_value(timeout_value) {}

    T value() {
        if (BSP::Dwt::GetTime() - m_last_time > timeout) {
            m_value = timeout_value;
        }
        return m_value;
    }

    // 取值运算
    operator T() {
        return value();
    }

    // 赋值运算
    TimeoutWatch& operator=(const T& m_value) {
        this->m_value = m_value;
        this->m_last_time = BSP::Dwt::GetTime();
        return *this;
    }

    TimeoutWatch& operator=(const TimeoutWatch& other) {
        this->m_value = other.m_value;
        this->m_last_time = BSP::Dwt::GetTime();
        return *this;
    }

private:
    const float timeout;
    const T timeout_value;

    T m_value;
    float m_last_time = 0;
};
