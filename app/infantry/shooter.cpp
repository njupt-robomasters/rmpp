#include "shooter.hpp"

Shooter::Shooter(PID::param_t* shoot_pid_param) :
    m_shoot(1, 7),
    n630_1(2, 42),
    n630_2(2, 41) {
    // 设置拨弹电机PID参数
    m_shoot.SetPID(Motor::SPEED_MODE, Motor::CURRENT_TYPE, shoot_pid_param);

    // 设置波弹电机方向
    m_shoot.SetInvert(true);
}

void Shooter::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return; // 防止重复设置
    this->is_enable = is_enable;

    if (!is_enable) { // 失能复位摩擦轮和拨弹电机状态
        is_prepare_shoot = false;
        is_shoot = false;
    }

    m_shoot.SetEnable(is_enable);
    n630_1.SetEnable(is_enable);
    n630_2.SetEnable(is_enable);
}

void Shooter::OnLoop() {
    backwardCalc();
    forwardCalc();

    n630_1.OnLoop();
    n630_2.OnLoop();
    m_shoot.OnLoop();
}

void Shooter::forwardCalc() {
    // 摩擦轮
    if (is_prepare_shoot) {
        const UnitFloat<rpm> n630_speed = bullet_speed.ref / WHEEL_RADIUS;
        n630_1.SetSpeed(n630_speed);
        n630_2.SetSpeed(n630_speed);
    } else {
        n630_1.SetSpeed(0 * default_unit);
        n630_2.SetSpeed(0 * default_unit);
    }

    // 拨弹电机
    if (is_shoot) {
        const UnitFloat<rpm> shoot_speed = shoot_freq.ref.toFloat(Hz) / SHOOT_PRE_ROUND * rps;
        m_shoot.SetSpeed(shoot_speed);
    } else {
        m_shoot.SetSpeed(0 * default_unit);
    }
}

void Shooter::backwardCalc() {
    // 摩擦轮
    bullet_speed.measure1 = n630_1.speed.measure * WHEEL_RADIUS;
    bullet_speed.measure2 = n630_2.speed.measure * WHEEL_RADIUS;

    // 拨弹电机
    shoot_freq.measure = m_shoot.speed.measure.toFloat(rps) * SHOOT_PRE_ROUND * Hz;
}