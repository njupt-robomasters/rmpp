#pragma once

#include "Chassis_Omni.hpp"
#include "motor/Motor.hpp"

class Chassis_Mecanum : public Chassis_Omni {
public:
    Chassis_Mecanum(const config_t& config, const motor_t& motor, const PID::config_t* vxyz_pid_config);

private:
    void forward() override;

    void backward() override;
};