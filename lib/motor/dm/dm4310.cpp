#include "dm4310.hpp"

DM4310::DM4310(const Config& config)
    : DmMotor({.motor = config.motor.with_defaults(DRIVE), .protocol = PROTOCOL, .enable_frequency = config.enable_frequency}) {}
