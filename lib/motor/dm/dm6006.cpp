#include "dm6006.hpp"

DM6006::DM6006(const Config& config)
    : DmMotor({.motor = config.motor.with_defaults(DRIVE), .protocol = PROTOCOL, .enable_frequency = config.enable_frequency}) {}
