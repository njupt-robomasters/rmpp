#include "nav.hpp"

Nav::Nav(const Config& config)
    : config(config) {}

void Nav::update() {
    if (chassis_speed_timer.elapsed() > config.timeout) {
        chassis_speed_timer.restart();
        chassis_speed = {};
    }
}
