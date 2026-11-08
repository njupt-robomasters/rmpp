#include "nav.hpp"

Nav::Nav(const Config& config)
    : config_(config) {}

void Nav::update() {
    if (chassis_speed_timer_.elapsed() > config_.timeout) {
        chassis_speed_timer_.restart();
        chassis_speed = {};
    }
}
