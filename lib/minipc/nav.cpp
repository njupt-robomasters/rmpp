#include "nav.hpp"

Nav::Nav(const Config& config)
    : config(config) {}

void Nav::update() {
    if (timer_chassis_speed_.elapsed() > config.timeout) {
        timer_chassis_speed_.restart();
        chassis_speed = {};
    }
}
