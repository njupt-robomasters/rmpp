#include "nav.hpp"

Nav::Nav(const Config& config)
    : timeout(config.timeout) {}

void Nav::update() {
    if (chassis_speed_timer.elapsed() > timeout) {
        chassis_speed_timer.restart();
        chassis_speed = {};
    }
}
