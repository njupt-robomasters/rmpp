#include "aim.hpp"

Aim::Aim(const Config& config)
    : config(config) {}

void Aim::update() {
    if (vision_timer.elapsed() > config.timeout) {
        vision_timer.restart();
        is_connect_vision = false;
        vision = {};
    }
    if (insta360_timer.elapsed() > config.timeout) {
        insta360_timer.restart();
        insta360 = {};
    }
}
