#include "aim.hpp"

Aim::Aim(const Config& config)
    : timeout(config.timeout) {}

void Aim::update() {
    if (vision_timer.elapsed() > timeout) {
        vision_timer.restart();
        is_connect_vision = false;
        vision = {};
    }
    if (insta360_timer.elapsed() > timeout) {
        insta360_timer.restart();
        insta360 = {};
    }
}
