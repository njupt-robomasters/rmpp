#include "aim.hpp"

Aim::Aim(const Config& config)
    : config_(config) {}

void Aim::update() {
    if (vision_timer_.elapsed() > config_.timeout) {
        vision_timer_.restart();
        is_connect_vision = false;
        vision = {};
    }
    if (insta360_timer_.elapsed() > config_.timeout) {
        insta360_timer_.restart();
        insta360 = {};
    }
}
