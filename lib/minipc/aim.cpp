#include "aim.hpp"

Aim::Aim(const Config& config)
    : config_(config) {}

void Aim::update() {
    if (timer_vision_.elapsed() > config_.timeout) {
        timer_vision_.restart();
        is_connect_vision = false;
        vision = {};
    }
    if (timer_insta360_.elapsed() > config_.timeout) {
        timer_insta360_.restart();
        insta360 = {};
    }
}
