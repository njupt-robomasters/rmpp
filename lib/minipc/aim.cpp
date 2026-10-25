#include "aim.hpp"

Aim::Aim(const Config& config)
    : config(config) {}

void Aim::update() {
    if (timer_vision_.elapsed() > config.timeout) {
        timer_vision_.restart();
        is_connect_vision = false;
        vision = {};
    }
    if (timer_insta360_.elapsed() > config.timeout) {
        timer_insta360_.restart();
        insta360 = {};
    }
}
