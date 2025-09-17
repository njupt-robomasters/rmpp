#pragma once

#include <tuple>

float signed_sqrt(float x);

float solve_alpha(float fc, float fs);

float lowpass_filter(float x, float next, float alpha);

float clamp(float x, float max);

float clamp(float x, float min, float max);

std::tuple<float, float> rotate(float x, float y, float angle);
