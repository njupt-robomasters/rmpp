#pragma once

#include <tuple>

float signed_sqrt(float x);

float solve_alpha(float fc, float fs);

float lowpass_filter(float x, float next, float alpha);

std::tuple<float, float> rotate(float x, float y, float angle);
