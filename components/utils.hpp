#pragma once

#include <cmath>
#include <tuple>

static constexpr float PI = M_PI;

float signed_sqrt(float x);

float solve_alpha(float fc, float fs);

float lowpass_filter(float x, float next, float alpha);

float norm_angle(float degree);

float calc_angle_err(float ref_degree, float measure_degree);

float clamp(float x, float max);

float clamp(float x, float min, float max);

std::tuple<float, float> rotate(float x, float y, float angle);
