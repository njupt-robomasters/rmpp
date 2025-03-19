#pragma once

struct mit_t {
    float kp, kd, ff; // ff为前馈
};

float signed_sqrt(float x);

float solve_alpha(float fc, float fs);

void lowpass_filter(float &x, float next, float alpha);

float norm_angle(float angle);

float calc_angle_err(float ref, float measure);

void clamp(float &x, float max);

void clamp(float &x, float min, float max);

void rotate(float &x, float &y, float angle);
