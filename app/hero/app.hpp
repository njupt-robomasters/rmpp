#pragma once

#include "chassis.hpp"
#include "gimbal.hpp"
#include "math/unit.hpp"
#include "misc.hpp"
#include "shooter.hpp"

// 遥控器使能：VT13 N、S 挡，或 FSi6X SWC 中、下挡；断联时不使能
inline bool rc_enable() {
    return (vt13.is_connect && (vt13.mode == VT13::N || vt13.mode == VT13::S)) ||
           (fsi6x.is_connect && (fsi6x.swc == FSi6X::MID || fsi6x.swc == FSi6X::DOWN));
}
