#pragma once

#include "melody/melody.hpp"
#include "rc/vt13.hpp"

inline Melody melody({});
inline VT13 vt13({.uart = Uart(6)});
