#pragma once

#include "rc/fsi6x.hpp"
#include "melody/melody.hpp"

inline Melody melody({});
inline FSi6X fsi6x({.uart = Uart(3)});
