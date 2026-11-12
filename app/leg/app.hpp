#pragma once

#include "melody/melody.hpp"
#include "rc/fsi6x.hpp"

inline Melody melody({});
inline FSi6X fsi6x({.uart = Uart(3)});
