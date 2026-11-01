#pragma once

#include "chassis.hpp"
#include "rc/fsi6x.hpp"

inline FSi6X fsi6x({.uart = Uart(3)});
