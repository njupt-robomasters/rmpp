#pragma once

#include "minipc/aim.hpp"

// 自瞄：send() 由 app.cpp 以 200 Hz 调用
inline Aim aim({.cdc = Cdc(1)});

void minipc_loop();
