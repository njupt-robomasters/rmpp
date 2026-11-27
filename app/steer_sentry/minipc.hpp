#pragma once

#include "minipc/aim.hpp"
#include "minipc/nav.hpp"

// 上位机：C 板只有一路 CDC，自瞄与导航共用；send() 由 app.cpp 以 200 Hz 轮流调用
inline Aim aim({.cdc = Cdc(1)});
inline Nav nav({.cdc = Cdc(1)});

void minipc_loop();
