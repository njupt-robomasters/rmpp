#pragma once

#include "motor/gm6020.hpp"

static GM6020 yaw({
                      .bus = {.can = Can(2), .rx_id = 0x205},
                      .drive = {.max_current = GM6020::MAX_CURRENT, .max_torque = GM6020::MAX_TORQUE, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
                      .install = {.is_invert = false},
                  },
                  {});

static GM6020 pitch({
                        .bus = {.can = Can(2), .rx_id = 0x206},
                        .drive = {.max_current = GM6020::MAX_CURRENT, .max_torque = GM6020::MAX_TORQUE, .kt = GM6020::KT, .reduction = GM6020::REDUCTION},
                        .install = {.is_invert = false},
                    },
                    {});
