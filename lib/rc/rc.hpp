#pragma once

#include "fsi6x.hpp"
#include "vt13.hpp"

class Rc {
  public:
    bool is_enable = false;

    bool is_rub = false, is_shoot = false, is_auto_aim = false, is_fn = false, is_calib = false;
    float x{}, y{}, r{}, pitch{}, yaw{}, shoot{};

    VT13::Mouse mouse;
    VT13::Key key;

    Rc(const FSi6X::Config& fsi6x_config, const VT13::Config& vt13_config);

    // 需要在循环中调用
    void on_loop();

  private:
    VT13 vt13_;
    FSi6X fsi6x_;
};
