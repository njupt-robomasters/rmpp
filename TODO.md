set_mit接口
0307 0308
磕台阶
电机状态/错误
并入老哨兵
RUB_RELEASE_SPEED写到vesc库里
轮腿位移*10

轮腿lqr参数放在 robot 下，用 chassis_leg_xxx.hpp 命名，chassis_leg.cpp 直接设置默认参数
深度优化 [chassis_leg.cpp](lib/robot/chassis_leg.cpp) .hpp 大代码，包括命名，顺序，注释

map_force 用 vmc 类似的词，他本质是 vmc 虚拟模型控制

update 里面的东西也比较乱，要整理

    // 倒地与自救
    enum class Fall {
        UPRIGHT, // 车体正向
        HEAD,    // 头部着地
        TAIL,    // 尾部着地
    };
能否放在自救结构体里

        bool is_fallen = false;    // 倒地锁存，站起完成、失能或跳跃时清除
        bool is_running = false;   // 自救运行中

        bool is_active = false; // 跳跃运行中
        int step = 0;           // 0 起跳、1 收腿、2 收腿保持、3 落地缓冲

这些能否也用一个枚举来维护，有个0表示的状态是空闲，这样其他地方协作的时候好判断

我想 solve_leg 应该包括下面的，可以通过

    // 关节角转杆方位角：电机角逆时针为正、杆方位角顺时针为正，零位对应 phi1 = π、phi4 = 0
    leg[0].phi1 = pi - motor.left_back.angle;
    leg[0].phi4 = 0 - motor.left_front.angle;
    leg[1].phi1 = pi - motor.right_front.angle;
    leg[1].phi4 = 0 - motor.right_back.angle;
    leg[0].d_phi1 = -motor.left_back.speed;
    leg[0].d_phi4 = -motor.left_front.speed;
    leg[1].d_phi1 = -motor.right_front.speed;
    leg[1].d_phi4 = -motor.right_back.speed;
    solve_leg(leg[0]);
    solve_leg(leg[1]);

    // 相对腿角：左右腿镜像安装，分别反向、同向换算为车体正下方为零、后摆为正
    leg[0].rel_theta = Angle(-leg[0].phi0 + pi / 2);
    leg[1].rel_theta = Angle(leg[1].phi0 - pi / 2);
    // 腿倾角：叠加车体俯仰，低头时轮轴相对竖直方向后摆
    leg[0].theta = Angle(leg[0].rel_theta + body.pitch);
    leg[1].theta = Angle(leg[1].rel_theta + body.pitch);
    leg[0].d_theta = -leg[0].d_phi0 + body.d_pitch;
    leg[1].d_theta = leg[1].d_phi0 + body.d_pitch;

    for (auto& l : leg) {
        // 重力取非负沿腿分量，气弹簧取等效伸腿力幅值
        l.Fg = std::max(config.mass * 9.8f / 2 * std::cos(l.theta), 0.0f);
        l.Fs = std::fabs(0.5f * config.spring_force * std::sin(l.phi_obc) * std::sin(pi - 0.5f * l.phi_bod - l.phi_obc));

        // 腿摆动引起的前向速度补偿
        l.v_swing = l.d_L * std::sin(l.theta) + l.L * l.d_theta * std::cos(l.theta);
    }

    // 轮速：电机安装方向已由 is_invert 修正为前进为正
    leg[0].v_wheel = motor.left_wheel.speed * config.wheel_radius;
    leg[1].v_wheel = motor.right_wheel.speed * config.wheel_radius;