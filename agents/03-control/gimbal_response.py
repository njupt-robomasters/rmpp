"""云台速度内环 / 位置外环频率响应计算。

参数取自 app/leg/leg_up/gimbal.hpp 与 lib/motor/gm6020.hpp。
用法：python -I agents/03-control/gimbal_response.py
"""

import cmath
import math

# GM6020：3 A 对应 2.223 N·m，直驱（reduction = 1）
KT = 2.223 / 3.0  # N·m/A

SAMPLE_DT = 1e-3  # 控制周期，s
FRAME_DT = 1.0 / 900  # yaw/pitch 电流帧周期，s
DELAY = SAMPLE_DT + FRAME_DT  # 一周期延迟 + 半周期 ZOH 近似

FREQS = [1.0, 3.0, 5.0]


def speed_closed(s, kp, ki, j):
    """速度环闭环 T_v(s)，反馈取自 IMU 陀螺。"""
    c = kp + ki / s
    g = KT / (j * s)
    return c * g / (1 + c * g)


def pos_closed(s, kp_pos, kp_v, ki_v, j, delay=DELAY):
    """位置闭环 T_p(s)：外环 P 输出世界角速度，串在速度环前。"""
    plant = speed_closed(s, kp_v, ki_v, j) * cmath.exp(-s * delay)
    loop = kp_pos * plant / s
    return loop / (1 + loop), loop  # 闭环、开环


def pos_open_delayless(s, kp_pos, j):
    """速度环理想（T_v = 1）时的开环，作对照。"""
    return kp_pos / s


def db(x):
    return 20 * math.log10(x)


def crossover(fn, lo=0.1, hi=500.0):
    """幅值穿越频率，rad/s。"""
    f_lo, f_hi = lo, hi
    for _ in range(200):
        f_mid = math.sqrt(f_lo * f_hi)
        if abs(fn(f_mid)) > 1:
            f_lo = f_mid
        else:
            f_hi = f_mid
    return math.sqrt(f_lo * f_hi)


def speed_open(s, kp, ki, j, delay=DELAY):
    """速度环开环 L_v(s) = C_v·kt/(J s)。"""
    return (kp + ki / s) * KT / (j * s) * cmath.exp(-s * delay)


def report(title, kp_pos, kp_v, ki_v, j, delay=DELAY):
    print(f"\n{title}  J={j} kg·m²  kp_v={kp_v} ki_v={ki_v} kp_pos={kp_pos}")
    loop_fn = lambda w: pos_closed(1j * w, kp_pos, kp_v, ki_v, j, delay)[1]
    wc = crossover(loop_fn)
    ph = math.atan2(loop_fn(wc).imag, loop_fn(wc).real)
    print(f"  位置环穿越 {wc:.1f} rad/s ({wc / 2 / math.pi:.2f} Hz)，相位裕度 {180 + math.degrees(ph):.0f}°")

    speed_fn = lambda w: speed_open(1j * w, kp_v, ki_v, j)
    wcv = crossover(speed_fn, 1.0, 5000)
    phv = math.atan2(speed_fn(wcv).imag, speed_fn(wcv).real)
    print(f"  速度环穿越 {wcv:.1f} rad/s ({wcv / 2 / math.pi:.1f} Hz)，"
          f"相位裕度 {180 + math.degrees(phv):.0f}°，积分零点 {ki_v / kp_v:.2f} rad/s "
          f"(穿越的 {ki_v / kp_v / wcv:.2f} 倍)")

    print("  f/Hz   |H|     dB     相位/°   1 Hz 参考相位")
    for f in FREQS:
        w = 2 * math.pi * f
        h = pos_closed(1j * w, kp_pos, kp_v, ki_v, j, delay)[0]
        h0 = pos_open_delayless(1j * w, kp_pos, j) / (1 + pos_open_delayless(1j * w, kp_pos, j))
        print(f"  {f:4.1f}  {abs(h):6.3f}  {db(abs(h)):+6.2f}  "
              f"{math.degrees(cmath.phase(h)):7.1f}   {math.degrees(cmath.phase(h0)):7.1f}")

    # 力矩阶跃下的最大角加速度与正弦跟踪幅值上限
    a_max = KT * 3.0 / j
    print(f"  电流限幅 3 A → 最大角加速度 {a_max:.0f} rad/s²；"
          f"5 Hz 可跟踪幅值 {a_max / (2 * math.pi * 5) ** 2 * 180 / math.pi:.2f}°")
    return wc


def main():
    print(f"kt = {KT:.3f} N·m/A，延迟 {DELAY * 1e3:.2f} ms")

    for j in (0.01, 0.02, 0.05, 0.10):
        report("yaw 现状", 30.0, 2.4, 8.0, j)

    print("\n=== pitch（速度环无积分 kp=5.0）===")
    for j in (0.005, 0.01, 0.02):
        report("pitch 现状", 30.0, 5.0, 0.0, j)

    # 速度环零极点与积分时间
    print("\n=== 速度环结构量 ===")
    for j in (0.01, 0.02, 0.05):
        a = KT * 2.4 / j
        b = KT * 8.0 / j
        disc = a * a / 4 - b
        poles = [(-a / 2 + math.sqrt(disc), "实"), (-a / 2 - math.sqrt(disc), "实")] if disc >= 0 else []
        print(f"  J={j}: kt·kp/J={a:.1f} rad/s, kt·ki/J={b:.1f} rad²/s², "
              f"pi 零点 ki/kp={8.0 / 2.4:.2f} rad/s, 阻尼比 {a / (2 * math.sqrt(b)):.2f}")

    # 按目标相位裕度反推内环 kp
    print("\n=== 位置环 kp 与内环带宽的关系（J=0.02, 0.05，PM≥60°）===")
    for j in (0.02, 0.05):
        for kp_v in (2.4, 4.0, 8.0, 16.0):
            ki_v = kp_v * 8.0 / 2.4
            loop_fn = lambda w: pos_closed(1j * w, 30.0, kp_v, ki_v, j, DELAY)[1]
            wc = crossover(loop_fn)
            pm = 180 + math.degrees(math.atan2(loop_fn(wc).imag, loop_fn(wc).real))
            h5 = pos_closed(1j * 2 * math.pi * 5, 30.0, kp_v, ki_v, j, DELAY)[0]
            print(f"  J={j} kp_v={kp_v:4.1f} ki_v={ki_v:5.1f}: 穿越 {wc:5.1f} rad/s，PM {pm:4.0f}°，"
                  f"5 Hz 增益 {db(abs(h5)):+5.2f} dB，相位 {math.degrees(cmath.phase(h5)):6.1f}°")


def torque_table():
    """正弦跟踪所需峰值力矩，N·m；J·A·ω²。"""
    print("\n=== 正弦跟踪峰值力矩（J·A·ω²，含速度环理想跟踪）===")
    print("  J     f/Hz   A=1°    A=3°    A=5°     电流(5°)")
    for j in (0.02, 0.05):
        for f in FREQS:
            w = 2 * math.pi * f
            row = [j * math.radians(a) * w * w for a in (1, 3, 5)]
            print(f"  {j:.2f}  {f:4.1f}  {row[0]:6.2f}  {row[1]:6.2f}  {row[2]:6.2f}   "
                  f"{row[2] / KT:5.2f} A / 3 A")


def kp_pos_sweep():
    """固定内环，扫位置环 kp：穿越频率、相位裕度与 5 Hz 响应。"""
    print("\n=== 位置环 kp 扫描（内环 kp_v=4.0 ki_v=13.3）===")
    for j in (0.02, 0.05):
        for kp_pos in (15.0, 30.0, 45.0, 60.0, 90.0):
            loop_fn = lambda w: pos_closed(1j * w, kp_pos, 4.0, 13.3, j, DELAY)[1]
            wc = crossover(loop_fn)
            pm = 180 + math.degrees(math.atan2(loop_fn(wc).imag, loop_fn(wc).real))
            h5 = pos_closed(1j * 2 * math.pi * 5, kp_pos, 4.0, 13.3, j, DELAY)[0]
            print(f"  J={j} kp_pos={kp_pos:4.1f}: 穿越 {wc:5.1f} rad/s，PM {pm:4.0f}°，"
                  f"5 Hz {db(abs(h5)):+5.2f} dB / {math.degrees(cmath.phase(h5)):6.1f}°")


def step_response():
    """速度指令阶跃：63% 上升时间、峰值与峰值时刻。"""
    print("\n=== 速度环单位阶跃（kp_v=2.4 ki_v=8.0）===")
    for j in (0.01, 0.02, 0.05, 0.10):
        w = 0.0  # 归一化速度
        i = 0.0  # 积分输出，A
        dt = 1e-6
        t63 = None
        peak, t_peak = 0.0, 0.0
        for n in range(int(1.5 / dt)):
            e = 1.0 - w
            dw = KT * (2.4 * e + i) / j
            w += dw * dt
            i += 8.0 * e * dt
            t = (n + 1) * dt
            if t63 is None and w >= 0.63:
                t63 = t
            if w > peak:
                peak, t_peak = w, t
        print(f"  J={j:.2f}: 63% 上升 {t63 * 1e3:5.1f} ms，峰值 {peak * 100:5.1f}%，"
              f"峰值时刻 {t_peak * 1e3:5.1f} ms，纯 P 时间常数 J/(kt·kp)={j / (KT * 2.4) * 1e3:.1f} ms，"
              f"初始斜率 kt·kp/J={KT * 2.4 / j:.0f} rad/s²")


def gain_compare():
    """同一 J 下，kp、ki 各自的作用：阶跃响应与穿越频率。"""
    print("\n=== kp/ki 组合对比（J=0.02，速度环）===")
    j = 0.02
    for kp, ki in ((2.4, 8.0), (7.2, 24.0), (2.4, 24.0), (7.2, 8.0)):
        w, i, dt = 0.0, 0.0, 1e-6
        t63, peak, t_peak = None, 0.0, 0.0
        for n in range(int(1.0 / dt)):
            e = 1.0 - w
            w += KT * (kp * e + i) / j * dt
            i += ki * e * dt
            t = (n + 1) * dt
            if t63 is None and w >= 0.63:
                t63 = t
            if w > peak:
                peak, t_peak = w, t
        # 2% 调节时间
        ts = None
        for n in range(int(1.0 / dt)):
            pass
        lf = lambda w_: speed_open(1j * w_, kp, ki, j)
        wc = crossover(lf, 1.0, 5000)
        pm = 180 + math.degrees(math.atan2(lf(wc).imag, lf(wc).real))
        # 0.01 rad/s 处的扰动抑制：|S| = 1/|1+L|
        s_lf = 1 / abs(1 + lf(0.01))
        print(f"  kp={kp:4.1f} ki={ki:5.1f} (零点 {ki / kp:5.2f} rad/s): "
              f"t63 {t63 * 1e3:5.1f} ms，峰值 {peak * 100:5.1f}% @{t_peak * 1e3:5.1f} ms，"
              f"穿越 {wc:5.1f} rad/s，PM {pm:4.0f}°，0.01 rad/s 扰动抑制 {s_lf * 100:6.1f}%")


if __name__ == "__main__":
    main()
    torque_table()
    kp_pos_sweep()
    step_response()
    gain_compare()
