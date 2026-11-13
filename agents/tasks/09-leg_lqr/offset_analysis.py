"""机体重心水平偏移 d 下的稳态：按本工程实际用法（C 拟合增益、位移限幅 x_max）

用法：python offset_analysis.py ../../../app/leg_down/lqr_fit.hpp
"""

import sys
import numpy as np
from lqr_model import model, lqr, fit_c, fit_eval

L = 0.18
X_MAX = 0.3  # chassis_leg.hpp x_max
A, B, E = model(L, L)
Kc = fit_eval(fit_c(sys.argv[1]), L, L)


def steady(K, d):
    """无限幅稳态 x = -(A-BK)^-1 E d"""
    return -np.linalg.solve(A - B @ K, E[:, 0] * d)


def drift(K, d, s_clamp):
    """位移被限幅在 s_clamp 时的稳态：s 不再反馈，求其余状态，ds 可为非零常数"""
    idx = list(range(1, 10))
    Acl = (A - B @ K)[np.ix_(idx, idx)]
    rhs = -(E[idx, 0] * d - (B @ K)[idx, 0] * s_clamp)
    return np.linalg.solve(Acl, rhs)


def report(name, K, x_max=X_MAX):
    x = steady(K, 0.01)
    d_lim = x_max / abs(x[0]) * 0.01
    print(f"[{name}] K_s={K[0,0]:.2f}  每 1 cm 重心偏移: s 稳态 {x[0]*100:.1f} cm, 腿角 {np.degrees(x[4]):.2f}°, 俯仰 {np.degrees(x[8]):.2f}°；"
          f"位移限幅 {x_max*100:.0f} cm 可抵抗的最大偏移 {d_lim*1000:.1f} mm")
    for d in (0.005, 0.01, 0.02):
        s = steady(K, d)[0]
        if abs(s) > x_max:
            v = drift(K, d, np.sign(s) * x_max)[0]
            print(f"    d={d*1000:.0f} mm: 超限 → 持续漂移 {v*100:.1f} cm/s")


np.set_printoptions(precision=3, suppress=True)
report("C 增益", Kc)
eig = np.linalg.eigvals(A - B @ Kc)
print("当前闭环最慢极点", eig[np.argsort(abs(eig.real))][:2])
