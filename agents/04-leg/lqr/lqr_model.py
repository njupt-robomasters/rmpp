"""复现 SOLDIER_LEG_4/Leg_Lqr__4.m 的 A、B、K，并分析重心偏移下的稳态

方程 (3.11)-(3.15) 对加速度、角度、力矩都是线性的，直接数值组装，不需要符号求解
状态 [s ds phi dphi th_l dth_l th_r dth_r th_b dth_b]，输入 [Tw_l Tw_r Tb_l Tb_r]，u = -Kx
"""

import numpy as np
from scipy.linalg import solve_continuous_are

G = 9.81
BODY = dict(R_w=0.06, R_l=0.22, l_c=0.037, m_w=0.263, m_l=1.224, m_b=18.5, I_w=2504e-7, I_b=0.529, I_z=0.607)

# 腿长 → l_w（轮到腿质心）、l_b（机体转轴到腿质心）、I_l
LEG = np.array([
    [0.11, 0.0620, 0.02499030, 0.0480], [0.12, 0.0730, 0.02455817, 0.0470],
    [0.13, 0.0824, 0.02413735, 0.0476], [0.14, 0.0920, 0.02372806, 0.0480],
    [0.15, 0.1010, 0.02333041, 0.0490], [0.16, 0.1100, 0.02294442, 0.0500],
    [0.17, 0.1190, 0.02256999, 0.0510], [0.18, 0.1275, 0.02220695, 0.0525],
    [0.19, 0.1361, 0.02185496, 0.0539], [0.20, 0.1446, 0.02151357, 0.0554],
    [0.21, 0.1530, 0.02118212, 0.0570], [0.22, 0.1614, 0.02085969, 0.0586],
    [0.23, 0.1700, 0.02054500, 0.0600], [0.24, 0.1779, 0.02023626, 0.0621],
    [0.25, 0.1861, 0.01993092, 0.0639], [0.26, 0.1943, 0.01962521, 0.0657],
    [0.27, 0.2024, 0.01931342, 0.0676], [0.28, 0.2100, 0.01898641, 0.0700],
    [0.29, 0.2187, 0.01862845, 0.0713], [0.30, 0.2267, 0.01819599, 0.0733],
])

Q = np.diag([60, 20, 12000, 80, 22000, 20, 22000, 20, 40000, 300]).astype(float)
Q_INT_S = 0.0  # ∫s 状态权重，用于 LQI；0 表示不启用
R = np.diag([60, 60, 6, 6]).astype(float)


def model(ll, lr, p=BODY):
    """返回 A(10x10)、B(10x4)、E(10x1)；E 为机体重心水平偏移 d（m）的扰动列"""
    R_w, R_l, l_c, m_w, m_l, m_b, I_w, I_b, I_z = (p[k] for k in ("R_w", "R_l", "l_c", "m_w", "m_l", "m_b", "I_w", "I_b", "I_z"))
    _, l_wl, l_bl, I_ll = LEG[np.argmin(abs(LEG[:, 0] - ll))]
    _, l_wr, l_br, I_lr = LEG[np.argmin(abs(LEG[:, 0] - lr))]
    l_l, l_r = ll, lr

    # 每行：M·[dd_wl dd_wr dd_ll dd_lr dd_b] + Nth·[th_ll th_lr th_b] + NT·[Twl Twr Tbl Tbr] = 0
    c3 = m_w * R_w**2 + I_w + m_l * R_w**2 + m_b * R_w**2 / 2
    c4 = m_w * R_w * l_c + I_w * l_c / R_w + m_l * R_w * l_c
    c5 = I_z * R_w / (2 * R_l) + I_w * R_l / R_w
    M = np.array([
        [I_w * l_l / R_w + m_w * R_w * l_l + m_l * R_w * l_bl, 0, m_l * l_wl * l_bl - I_ll, 0, 0],
        [0, I_w * l_r / R_w + m_w * R_w * l_r + m_l * R_w * l_br, 0, m_l * l_wr * l_br - I_lr, 0],
        [-c3, -c3, -(m_l * R_w * l_wl + m_b * R_w * l_l / 2), -(m_l * R_w * l_wr + m_b * R_w * l_r / 2), 0],
        [c4, c4, m_l * l_wl * l_c, m_l * l_wr * l_c, -I_b],
        [c5, -c5, I_z * l_l / (2 * R_l), -I_z * l_r / (2 * R_l), 0],
    ])
    Nth = np.array([
        [(m_l * l_wl + m_b * l_l / 2) * G, 0, 0],
        [0, (m_l * l_wr + m_b * l_r / 2) * G, 0],
        [0, 0, 0],
        [0, 0, m_b * G * l_c],
        [0, 0, 0],
    ])
    NT = np.array([
        [-(1 + l_l / R_w), 0, 1, 0],
        [0, -(1 + l_r / R_w), 0, 1],
        [1, 1, 0, 0],
        [-l_c / R_w, -l_c / R_w, -1, -1],
        [-R_l / R_w, R_l / R_w, 0, 0],
    ])
    Minv = np.linalg.inv(M)
    J_A = -Minv @ Nth
    J_B = -Minv @ NT
    # 重心水平偏移 d 等效于 eqn4 重力项中 th_b 多出 d/l_c
    J_E = J_A[:, 2:3] / l_c

    def rows(J):
        out = np.zeros((10, J.shape[1]))
        out[1] = R_w * (J[0] + J[1]) / 2
        out[3] = R_w * (-J[0] + J[1]) / (2 * R_l) - l_l * J[2] / (2 * R_l) + l_r * J[3] / (2 * R_l)
        out[5], out[7], out[9] = J[2], J[3], J[4]
        return out

    A = np.zeros((10, 10))
    A[0, 1] = A[2, 3] = A[4, 5] = A[6, 7] = A[8, 9] = 1
    A[:, [4, 6, 8]] += rows(J_A)
    return A, rows(J_B), rows(J_E)


def lqr(A, B, Q=Q, R=R):
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def fit_c(path):
    """读取 C 头文件中的 LQR_FIT"""
    import re
    text = open(path, encoding="utf-8").read()
    body = text[text.index("LQR_FIT"):]
    nums = [float(x) for x in re.findall(r"-?\d+\.?\d*(?:e-?\d+)?(?=f)", body)[: 4 * 10 * 6]]
    return np.array(nums).reshape(4, 10, 6)


def fit_eval(P, ll, lr):
    t = np.array([1, ll, lr, ll * ll, ll * lr, lr * lr])
    return P @ t


if __name__ == "__main__":
    import sys
    np.set_printoptions(precision=4, suppress=True, linewidth=160)
    L = 0.18
    A, B, E = model(L, L)
    K = lqr(A, B)
    print(f"== K（L={L}，MATLAB 复现）==\n{K}")
    if len(sys.argv) > 1:
        Kc = fit_eval(fit_c(sys.argv[1]), L, L)
        print(f"== C 拟合值 ==\n{Kc}\n最大相对差 {np.max(abs(Kc - K)) / np.max(abs(K)):.3%}")
    print("闭环极点", np.sort_complex(np.linalg.eigvals(A - B @ K)))
