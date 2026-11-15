"""按 lqr_model.py 中的机体参数与 Q、R 生成 LQR/LQI 增益拟合系数，写入 app/leg_down/lqr_fit.hpp

LQR（Q_INT_S=0）：10 状态，增益矩阵 [4×10]，poly22 拟合为 [4][10][6]
LQI（Q_INT_S>0）：增广 ∫s 第 11 状态，增益矩阵 [4×11]，拟合为 [4][11][6]，第 11 列为 K_∫s

与 Leg_Lqr__4.m 第 3 步相同：腿长表两两组合求 K，对每个 K_ij 做 poly22 最小二乘拟合
用法：python gen_lqr_fit.py [输出路径]
"""

import sys
from pathlib import Path
import numpy as np
from scipy.linalg import solve_continuous_are
from lqr_model import BODY, LEG, Q, Q_INT_S, R, model, lqr, fit_eval

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[3] / "app/leg_down/lqr_fit.hpp"
OUTPUTS = ["左轮力矩", "右轮力矩", "左摆动力矩", "右摆动力矩"]
USE_LQI = Q_INT_S > 0
N_STATES = 11 if USE_LQI else 10


def lqi(A, B):
    """LQI：增广第 11 状态 z = ∫s，返回 (4×11) 增益矩阵"""
    Aa = np.zeros((11, 11)); Aa[:10, :10] = A; Aa[10, 0] = 1
    Ba = np.vstack([B, np.zeros((1, 4))])
    Qa = np.zeros((11, 11)); Qa[:10, :10] = Q; Qa[10, 10] = Q_INT_S
    P = solve_continuous_are(Aa, Ba, Qa, R)
    return np.linalg.solve(R, Ba.T @ P)


def fit():
    X, Y = [], []
    for ll in LEG[:, 0]:
        for lr in LEG[:, 0]:
            A, B, _ = model(ll, lr)
            X.append([1, ll, lr, ll * ll, ll * lr, lr * lr])
            K = lqi(A, B) if USE_LQI else lqr(A, B)
            Y.append(K.ravel())
    X, Y = np.array(X), np.array(Y)
    P = np.linalg.lstsq(X, Y, rcond=None)[0].T.reshape(4, N_STATES, 6)
    err = np.max(abs(X @ P.reshape(4 * N_STATES, 6).T - Y)) / np.max(abs(Y))
    return P, err


def num(v):
    s = f"{v:.6g}"
    if "." not in s and "e" not in s:
        s += ".0"
    return s + "f"


def main():
    P, err = fit()
    body = "、".join(f"{k} = {v:g}" for k, v in BODY.items())
    mode = f"LQI Q_∫s={Q_INT_S:g}" if USE_LQI else "LQR"
    state_comment = (
        "// 状态：位移、速度、航向角、航向角速度、左腿倾角及角速度、右腿倾角及角速度、俯仰及角速度、∫位移"
        if USE_LQI else
        "// 状态：位移、速度、航向角、航向角速度、左腿倾角及角速度、右腿倾角及角速度、俯仰及角速度"
    )
    lines = [
        "#pragma once",
        "",
        f"// LQR 增益拟合系数 [输出][状态][项]，由 agents/tasks/09-leg_lqr/gen_lqr_fit.py 生成（{mode}），不手改",
        "// 输出：左轮、右轮（前进为正）、左摆、右摆（后摆为正），N·m",
        state_comment,
        "// 项：1、左腿长、右腿长、左腿长²、左×右腿长、右腿长²，腿长单位 m",
        f"// 机体：{body}（SI 单位）",
        f"// Q = diag({', '.join(f'{q:g}' for q in np.diag(Q))})" + (f"  Q_∫s = {Q_INT_S:g}" if USE_LQI else ""),
        f"// R = diag({', '.join(f'{r:g}' for r in np.diag(R))})",
        f"// 腿长 {LEG[0, 0]:g}–{LEG[-1, 0]:g} m，拟合最大相对误差 {err:.2%}",
        f"inline constexpr float LQR_FIT[4][{N_STATES}][6] = {{",
    ]
    for i in range(4):
        lines += ["    {", f"        // {OUTPUTS[i]}"]
        lines += [f"        {{{', '.join(num(v) for v in P[i, j])}}}," for j in range(N_STATES)]
        lines.append("    },")
    lines += ["};", ""]
    OUT.write_text("\n".join(lines), encoding="utf-8", newline="\n")

    np.set_printoptions(precision=4, suppress=True, linewidth=160)
    print(f"写入 {OUT}（{mode}，{N_STATES} 状态），拟合最大相对误差 {err:.2%}")
    t = np.array([1, .18, .18, .18*.18, .18*.18, .18*.18])
    print(f"L=0.18 时 K：\n{P @ t}")


if __name__ == "__main__":
    main()
