"""按 lqr_model.py 中的机体参数与 Q、R 生成 LQR 增益二次拟合系数，写入 app/leg_down/lqr_fit.hpp

与 Leg_Lqr__4.m 第 3 步相同：腿长表两两组合求 K，对每个 K_ij 做 poly22 最小二乘拟合
用法：python gen_lqr_fit.py [输出路径]
"""

import sys
from pathlib import Path
import numpy as np
from lqr_model import BODY, LEG, Q, R, model, lqr, fit_eval

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[3] / "app/leg_down/lqr_fit.hpp"
OUTPUTS = ["左轮力矩", "右轮力矩", "左摆动力矩", "右摆动力矩"]


def fit():
    X, Y = [], []
    for ll in LEG[:, 0]:
        for lr in LEG[:, 0]:
            A, B, _ = model(ll, lr)
            X.append([1, ll, lr, ll * ll, ll * lr, lr * lr])
            Y.append(lqr(A, B).ravel())
    X, Y = np.array(X), np.array(Y)
    P = np.linalg.lstsq(X, Y, rcond=None)[0].T.reshape(4, 10, 6)
    err = np.max(abs(X @ P.reshape(40, 6).T - Y)) / np.max(abs(Y))
    return P, err


def num(v):
    s = f"{v:.6g}"
    if "." not in s and "e" not in s:
        s += ".0"
    return s + "f"


def main():
    P, err = fit()
    body = "、".join(f"{k} = {v:g}" for k, v in BODY.items())
    lines = [
        "#pragma once",
        "",
        "// LQR 增益拟合系数 [输出][状态][项]，由 agents/tasks/09-leg_lqr/gen_lqr_fit.py 生成，不手改",
        "// 输出：左轮、右轮（前进为正）、左摆、右摆（后摆为正），N·m",
        "// 状态：位移、速度、航向角、航向角速度、左腿倾角及角速度、右腿倾角及角速度、俯仰及角速度",
        "// 项：1、左腿长、右腿长、左腿长²、左×右腿长、右腿长²，腿长单位 m",
        f"// 机体：{body}（SI 单位）",
        f"// Q = diag({', '.join(f'{q:g}' for q in np.diag(Q))})",
        f"// R = diag({', '.join(f'{r:g}' for r in np.diag(R))})",
        f"// 腿长 {LEG[0, 0]:g}–{LEG[-1, 0]:g} m，拟合最大相对误差 {err:.2%}",
        "inline constexpr float LQR_FIT[4][10][6] = {",
    ]
    for i in range(4):
        lines += ["    {", f"        // {OUTPUTS[i]}"]
        lines += [f"        {{{', '.join(num(v) for v in P[i, j])}}}," for j in range(10)]
        lines.append("    },")
    lines += ["};", ""]
    OUT.write_text("\n".join(lines), encoding="utf-8", newline="\n")

    np.set_printoptions(precision=4, suppress=True, linewidth=160)
    print(f"写入 {OUT}，拟合最大相对误差 {err:.2%}")
    print("L = 0.18 时 K（可与 Leg_Lqr__4.m Step 2.5 输出对照）：")
    print(fit_eval(P, 0.18, 0.18))


if __name__ == "__main__":
    main()
