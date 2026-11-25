"""对照仿真 power_maps() 与固件口径：固件用拟合表 LQR_FIT、LQR_POSTURE_FIT 在控制拍内求 M、H⁻¹

用法：python check_maps.py（需 wheel_leg_lab 的 .venv，含 scipy）
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LAB = ROOT.parent / 'wheel_leg_lab'
sys.path.insert(0, str(LAB))
from lab import Settings, power_maps  # noqa: E402
from lqr_model import LEG, Q, R, fit_c, fit_eval, lqr, model  # noqa: E402

FIT = fit_c(ROOT / 'app/leg/leg_down/lqr_fit.hpp')
text = (ROOT / 'app/leg/leg_down/lqr_fit.hpp').read_text(encoding='utf-8')
block = text.split('LQR_POSTURE_FIT[2][6] = {')[1].split('};')[0]
POSTURE = np.array([[float(v.strip().rstrip('f')) for v in row.strip().strip('{},').split(',')] for row in block.strip().splitlines()])


def firmware_maps(ll, lr):
    gain = fit_eval(FIT, ll, lr)
    terms = np.array([1, ll, lr, ll * ll, ll * lr, lr * lr])
    pitch_rate = POSTURE @ terms
    wheel_modes = gain[:2, [9, 3]]
    leg_modes = np.column_stack((pitch_rate - gain[2:, 9], -gain[2:, 3]))
    leg_map = leg_modes @ np.linalg.inv(wheel_modes)
    cost = np.diag(np.diag(R)[:2]) + leg_map.T @ np.diag(np.diag(R)[2:]) @ leg_map
    return leg_map, np.linalg.inv(cost), np.linalg.det(wheel_modes)


def exact_pitch_rate(ll, lr):
    A, B, _ = model(ll, lr)
    return lqr(A[4:, 4:], B[4:, 2:], Q[4:, 4:], R[2:, 2:])[:, 5]


def main():
    for length in (.14, .18, .22, .30):
        settings = Settings(length=length)
        lab_map, lab_inverse = power_maps(settings, settings.gain_si())
        leg_map, inverse, det = firmware_maps(length, length)
        print(f'L={length:.2f} det(C)={det:+.3f}  M 差 {np.max(abs(leg_map - lab_map)):.4f} / {np.max(abs(lab_map)):.4f}'
              f'  H⁻¹ 差 {np.max(abs(inverse - lab_inverse)):.2e} / {np.max(abs(lab_inverse)):.2e}')
        print('  M =', np.array2string(leg_map, precision=4).replace('\n', ''), ' H⁻¹ =', np.array2string(inverse, precision=6).replace('\n', ''))

    worst = 0.
    for ll in LEG[:, 0]:
        for lr in LEG[:, 0]:
            terms = np.array([1, ll, lr, ll * ll, ll * lr, lr * lr])
            exact = exact_pitch_rate(ll, lr)
            worst = max(worst, np.max(abs(POSTURE @ terms - exact)))
    print(f'俯仰角速度增益拟合最大绝对误差 {worst:.3f} N·m/(rad/s)')


if __name__ == '__main__':
    main()
