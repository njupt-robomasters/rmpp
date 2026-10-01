"""编译实际 IMU C++，用 NumPy 双精度参考与模拟外设验证；无需 STM32 硬件。"""

import argparse
import ctypes as ct
import json
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build/imu-tests"
F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
I16 = np.ctypeslib.ndpointer(dtype=np.int16, flags="C_CONTIGUOUS")


def array(values):
    return np.ascontiguousarray(values, dtype=np.float32)


def product(q, r):
    q, r = np.asarray(q), np.asarray(r)
    return np.r_[q[0] * r[0] - q[1:] @ r[1:],
                 q[0] * r[1:] + r[0] * q[1:] + np.cross(q[1:], r[1:])]


def rotation(q):
    w, x, y, z = q
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1 - 2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1 - 2*(x*x+y*y)]])


def error(q, reference):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    reference = reference / np.linalg.norm(reference)
    return np.degrees(2 * np.arccos(np.clip(abs(q @ reference), 0, 1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    args = parser.parse_args()
    BUILD.mkdir(parents=True, exist_ok=True)
    dll = BUILD / "imu_probe.dll"
    command = [args.cxx, "-std=c++23", "-O2", "-Wall", "-Wextra", "-Werror", "-shared",
               "-static-libgcc", "-static-libstdc++", "-Ilib", "tests/imu_probe.cpp",
               "lib/imu/imu.cpp", "lib/imu/bmi088.cpp", "lib/motor/pid.cpp", "-o", str(dll)]
    subprocess.run(command, cwd=ROOT, check=True)
    lib = ct.CDLL(str(dll))
    signatures = {
        "ekf_create": ([ct.c_void_p], ct.c_void_p), "ekf_destroy": ([ct.c_void_p], None),
        "ekf_step": ([ct.c_void_p, F32, F32, ct.c_float, F32], None),
        "ekf_reset_bias": ([ct.c_void_p], None),
        "convert_euler": ([F32, F32, F32], None), "rotate_vector": ([F32, F32, ct.c_bool], None),
        "integrate": ([F32, F32, ct.c_float], None),
        "matrix_product": ([F32, F32, F32, ct.c_bool], None),
        "matrix_solve": ([F32, F32, F32], None), "covariance_check": ([F32], ct.c_bool),
        "imu_create": ([ct.c_int, F32, F32, ct.c_float], None),
        "imu_sample": ([ct.c_int, I16, I16, ct.c_int16, ct.c_bool], None),
        "advance": ([ct.c_uint32], None), "imu_step": ([ct.c_int, F32], None),
        "imu_calibrate": ([ct.c_int], None),
    }
    for name, (arguments, result) in signatures.items():
        getattr(lib, name).argtypes = arguments
        getattr(lib, name).restype = result
    rng = np.random.default_rng(20261001)
    results = {}

    # 固定矩阵乘法、转置乘法和 Cholesky 回代，与双精度比较
    for _ in range(300):
        a, b = array(rng.normal(size=(6, 6))), array(rng.normal(size=(6, 6)))
        out = np.zeros((6, 6), dtype=np.float32)
        for transpose in [False, True]:
            lib.matrix_product(a, b, out, transpose)
            reference = a.astype(float) @ (b.T if transpose else b).astype(float)
            np.testing.assert_allclose(out, reference, atol=2e-6, rtol=2e-6)
        basis = rng.normal(size=(3, 3))
        covariance = basis @ basis.T + np.eye(3) * 0.1
        lower = array(np.linalg.cholesky(covariance))
        rhs, solution = array(rng.normal(size=3)), np.zeros(3, dtype=np.float32)
        lib.matrix_solve(lower, rhs, solution)
        np.testing.assert_allclose(solution, np.linalg.solve(covariance, rhs), atol=2e-5, rtol=2e-5)
    for scale in [1e-15, 1, 1e15]:
        lower = array(np.diag([1e-3, 1, 1e3]) * scale)
        rhs, solution = array([1, 2, 3]), np.zeros(3, dtype=np.float32)
        lib.matrix_solve(lower, rhs, solution)
        np.testing.assert_allclose(solution, rhs.astype(float) / np.diag(lower).astype(float)**2, rtol=1e-6)
    for invalid in [-1, np.nan, np.inf]:
        p = np.eye(6, dtype=np.float32)
        p[0, 0] = invalid
        assert not lib.covariance_check(p)
    results["matrix_cases"] = 903

    max_angle_error = 0
    for _ in range(1000):
        angles = array(rng.uniform([-np.pi, -1.5, -np.pi], [np.pi, 1.5, np.pi]))
        q, back = np.zeros(4, dtype=np.float32), np.zeros(3, dtype=np.float32)
        lib.convert_euler(angles, q, back)
        np.testing.assert_allclose(back, angles, atol=5e-6)
        yaw, pitch, roll = angles.astype(float)
        reference = product(product([np.cos(yaw/2), 0, 0, np.sin(yaw/2)],
                                    [np.cos(pitch/2), 0, np.sin(pitch/2), 0]),
                            [np.cos(roll/2), np.sin(roll/2), 0, 0])
        max_angle_error = max(max_angle_error, error(q, reference))
        vector = array(rng.normal(size=3))
        original = vector.copy()
        lib.rotate_vector(vector, q, False)
        np.testing.assert_allclose(vector, rotation(reference) @ original, atol=3e-6)
        lib.rotate_vector(vector, q, True)
        np.testing.assert_allclose(vector, original, atol=5e-6)
    results["conversion_max_error_deg"] = max_angle_error

    zero, gravity = array([0, 0, 0]), array([0, 0, 9.81])
    out = np.zeros(6, dtype=np.float32)
    max_trajectory_error = 0
    max_norm_error = 0
    # 倾斜、倒置和接近欧拉奇点启动
    for roll, pitch in [(0, 0), (0.8, -0.7), (np.pi, 0), (0, 1.5707), (-2.8, 0.3)]:
        expected = product([np.cos(pitch/2), 0, np.sin(pitch/2), 0],
                           [np.cos(roll/2), np.sin(roll/2), 0, 0])
        instance = lib.ekf_create(None)
        accel = array(rotation(expected).T @ gravity)
        lib.ekf_step(instance, zero, accel, 0.001, out)
        assert error(out[:4], expected) < 0.03
        for _ in range(1000):
            lib.ekf_step(instance, zero, accel, 0.001, out)
        assert error(out[:4], expected) < 0.03
        lib.ekf_destroy(instance)
    # 恒角速度解析轨迹，覆盖各轴、组合轴及不同 dt
    for rate in [[1, 0, 0], [0, -1.3, 0], [0, 0, 2], [0.7, -0.4, 0.8]]:
        for dt in [0.0005, 0.001, 0.004, 0.01]:
            instance = lib.ekf_create(None)
            lib.ekf_step(instance, zero, gravity, dt, out)
            omega = np.array(rate, dtype=float)
            norm = np.linalg.norm(omega)
            for step in range(1, int(5/dt)+1):
                half_angle = norm * step * dt / 2
                expected = np.r_[np.cos(half_angle), omega / norm * np.sin(half_angle)]
                accel = array(rotation(expected).T @ gravity)
                lib.ekf_step(instance, array(omega), accel, dt, out)
                max_trajectory_error = max(max_trajectory_error, error(out[:4], expected))
                max_norm_error = max(max_norm_error, abs(np.linalg.norm(out[:4].astype(float))-1))
                assert np.isfinite(out).all()
            lib.ekf_destroy(instance)
    assert max_trajectory_error < 0.1, max_trajectory_error
    assert max_norm_error < 1e-5, max_norm_error
    results["trajectory_max_error_deg"] = max_trajectory_error
    results["quaternion_max_norm_error"] = max_norm_error

    # 抖动周期与含噪组合转动；交错更新另一个实例不能污染当前滤波器
    instance, isolated = lib.ekf_create(None), lib.ekf_create(None)
    other = np.zeros(6, dtype=np.float32)
    lib.ekf_step(instance, zero, gravity, 0.001, out)
    lib.ekf_step(isolated, zero, gravity, 0.001, other)
    reference = np.array([1., 0, 0, 0])
    noisy_errors = []
    for i in range(10000):
        dt = rng.uniform(0.0005, 0.003)
        omega = np.array([0.3*np.cos(i*0.002), 0.2*np.sin(i*0.002), 0.4])
        norm = np.linalg.norm(omega)
        reference = product(reference, np.r_[np.cos(norm*dt/2), omega/norm*np.sin(norm*dt/2)])
        reference /= np.linalg.norm(reference)
        gyro = array(omega + rng.normal(0, 0.001, 3) + [0.004, -0.003, 0.002])
        accel = array(rotation(reference).T @ gravity + rng.normal(0, 0.03, 3))
        lib.ekf_step(instance, gyro, accel, dt, out)
        lib.ekf_step(isolated, zero, gravity, dt, other)
        noisy_errors.append(error(out[:4], reference))
        assert np.isfinite(out).all()
        assert error(other[:4], np.array([1, 0, 0, 0])) < 0.001
    assert max(noisy_errors) < 3
    results["jitter_noise_rms_deg"] = float(np.sqrt(np.mean(np.square(noisy_errors))))
    lib.ekf_destroy(instance)
    lib.ekf_destroy(isolated)

    # 极小观测方差逼近奇异协方差；失败时必须保持有限姿态
    config = array([0, 0, 1e-20, 1, 0.02, 1e-8])
    instance = lib.ekf_create(config.ctypes.data)
    for i in range(3000):
        lib.ekf_step(instance, zero, gravity if i % 3 else zero, 0.001, out)
        assert np.isfinite(out).all()
        assert abs(np.linalg.norm(out[:4].astype(float))-1) < 1e-5
    lib.ekf_destroy(instance)

    # 公开积分接口同样使用标准轴定义，零 dt 和非有限输入保持原值
    integrated = array([1, 0, 0, 0])
    lib.integrate(integrated, array([1, 0, 0]), 0.8)
    expected = np.array([np.cos(0.4), np.sin(0.4), 0, 0])
    assert error(integrated, expected) < 0.001
    before = integrated.copy()
    for dt in [0, -1, np.nan]:
        lib.integrate(integrated, array([1, 2, 3]), dt)
        np.testing.assert_array_equal(integrated, before)

    instance = lib.ekf_create(None)
    lib.ekf_step(instance, zero, gravity, 0.001, out)
    initial = out.copy()
    for dt in [0, -1, np.nan, np.inf]:
        lib.ekf_step(instance, array([1, 2, 3]), gravity, dt, out)
        np.testing.assert_array_equal(out, initial)
    for invalid in [np.nan, np.inf]:
        lib.ekf_step(instance, array([invalid, 0, 0]), gravity, 0.001, out)
        np.testing.assert_array_equal(out, initial)
    for _ in range(1000):
        lib.ekf_step(instance, array([0, 0, 1]), zero, 0.001, out)
    assert error(out[:4], np.array([np.cos(0.5), 0, 0, np.sin(0.5)])) < 0.03
    lib.ekf_destroy(instance)

    # 已收敛姿态遇到加速度冲击；持续静稳残差能恢复融合
    instance = lib.ekf_create(None)
    lib.ekf_step(instance, zero, gravity, 0.001, out)
    for _ in range(2000):
        lib.ekf_step(instance, zero, gravity, 0.001, out)
    for _ in range(100):
        lib.ekf_step(instance, zero, array([30, 0, 9.81]), 0.001, out)
    assert error(out[:4], np.array([1, 0, 0, 0])) < 0.01
    tilt = np.array([np.cos(0.4), np.sin(0.4), 0, 0])
    for _ in range(30000):
        lib.ekf_step(instance, zero, array(rotation(tilt).T @ gravity), 0.001, out)
    assert error(out[:4], tilt) < 1.0, (error(out[:4], tilt), out)
    results["static_recovery_error_deg"] = error(out[:4], tilt)
    lib.ekf_destroy(instance)

    # 零偏估计，重置时保留姿态；Z 零偏不在模型内
    instance = lib.ekf_create(None)
    lib.ekf_step(instance, zero, gravity, 0.001, out)
    for _ in range(60000):
        lib.ekf_step(instance, array([0.006, -0.004, 0]), gravity, 0.001, out)
    np.testing.assert_allclose(out[4:], [0.006, -0.004], atol=0.0002)
    results["estimated_bias_rad_s"] = out[4:].tolist()
    before = out[:4].copy()
    lib.ekf_reset_bias(instance)
    lib.ekf_step(instance, zero, gravity, 0, out)
    np.testing.assert_array_equal(out[:4], before)
    np.testing.assert_array_equal(out[4:], [0, 0])
    lib.ekf_destroy(instance)

    def sample(index, gyro=(0, 0, 0), accel=(0, 0, 5467), temp=176, fail=False):
        lib.imu_sample(index, np.array(gyro, dtype=np.int16), np.array(accel, dtype=np.int16), temp, fail)

    def step(index, microseconds=1000):
        lib.advance(microseconds)
        output = np.zeros(22, dtype=np.float32)
        lib.imu_step(index, output)
        return output

    # 构造不通信，SPI 解码、负温度、加热限幅和实例隔离
    lib.imu_create(0, zero, zero, 9.81)
    lib.imu_create(1, array([-np.pi/2, 0, 0]), zero, 9.81)
    sample(0, (1000, -2000, 3000), (100, -200, 5467), -240)
    a = step(0)
    np.testing.assert_allclose(a[:3], np.array([1000, -2000, 3000])*0.001065264436, rtol=1e-6)
    assert a[14] == -7 and a[15] == 1
    sample(1, (1000, 0, 0))
    b = step(1)
    np.testing.assert_allclose(b[:3], [0, -1.065264436, 0], atol=2e-6)
    sample(0, temp=400)
    a = step(0)
    assert a[14] == 73 and a[15] == 0
    sample(0, fail=True)
    np.testing.assert_array_equal(step(0)[6:10], a[6:10])

    # 校准：运动重开窗口、超时保留旧值、有效样本才计数、成功原子提交
    original = array([0.001, 0.002, 0.003])
    lib.imu_create(2, zero, original, 9.7)
    sample(2, (2, -3, 1))
    step(2)
    lib.imu_calibrate(2)
    for _ in range(100):
        a = step(2)
    assert a[17] == 100
    sample(2, (500, 0, 0))
    assert step(2)[17] == 0
    sample(2, fail=True)
    a = step(2, 10_000_001)
    assert a[16] == 3
    np.testing.assert_array_equal(a[18:21], original)
    assert abs(a[21] - 9.7) < 1e-6
    sample(2, (2, -3, 1))
    lib.imu_calibrate(2)
    for _ in range(5999):
        a = step(2)
    assert a[16] == 1 and a[17] == 5999
    np.testing.assert_array_equal(a[18:21], original)
    b = step(2)
    assert b[16] == 2 and b[17] == 6000
    np.testing.assert_allclose(b[18:21], np.array([2, -3, 1])*0.001065264436, rtol=1e-6)
    assert error(a[6:10], b[6:10]) < 0.1
    assert abs(a[13] - b[13]) < 0.001

    # yaw 多次跨越 ±π；重新初始化的等待不能计入积分
    lib.imu_create(3, zero, zero, 9.81)
    sample(3, (0, 0, 1000))
    step(3)
    for _ in range(12000):
        a = step(3)
    assert abs(a[13] - 12 * 1.065264436) < 0.002
    results["accumulated_yaw_rad"] = float(a[13])

    # 初始化失败后重试，长等待不得积成姿态跳变；hero 安装映射绕 Y 翻转
    lib.imu_create(4, array([0, np.pi, 0]), zero, 9.81)
    sample(4, fail=True)
    assert step(4)[15] == 0
    sample(4)
    step(4, 5_000_000)
    a = step(4)
    np.testing.assert_allclose(a[3:6], [0, 0, -5467*0.00179443359375], atol=2e-6)
    lib.imu_create(5, zero, zero, 9.81)
    sample(5, (0, 0, 1000), fail=True)
    step(5)
    sample(5, (0, 0, 1000))
    step(5, 5_000_000)
    a = step(5)
    assert abs(a[13]) < 0.002

    results["integration_cases"] = "SPI decoding, installation, heating, calibration, failure, isolation, yaw wrap"

    (BUILD / "results.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2))
    print("IMU checks passed")


if __name__ == "__main__":
    main()
