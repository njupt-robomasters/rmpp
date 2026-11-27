"""离线分析 Ember yaw 正弦观察数据；仅读取 CSV，输出 agents/tmp/sine_observe。"""

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "agents/tmp/sine_observe"


def fit(values, time, frequency):
    phase = 2 * np.pi * frequency * time
    matrix = np.column_stack([np.sin(phase), np.cos(phase), np.ones(len(time)), time - time.mean()])
    coefficients = np.linalg.lstsq(matrix, values, rcond=None)[0]
    residual = np.sqrt(np.mean((values - matrix @ coefficients) ** 2))
    return coefficients, float(residual)


def phase_lag(reference, measurement):
    return float((np.arctan2(reference[1], reference[0]) -
                  np.arctan2(measurement[1], measurement[0]) + np.pi) % (2 * np.pi) - np.pi)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=OUT / "initial_chart.csv")
    args = parser.parse_args()
    with args.input.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    timestamps = np.array([datetime.fromisoformat(row["time"].replace("Z", "+00:00")).timestamp() for row in rows])
    time = timestamps - timestamps[0]
    ref = np.array([float(row["gimbal.yaw.ref.radians [f32]"]) for row in rows])
    measure = np.array([float(row["gimbal.yaw.measure.radians [f32]"]) for row in rows])
    current = np.array([float(row["gimbal.motor.yaw2.current_ref [f32]"]) for row in rows])
    loss = lambda frequency: fit(ref, time, frequency)[1]
    frequency = min(np.arange(2.7, 3.3001, 0.001), key=loss)
    frequency = float(min(np.arange(frequency - 0.001, frequency + 0.00101, 0.00001), key=loss))
    cr, residual_ref = fit(ref, time, frequency)
    cm, residual_measure = fit(measure, time, frequency)
    lag = phase_lag(cr, cm)
    ref_amplitude = float(np.hypot(*cr[:2]))
    measure_amplitude = float(np.hypot(*cm[:2]))
    changed = np.r_[True, np.abs(np.diff(ref)) > 1e-7]
    hold_times = np.diff(time[changed])
    omega = 2 * np.pi * frequency
    # 由当时角度环 kp=10、零角速度前馈推算速度指令基波；测量速度由角度基波求导，属于推算
    speed_ref = 10 * (cr[:2] - cm[:2])
    speed_measure = omega * np.array([-cm[1], cm[0]])
    result = {
        "samples": len(rows), "duration_s": float(time[-1]), "frequency_hz": frequency,
        "interval_median_ms": float(np.median(np.diff(time)) * 1000),
        "interval_p95_ms": float(np.percentile(np.diff(time), 95) * 1000),
        "reference_amplitude_deg": float(np.degrees(ref_amplitude)),
        "measured_amplitude_deg": float(np.degrees(measure_amplitude)),
        "amplitude_ratio": measure_amplitude / ref_amplitude,
        "phase_lag_deg": float(np.degrees(lag)), "equivalent_delay_ms": lag / omega * 1000,
        "error_rms_deg": float(np.degrees(np.sqrt(np.mean((ref - measure) ** 2)))),
        "error_peak_deg": float(np.degrees(np.max(np.abs(ref - measure)))),
        "reference_residual_deg": float(np.degrees(residual_ref)),
        "measured_residual_deg": float(np.degrees(residual_measure)),
        "current_ref_peak_A": float(np.max(np.abs(current))),
        "current_ref_rms_A": float(np.sqrt(np.mean(current ** 2))),
        "current_ref_saturated_fraction": float(np.mean(np.abs(current) >= 8.49)),
        "observed_target_changes_hz": float((changed.sum() - 1) / time[-1]),
        "observed_target_hold_median_ms": float(np.median(hold_times) * 1000),
        "derived_speed_ref_amplitude_rad_s": float(np.hypot(*speed_ref)),
        "derived_speed_measure_amplitude_rad_s": float(np.hypot(*speed_measure)),
        "derived_speed_ratio": float(np.hypot(*speed_measure) / np.hypot(*speed_ref)),
        "derived_speed_lag_deg": float(np.degrees(phase_lag(speed_ref, speed_measure))),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "analysis.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
