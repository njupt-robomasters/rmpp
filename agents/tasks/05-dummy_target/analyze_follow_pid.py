"""分析 capture_follow_pid.py 的 CSV，只使用已使能的跟随样本。"""

import csv
import json
import math
from pathlib import Path
import statistics
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


source = Path(sys.argv[1])
with source.open(encoding="utf-8") as stream:
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)]
active = [row for row in rows if row["enable"] == 1 and row["mode"] == 1]
# SWD 不暂停 MCU；剔除从不同计算周期读到的明显不一致字段。
clean = [row for row in active
         if abs(row["p"] - 10 * row["err"]) < 1e-4
         and abs(row["out"] - max(-6.283185482, min(
             6.283185482, row["p"] + row["i"] + row["d"]))) < 1e-4]
summary = {
    "source": source.name,
    "total_samples": len(rows),
    "active_samples": len(active),
    "consistent_samples": len(clean),
    "active_host_s": [active[0]["host_s"], active[-1]["host_s"]],
    "active_sampling_hz": (len(active) - 1) / (active[-1]["host_s"] - active[0]["host_s"]),
    "dt_ms_min_median_max": [1000 * reducer(row["dt"] for row in clean)
                              for reducer in (min, statistics.median, max)],
    "zero_d_samples": sum(row["d"] == 0 for row in clean),
    "saturated_samples": sum(abs(row["out"]) > 6.28 for row in clean),
    "extremes": [],
}
for row in (max(clean, key=lambda row: row["d"]), min(clean, key=lambda row: row["d"])):
    summary["extremes"].append({**row,
        "inferred_error_step_deg": row["d"] * row["dt"] / 2 * 180 / math.pi})
source.with_suffix(".analysis.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

plt.rcParams["font.family"] = "Microsoft YaHei"
plt.rcParams["axes.unicode_minus"] = False
fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, layout="constrained")
time = [row["host_s"] - active[0]["host_s"] for row in clean]
axes[0].plot(time, [row["err"] for row in clean], color="#c84444", linewidth=1,
             marker=".", markersize=2)
axes[0].set_ylabel("跟随误差 / rad")
axes[0].set_title("C 板只读实测：角度误差与微分输出（已使能的跟随样本）")
axes[1].scatter(time, [row["d"] for row in clean], color="#208052", s=9,
                label="实测 D 项，kd = 2")
axes[1].axhline(6.283185482, color="#888888", linestyle="--", linewidth=.8,
                label="PID 总输出限幅 ±6.283 rad/s")
axes[1].axhline(-6.283185482, color="#888888", linestyle="--", linewidth=.8)
axes[1].set_ylabel("D 项 / (rad/s)")
axes[1].set_xlabel("进入跟随后的时间 / s")
axes[1].legend(loc="upper right")
for axis in axes:
    axis.grid(alpha=.2)
fig.savefig(source.with_suffix(".png"), dpi=160)
print(json.dumps(summary, ensure_ascii=False, indent=2))
