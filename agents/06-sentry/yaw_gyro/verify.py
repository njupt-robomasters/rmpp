"""编译真实 GimbalFold 验证小 yaw 的 IMU 角速度内环，并对本次改动的源文件执行 ARM 对象编译。"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/yaw_gyro"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-only", action="store_true")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if not args.arm_only:
        env = {
            **os.environ,
            "ZIG_GLOBAL_CACHE_DIR": str(OUT / "zig_cache"),
            "ZIG_LOCAL_CACHE_DIR": str(OUT / "zig_local_cache"),
        }
        exe = OUT / "gimbal_fold_gyro_test.exe"
        compile_cmd = [
            str(ROOT / "agents/.tools/ziglang/zig.exe"),
            "c++",
            "-std=c++23",
            "-O1",
            "-Wall",
            "-Wextra",
            "-Wno-unknown-pragmas",
            "-Wno-deprecated-declarations",
            "-DEIGEN_DONT_VECTORIZE",
            "-DEIGEN_MAX_ALIGN_BYTES=0",
            "-Ilib",
            "-Ilib/third_party/eigen-3.4.0",
            str(TASK / "gimbal_fold_gyro_test.cpp"),
            "lib/robot/gimbal_fold.cpp",
            "lib/motor/motor.cpp",
            "lib/pid/pid.cpp",
            "-o",
            str(exe),
        ]
        res = subprocess.run(compile_cmd, cwd=ROOT, env=env, text=True, capture_output=True)
        (OUT / "host_compile.log").write_text(res.stdout + res.stderr, encoding="utf-8")
        if res.returncode:
            print(res.stdout + res.stderr, flush=True)
        res.check_returncode()

        run_res = subprocess.run([str(exe)], cwd=ROOT, text=True, capture_output=True)
        (OUT / "host_test.log").write_text(run_res.stdout + run_res.stderr, encoding="utf-8")
        print(run_res.stdout + run_res.stderr, end="", flush=True)
        run_res.check_returncode()

    db = ROOT / "build/jelly_sentry/compile_commands.json"
    if db.exists():
        rows = json.loads(db.read_text(encoding="utf-8"))
        targets = [
            row
            for row in rows
            if row["file"].replace("\\", "/").endswith(
                ("/lib/robot/gimbal_fold.cpp", "/app/jelly_sentry/gimbal.cpp")
            )
        ]
        summary = {}
        for row in targets:
            rel = Path(row["file"]).relative_to(ROOT).as_posix()
            obj = OUT / (rel.replace("/", "_") + ".obj")
            cmd = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{obj.as_posix()}" -c "{row["file"]}"', row["command"])
            arm_res = subprocess.run(cmd, cwd=row["directory"], text=True, capture_output=True)
            log = arm_res.stdout + arm_res.stderr
            (OUT / (obj.stem + ".log")).write_text(log, encoding="utf-8")
            if log:
                print(log, flush=True)
            arm_res.check_returncode()
            summary[rel] = {"exit_code": arm_res.returncode, "warnings": log.count("warning:")}
            print("ARM compiled:", rel, flush=True)
        (OUT / "arm_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    for cache_dir in ("zig_cache", "zig_local_cache"):
        target = (OUT / cache_dir).resolve()
        assert target.is_relative_to(OUT.resolve())
        shutil.rmtree(target, ignore_errors=True)


if __name__ == "__main__":
    main()
