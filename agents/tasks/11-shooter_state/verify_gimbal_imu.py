"""验证 GimbalClassic IMU 闭环；可选编译真实 ARM 对象，不链接固件。"""

import argparse
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/out/11-shooter_state/gimbal_imu"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--arm", action="store_true")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    executable = OUT / "gimbal_imu_test.exe"
    subprocess.run([
        args.cxx, "-std=c++23", "-O2", "-Wall", "-Wextra",
        "-I" + str(ROOT / "lib"), "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"),
        str(TASK / "gimbal_imu_test.cpp"),
        *[str(ROOT / "lib" / source) for source in [
            "robot/gimbal_classic.cpp", "motor/motor.cpp", "pid/pid.cpp",
        ]], "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
    if not args.arm:
        return
    count = 0
    for name in ["infantry", "leg-leg_up"]:
        rows = json.loads((ROOT / "build" / name / "compile_commands.json").read_text())
        selected = [row for row in rows if "/app/" in row["file"].replace("\\", "/")]
        if name == "leg-leg_up":
            selected += [row for row in rows if row["file"].endswith("/lib/robot/gimbal_classic.cpp")]
        for row in selected:
            source = Path(row["file"]).relative_to(ROOT).as_posix()
            output = OUT / (name + "_" + source.replace("/", "_") + ".obj")
            command = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{output.as_posix()}" -c "{row["file"]}"', row["command"])
            result = subprocess.run(command, cwd=row["directory"], text=True, capture_output=True)
            (OUT / (output.stem + ".log")).write_text(result.stdout + result.stderr, encoding="utf-8")
            if result.returncode or result.stderr:
                print(result.stdout + result.stderr, flush=True)
            result.check_returncode()
            count += 1
            print("ARM compiled:", source, flush=True)
    print(f"ARM: {count} objects compiled; logs in {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
