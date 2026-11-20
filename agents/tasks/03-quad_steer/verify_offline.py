"""离线反馈回归及相关源码 ARM 对象编译，不链接固件。"""

import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = TASK / "out/offline"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    common = ["g++", "-std=c++23", "-O2", "-Wall", "-Wextra",
              "-I" + str(ROOT / "lib"), "-I" + str(ROOT / "lib/third_party/eigen-3.4.0")]
    for name, sources in {
        "motor": [ROOT / "agents/tasks/10-global_refactor/motor_config_test.cpp", ROOT / "lib/motor/motor.cpp"],
        "chassis": [TASK / "offline_test.cpp", *[ROOT / "lib" / p for p in
                    ["motor/motor.cpp", "pid/pid.cpp", "power/motor_power.cpp",
                     "robot/chassis/chassis_matrix.cpp", "robot/chassis/chassis_steer.cpp"]]],
    }.items():
        exe = OUT / (name + ".exe")
        subprocess.run([*common, *map(str, sources), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

    count = 0
    for preset, sources in {
        "leg-leg_up": ["lib/motor/motor.cpp", "lib/robot/chassis/chassis_matrix.cpp",
                   "lib/robot/chassis/chassis_steer.cpp", "app/leg/leg_up/chassis.cpp"],
        "sentry": ["app/sentry/gimbal.cpp", "app/sentry/app.cpp"],
    }.items():
        rows = json.loads((ROOT / "build" / preset / "compile_commands.json").read_text(encoding="utf-8"))
        for source in sources:
            row = next(r for r in rows if Path(r["file"]).resolve() == ROOT / source)
            output = OUT / (source.replace("/", "_") + ".obj")
            command = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{output.as_posix()}" -c "{(ROOT / source).as_posix()}"', row["command"])
            result = subprocess.run(command, cwd=row["directory"], capture_output=True, text=True)
            output.with_suffix(".log").write_text(result.stdout + result.stderr, encoding="utf-8")
            if result.returncode or result.stderr:
                print(result.stdout + result.stderr, flush=True)
            result.check_returncode()
            count += 1
    print(f"PASS: {count} ARM objects", flush=True)


if __name__ == "__main__":
    main()
