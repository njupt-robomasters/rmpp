"""验证原生电机输出和拨弹状态机；仅编译对象及主机测试，不链接固件。"""

import argparse
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/shooter"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--arm-build", type=Path)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    units = ["motor/motor.cpp", "motor/vesc.cpp", "motor/x630.cpp",
             "robot/shooter_17mm.cpp", "pid/pid.cpp"]
    exe = OUT / "output_state_test.exe"
    subprocess.run([args.cxx, "-std=c++23", "-O2", "-Wall", "-Wextra",
                    "-I" + str(ROOT / "lib"), "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"),
                    str(TASK / "output_state_test.cpp"),
                    *[str(ROOT / "lib" / unit) for unit in units], "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    if args.arm_build is None:
        return
    rows = json.loads((args.arm_build / "compile_commands.json").read_text())
    sources = ["lib/robot/gimbal_classic.cpp",
               "lib/robot/gimbal_dual_yaw.cpp", "lib/robot/shooter_17mm.cpp",
               "lib/robot/shooter_42mm.cpp"]
    sources += [p.relative_to(ROOT).as_posix() for p in (ROOT / "lib/motor").glob("*.cpp")]
    sources += ["app/leg/leg_up/app.cpp", "app/leg/leg_up/gimbal.cpp", "app/leg/leg_up/shooter.cpp",
                "app/infantry/gimbal.cpp", "app/steer_sentry/gimbal.cpp", "app/examples/gimbal/app.cpp"]
    template = next(r for r in rows if r["file"].endswith("/app/leg/leg_up/gimbal.cpp"))
    for source in sources:
        row = next((r for r in rows if r["file"].endswith("/" + source)), template)
        output = OUT / (source.replace("/", "_") + ".obj")
        command = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{output.as_posix()}" -c "{(ROOT / source).as_posix()}"', row["command"])
        subprocess.run(command, cwd=row["directory"], check=True)
        print("ARM compiled:", source, flush=True)


if __name__ == "__main__":
    main()
