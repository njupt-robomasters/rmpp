"""比较类布局重构前后的电机输出和超电仿真，产物放在任务 out/。"""

import argparse
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = TASK / "out/layout_regression"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--baseline", default="795e263")
    args = parser.parse_args()
    baseline = OUT / "baseline"
    baseline.mkdir(parents=True, exist_ok=True)
    paths = subprocess.check_output(
        ["git", "ls-tree", "-r", "--name-only", args.baseline, "lib"], cwd=ROOT, text=True
    ).splitlines()
    for name in paths:
        path = Path(name)
        if "third_party" in path.parts or path.suffix not in (".hpp", ".cpp", ".tpp"):
            continue
        dest = baseline / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(subprocess.check_output(["git", "show", args.baseline + ":" + name], cwd=ROOT))

    cases = {
        "motor": (TASK / "layout_motor_regression.cpp", [
            "motor/motor.cpp", "motor/m3508.cpp", "motor/m2006.cpp", "motor/gm6020.cpp",
            "motor/x630.cpp", "motor/rm6623.cpp", "motor/rmdl7025.cpp", "motor/vesc.cpp",
            "robot/shooter/shooter_17mm.cpp", "pid/pid.cpp",
        ]),
        "cap": (ROOT / "agents/tasks/06-super_cap_capacitance/sim.cpp", ["power/super_cap.cpp"]),
        "power": (ROOT / "agents/tasks/06-super_cap_capacitance/power_sim.cpp", ["power/super_cap.cpp"]),
        "melody": (TASK / "layout_melody_regression.cpp", ["melody/melody.cpp"]),
    }
    for name, (source, units) in cases.items():
        outputs = []
        for version, lib in [("baseline", baseline / "lib"), ("current", ROOT / "lib")]:
            flags = []
            if version == "baseline":
                flags = ["-DLAYOUT_BASELINE"]
                if name in ("cap", "power"):
                    text = source.read_text(encoding="utf-8").replace("last_cycles", "last_cycles_")
                    text = re.sub(r"super_cap\.(voltage|current|power|energy)_(\w+)", r"super_cap.\1.\2", text)
                    source_path = OUT / (name + "_baseline.cpp")
                    source_path.write_text(text, encoding="utf-8")
                else:
                    source_path = source
            else:
                source_path = source
            exe = OUT / (name + "_" + version + ".exe")
            command = [args.cxx, "-std=c++23", "-O2", "-Wall", *flags, "-I" + str(lib),
                       "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"), str(source_path),
                       *[str(lib / unit) for unit in units], "-o", str(exe)]
            subprocess.run(command, cwd=ROOT, check=True)
            output = subprocess.check_output([str(exe)], cwd=ROOT)
            (OUT / (name + "_" + version + ".txt")).write_bytes(output)
            outputs.append(output)
        if outputs[0] != outputs[1]:
            raise RuntimeError(name + " 重构前后输出不同，查看 out/layout_regression 中的对照文件")
        print(name + ": baseline/current outputs match", flush=True)


if __name__ == "__main__":
    main()
