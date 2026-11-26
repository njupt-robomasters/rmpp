"""复用已有回归，对比成员排序前后的电机、超电与蜂鸣器行为。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/global_refactor/class_order/regression"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default="ba752f6")
    parser.add_argument("--cxx", default="g++")
    args = parser.parse_args()
    baseline = OUT / "baseline"
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
        "motor": (TASK / "motor_config_test.cpp", ["motor/motor.cpp"]),
        "cap": (ROOT / "agents/07-power/super_cap/sim.cpp", ["power/super_cap.cpp"]),
        "power": (ROOT / "agents/07-power/super_cap/power_sim.cpp", ["power/super_cap.cpp"]),
        "melody": (TASK / "layout_melody_regression.cpp", ["melody/melody.cpp"]),
    }

    def run(item):
        name, (source, units) = item
        outputs = []
        for version, lib in [("baseline", baseline / "lib"), ("current", ROOT / "lib")]:
            exe = OUT / (name + "_" + version + ".exe")
            command = [args.cxx, "-std=c++23", "-O2", "-Wall", "-Wextra", "-Werror=reorder",
                       "-I" + str(lib), "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"),
                       str(source), *[str(lib / unit) for unit in units], "-o", str(exe)]
            result = subprocess.run(command, cwd=ROOT, capture_output=True)
            (OUT / (name + "_" + version + ".log")).write_bytes(result.stdout + result.stderr)
            if result.returncode:
                raise RuntimeError(f"{name}/{version}: compile failed; see log")
            output = subprocess.check_output([str(exe)], cwd=ROOT)
            (OUT / (name + "_" + version + ".txt")).write_bytes(output)
            outputs.append(output)
        if outputs[0] != outputs[1]:
            raise RuntimeError(name + ": baseline/current outputs differ")
        return name + ": baseline/current outputs match"

    with ThreadPoolExecutor(max_workers=2) as pool:
        for result in pool.map(run, cases.items()):
            print(result, flush=True)


if __name__ == "__main__":
    main()
