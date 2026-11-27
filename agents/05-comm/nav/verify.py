"""真实编译 Nav、折叠云台、达妙驱动与底盘/云台流程，模拟时钟和通信；可选编译相关 ARM 对象。"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/nav"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--apps-only", action="store_true", help="只验证应用流程")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    shared_zig = ROOT / "agents/.tools/ziglang/zig.exe"
    compiler = shared_zig if shared_zig.exists() else OUT / "tools/ziglang/zig.exe"
    env = {**os.environ, "ZIG_GLOBAL_CACHE_DIR": str(OUT / "zig_cache"),
           "ZIG_LOCAL_CACHE_DIR": str(OUT / "zig_local_cache")}
    common = [str(compiler), "c++", "-std=c++23", "-O1", "-Wall", "-Wextra",
              "-Wno-unknown-pragmas", "-Wno-unused-function",
              "-DEIGEN_DONT_VECTORIZE", "-DEIGEN_MAX_ALIGN_BYTES=0",
              "-I" + str(TASK), "-I" + str(OUT), "-I" + str(ROOT / "lib"),
              "-I" + str(ROOT / "lib/third_party/eigen-3.4.0"),
              "-I" + str(ROOT / "app/jelly_sentry")]
    shutil.copyfile(ROOT / "app/jelly_sentry/chassis.cpp", OUT / "chassis.cpp")
    shutil.copyfile(ROOT / "app/jelly_sentry/gimbal.cpp", OUT / "gimbal.cpp")
    shutil.copyfile(ROOT / "app/jelly_sentry/minipc.cpp", OUT / "minipc.cpp")
    shutil.copyfile(ROOT / "app/jelly_sentry/sentry.cpp", OUT / "sentry.cpp")
    shutil.copyfile(ROOT / "app/jelly_sentry/sentry.hpp", OUT / "sentry.hpp")
    shutil.copyfile(ROOT / "app/jelly_sentry/shooter.cpp", OUT / "shooter.cpp")
    production_app = (ROOT / "app/jelly_sentry/app.hpp").read_text(encoding="utf-8")
    app_functions = production_app[production_app.index("inline bool is_sentry_test()") :]
    app_stub = (TASK / "chassis_app.hpp").read_text(encoding="utf-8")
    (OUT / "app.hpp").write_text(app_stub + "\n" + app_functions, encoding="utf-8")
    for header in ["chassis.hpp", "gimbal.hpp", "minipc.hpp", "shooter.hpp"]:
        (OUT / header).write_text('#include "app.hpp"\n', encoding="utf-8")
    cases = {
        "nav_test": [ROOT / "lib/robot/gimbal_fold.cpp",
                     ROOT / "lib/motor/motor.cpp", ROOT / "lib/pid/pid.cpp"],
        "chassis_test": [],
        "gimbal_test": [],
    }
    for name, units in cases.items():
        if args.apps_only and name == "nav_test":
            continue
        exe = OUT / (name + ".exe")
        command = [*common, str(TASK / (name + ".cpp")), str(ROOT / "lib/minipc/nav.cpp"),
                   *map(str, units), "-o", str(exe)]
        result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8", errors="replace")
        (OUT / (name + "_compile.log")).write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode:
            print(result.stdout + result.stderr, flush=True)
        result.check_returncode()
        subprocess.run([str(exe)], cwd=ROOT, check=True)
    if not args.arm:
        return

    summary = {}
    for preset in ["jelly_sentry", "steer_sentry"]:
        database = ROOT / "build" / preset / "compile_commands.json"
        if not database.exists():
            continue
        rows = json.loads(database.read_text(encoding="utf-8"))
        selected = [row for row in rows if "/app/" in row["file"].replace("\\", "/")
                    or row["file"].endswith("/lib/minipc/nav.cpp")]
        for row in selected:
            source = Path(row["file"]).relative_to(ROOT).as_posix()
            output = OUT / (preset + "_" + source.replace("/", "_") + ".obj")
            command = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{output.as_posix()}" -c "{row["file"]}"', row["command"])
            result = subprocess.run(command, cwd=row["directory"], capture_output=True, text=True, encoding="utf-8", errors="replace")
            log = result.stdout + result.stderr
            (OUT / (output.stem + ".log")).write_text(log, encoding="utf-8")
            summary[preset + ":" + source] = {"exit_code": result.returncode, "warnings": log.count("warning:")}
            if result.returncode or log:
                print(log, flush=True)
            result.check_returncode()
            print("ARM compiled:", preset, source, flush=True)
    (OUT / "arm_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
