"""复用共享工具验证自瞄协议、导航回归及相关 ARM 对象；验证后移除 Zig 编译缓存。"""

import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/minipc_aim"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "ZIG_GLOBAL_CACHE_DIR": str(OUT / "zig_cache"),
           "ZIG_LOCAL_CACHE_DIR": str(OUT / "zig_local_cache")}
    command = [str(ROOT / "agents/.tools/ziglang/zig.exe"), "c++", "-std=c++23", "-O1", "-Wall", "-Wextra",
               "-Wno-unknown-pragmas", "-Wno-unused-function", "-DEIGEN_DONT_VECTORIZE", "-DEIGEN_MAX_ALIGN_BYTES=0",
               "-I" + str(ROOT / "lib"), "-I" + str(ROOT / "lib/third_party/eigen-3.4.0")]
    for name, source, extra in [
        ("aim_test", TASK / "aim_test.cpp", ["lib/minipc/aim.cpp", "lib/minipc/nav.cpp"]),
        ("nav_test", TASK.parent / "nav/nav_test.cpp",
         ["lib/minipc/nav.cpp", "lib/robot/gimbal_fold.cpp", "lib/motor/motor.cpp", "lib/pid/pid.cpp"]),
    ]:
        exe = OUT / (name + ".exe")
        result = subprocess.run([*command, "-I" + str(ROOT / "app/jelly_sentry"), str(source),
                                 *[str(ROOT / unit) for unit in extra], "-o", str(exe)],
                                cwd=ROOT, env=env, text=True, capture_output=True)
        (OUT / (name + "_compile.log")).write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode:
            print(result.stdout + result.stderr, flush=True)
        result.check_returncode()
        result = subprocess.run([str(exe)], cwd=ROOT, text=True, capture_output=True)
        (OUT / (name + ".log")).write_text(result.stdout + result.stderr, encoding="utf-8")
        print(result.stdout + result.stderr, flush=True)
        result.check_returncode()

    rows = json.loads((ROOT / "build/jelly_sentry/compile_commands.json").read_text(encoding="utf-8"))
    units = [row for row in rows if "/app/jelly_sentry/" in row["file"].replace("\\", "/")
             or row["file"].endswith(("/lib/minipc/aim.cpp", "/lib/minipc/nav.cpp"))]
    # C 板构建目录已由用户清理，复用现有 ARM C++ 命令做接口编译，输出标明采用 mc02 编译环境
    template = next(row for row in rows if row["file"].endswith("/app/jelly_sentry/app.cpp"))
    for name in ["app", "chassis", "gimbal", "misc", "shooter"]:
        units.append({**template, "file": (ROOT / f"app/sentry/{name}.cpp").as_posix()})
    summary = {}
    for row in units:
        relative = Path(row["file"]).relative_to(ROOT).as_posix()
        output = OUT / (relative.replace("/", "_") + ".obj")
        cmd = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{output.as_posix()}" -c "{row["file"]}"', row["command"])
        result = subprocess.run(cmd, cwd=row["directory"], text=True, capture_output=True)
        log = result.stdout + result.stderr
        (OUT / (output.stem + ".log")).write_text(log, encoding="utf-8")
        if log:
            print(log, flush=True)
        result.check_returncode()
        summary[relative] = {"exit_code": result.returncode, "warnings": log.count("warning:")}
        print("ARM compiled:", relative, flush=True)
    (OUT / "arm_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    for name in ["zig_cache", "zig_local_cache"]:
        target = (OUT / name).resolve()
        assert target.is_relative_to(OUT.resolve())
        shutil.rmtree(target, ignore_errors=True)


if __name__ == "__main__":
    main()
