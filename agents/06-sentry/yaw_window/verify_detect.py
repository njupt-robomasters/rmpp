"""编译真实 GimbalFold 并验证识别状态切换；产物与缓存保存在 out。"""
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = ROOT / "agents/tmp/yaw_window"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "ZIG_GLOBAL_CACHE_DIR": str(OUT / "detect_zig_global"),
           "ZIG_LOCAL_CACHE_DIR": str(OUT / "detect_zig_local")}
    exe = OUT / "detect_test.exe"
    command = [str(ROOT / "agents/.tools/ziglang/zig.exe"), "c++", "-std=c++23", "-O1",
               "-Wno-unknown-pragmas", "-Wno-deprecated-declarations",
               "-DEIGEN_DONT_VECTORIZE", "-DEIGEN_MAX_ALIGN_BYTES=0",
               "-Ilib", "-Ilib/third_party/eigen-3.4.0", "-Iapp/jelly_sentry",
               str(TASK / "detect_test.cpp"), "lib/robot/gimbal_fold.cpp",
               "lib/motor/motor.cpp", "lib/pid/pid.cpp", "-o", str(exe)]
    subprocess.run(command, cwd=ROOT, env=env, check=True)
    result = subprocess.run([str(exe)], cwd=ROOT, capture_output=True, text=True)
    (OUT / "detect_host.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    print(result.stdout + result.stderr, end="")
    result.check_returncode()


if __name__ == "__main__":
    main()
