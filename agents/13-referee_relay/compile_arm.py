"""用已有构建目录的 compile_commands.json 参数，以 ARM GCC 单独编译本任务涉及的源文件（含引用了改动头文件的 app 文件）。"""

from pathlib import Path
import json
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "agents/tmp/13-referee_relay/arm"

# 构建目录 → 待编译文件；模板取同目标（lib 或 app）中已有条目的参数
JOBS = {
    "leg-leg_down": ["lib/referee/referee_relay.cpp", "lib/referee/referee.cpp", "lib/referee/referee_sender.cpp",
                     "lib/referee/robot_ui.cpp", "lib/rc/vt13.cpp",
                     "app/leg/leg_down/app.cpp", "app/leg/leg_down/chassis.cpp"],
    "leg-leg_up": ["app/leg/leg_up/app.cpp", "app/leg/leg_up/chassis.cpp", "app/leg/leg_up/gimbal.cpp", "app/leg/leg_up/shooter.cpp"],
    "steer_sentry": ["app/steer_sentry/app.cpp", "app/steer_sentry/chassis.cpp", "app/steer_sentry/gimbal.cpp", "app/steer_sentry/shooter.cpp",
               "app/examples/referee/app.cpp"],
}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    failed = 0
    for build, files in JOBS.items():
        commands = json.loads((ROOT / "build" / build / "compile_commands.json").read_text(encoding="utf-8"))
        for file in files:
            group = "/lib/" if file.startswith("lib/") else "/app/"
            entry = next(c for c in commands if group in Path(c["file"]).as_posix() and c["file"].endswith(".cpp"))
            args = shlex.split(entry["command"], posix=False) if "command" in entry else entry["arguments"]
            args = [a.strip('"') for a in args]
            index = args.index("-o")
            obj = OUT / (build + "_" + file.replace("/", "_") + ".o")
            args[index + 1] = str(obj)
            args = [a for a in args if Path(a).as_posix() != Path(entry["file"]).as_posix()]
            args[args.index("-c") + 1:args.index("-c") + 1] = [str(ROOT / file)]
            result = subprocess.run(args, cwd=entry["directory"], capture_output=True, text=True)
            status = "OK" if result.returncode == 0 and not result.stderr.strip() else "FAIL" if result.returncode else "WARN"
            print(f"{status} {build}: {file}")
            if result.stderr.strip():
                print(result.stderr)
            failed += result.returncode != 0
    raise SystemExit(failed)


if __name__ == "__main__":
    main()
