"""用已有构建目录的 compile_commands.json 参数，单独编译本轮改动的 minipc 库与两个哨兵的 app。"""

from pathlib import Path
import json
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "agents/tmp/21-minipc_protocol/arm"

# 构建目录 → 待编译文件；参数取同目标（lib 或 app）中已有条目，C 板与 mc02 各编一遍库
JOBS = {
    "jelly_sentry": ["lib/minipc/aim.cpp", "lib/minipc/nav.cpp",
                     "app/jelly_sentry/app.cpp", "app/jelly_sentry/chassis.cpp"],
    "steer_sentry": ["lib/minipc/aim.cpp", "lib/minipc/nav.cpp",
                     "app/steer_sentry/app.cpp", "app/steer_sentry/chassis.cpp"],
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
            obj = OUT / (build + "_" + file.replace("/", "_") + ".o")
            args[args.index("-o") + 1] = str(obj)
            args = [a for a in args if Path(a).as_posix() != Path(entry["file"]).as_posix()]
            args[args.index("-c") + 1:args.index("-c") + 1] = [str(ROOT / file)]
            result = subprocess.run(args, cwd=entry["directory"], capture_output=True, text=True)
            status = "OK" if result.returncode == 0 and not result.stderr.strip() else "FAIL" if result.returncode else "WARN"
            print(f"{status} {build}: {file}")
            if result.stderr.strip():
                print(result.stderr)
            failed += result.returncode != 0 or bool(result.stderr.strip())
    raise SystemExit(failed)


if __name__ == "__main__":
    main()
