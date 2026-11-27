"""复用当前编译数据库，只编译本轮合并影响的真实 ARM 源文件，不链接或操作硬件。"""

import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "agents/tmp/upstream_sync"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    targets = {
        "jelly_sentry": [*sorted((ROOT / "app/jelly_sentry").glob("*.cpp")),
                         *[ROOT / "lib" / name for name in (
                             "bsp/imu.cpp", "robot/chassis_matrix.cpp", "robot/gimbal_fold.cpp",
                             "robot/gimbal_classic.cpp", "robot/shooter_17mm.cpp")]],
        "leg-leg_up": [ROOT / "app/leg/leg_up/gimbal.cpp", ROOT / "app/leg/leg_up/referee.cpp"],
    }
    summary = {}
    for preset, sources in targets.items():
        rows = json.loads((ROOT / "build" / preset / "compile_commands.json").read_text(encoding="utf-8"))
        for source in sources:
            source = source.resolve()
            row = next((r for r in rows if Path(r["file"]).resolve() == source), None)
            if row is None and source.name == "sentry.cpp":
                row = next(r for r in rows if r["file"].replace("\\", "/").endswith("/app/jelly_sentry/gimbal.cpp"))
            if row is None:
                raise RuntimeError(f"Missing compile command: {source}")
            label = preset + "_" + source.relative_to(ROOT).as_posix().replace("/", "_")
            output = OUT / (label + ".obj")
            command = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{output.as_posix()}" -c "{source.as_posix()}"', row["command"])
            result = subprocess.run(command, cwd=row["directory"], capture_output=True, text=True)
            log = result.stdout + result.stderr
            (OUT / (label + ".log")).write_text(log, encoding="utf-8")
            summary[label] = {"exit_code": result.returncode, "warnings": log.count("warning:"), "command": command}
            print(f"{label}: exit={result.returncode}, warnings={log.count('warning:')}", flush=True)
            if log:
                print(log, flush=True)
    (OUT / "arm_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    raise SystemExit(any(r["exit_code"] for r in summary.values()))


if __name__ == "__main__":
    main()
