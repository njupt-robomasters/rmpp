"""编译哨兵及调试入口的对象目标，日志留在 out，不链接固件。"""

import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "agents/out/14-upstream_merge_review"
OUT.mkdir(exist_ok=True)
cache = (ROOT / "build/jelly_sentry/CMakeCache.txt").read_text(encoding="utf-8")
cmake = re.search(r"CMAKE_COMMAND:INTERNAL=(.*)", cache)[1]
ninja = re.search(r"CMAKE_MAKE_PROGRAM:FILEPATH=(.*)", cache)[1]
rows = json.loads((ROOT / "build/jelly_sentry/compile_commands.json").read_text())
compiler = rows[0]["command"].split(" ")[0].strip('"')
env = {**os.environ, "PATH": os.pathsep.join([str(Path(compiler).parent), str(Path(ninja).parent), os.environ["PATH"]])}
presets = json.loads((ROOT / "CMakePresets.json").read_text())["configurePresets"]
results = {}
for item in presets:
    preset = item["name"]
    if preset != "jelly_sentry" and not preset.startswith("jelly_sentry_debug-"):
        continue
    targets = ["rmpp_app"]
    if preset == "jelly_sentry":
        targets += ["rmpp_lib", "rmpp_bsp", "CMakeFiles/rmpp.dir/boards/mc02/Core/Src/fdcan.c.obj"]
    command = [cmake, "--build", f"build/{preset}", "--target", *targets, "-j", "4"]
    result = subprocess.run(command, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log = result.stdout.decode("utf-8", errors="replace")
    (OUT / f"{preset}_compile.log").write_text(log, encoding="utf-8")
    results[preset] = {"exit_code": result.returncode, "warnings": log.count("warning:"), "command": command}
    print(f"{preset}: exit={result.returncode}, warnings={log.count('warning:')}", flush=True)
    if result.returncode:
        print(log, flush=True)
(OUT / "compile_summary.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
raise SystemExit(any(r["exit_code"] for r in results.values()))
