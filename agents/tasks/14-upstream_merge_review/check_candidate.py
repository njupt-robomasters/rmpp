"""只在out中展开候选合并树并执行ARM语法编译，不改工程索引与源码。"""

import io
import json
import os
from pathlib import Path
import shlex
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "agents/out/14-upstream_merge_review"
REPO = OUT / "review.git"
summary = json.loads((OUT / "summary.json").read_text())
tree = summary["results"]["working"]["tree"]
candidate = OUT / ("candidate_" + tree[:8])
candidate.mkdir(exist_ok=True)
archive = subprocess.check_output(["git", f"--git-dir={REPO}", "archive", tree], cwd=ROOT)
with tarfile.open(fileobj=io.BytesIO(archive)) as files:
    files.extractall(candidate, filter="data")

commands = json.loads((ROOT / "build/jelly_sentry/compile_commands.json").read_text())
sources = ["lib/motor/el05.cpp", "lib/robot/shooter_17mm.cpp",
           "app/jelly_sentry/gimbal_fold.cpp", "app/jelly_sentry/gimbal.cpp",
           "app/jelly_sentry/app.cpp", "app/jelly_sentry/chassis.cpp",
           "app/jelly_sentry/shooter.cpp"]
results = []
for source in sources:
    row = next(x for x in commands if x["file"].replace("\\", "/").endswith("/" + source))
    args = shlex.split(row["command"].replace("\\", "/"))
    clean = []
    skip = False
    for arg in args:
        if skip:
            skip = False
            continue
        if arg in ("-o", "-c", "-MF", "-MT", "-MQ"):
            skip = True
            continue
        if arg in ("-MD", "-MMD"):
            continue
        clean.append(arg.replace(ROOT.as_posix(), candidate.as_posix()))
    clean += ["-fsyntax-only", "-fmax-errors=8", str(candidate / source)]
    env = dict(os.environ)
    env["PATH"] = str(Path(clean[0]).parent) + os.pathsep + env["PATH"]
    result = subprocess.run(clean, cwd=candidate, env=env, capture_output=True, text=True)
    log = source.replace("/", "_") + "_candidate.log"
    (OUT / log).write_text(result.stdout + result.stderr, encoding="utf-8")
    errors = [line for line in result.stderr.splitlines() if "error:" in line]
    results.append({"source": source, "exit_code": result.returncode, "errors": errors, "log": log})
report = {"tree": tree, "candidate": str(candidate), "results": results}
(OUT / "candidate_compile.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
print(json.dumps(report, indent=2, ensure_ascii=False))
