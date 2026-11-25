"""编译电机可选反馈改动涉及的真实 C 板 ARM 对象：各预设的全部 lib/、app/ 手写源码，覆盖 DM 模板实例化。"""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parents[2] / "out/10-global_refactor/motor_feedback/arm"
PRESETS = ["dummy", "infantry", "sentry", "leg-leg_down", "leg-leg_up", "examples-motor", "examples-gimbal", "test"]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    selected = {}
    for preset in PRESETS:
        for row in json.loads((ROOT / "build" / preset / "compile_commands.json").read_text(encoding="utf-8")):
            path = Path(row["file"]).resolve()
            relative = path.relative_to(ROOT).as_posix()
            if not path.is_file() or not (relative.startswith("lib/") or relative.startswith("app/")) or "/third_party/" in relative:
                continue
            selected[(preset, relative)] = row

    def compile_one(item):
        (preset, source), row = item
        output = OUT / (preset + "_" + source.replace("/", "_") + ".obj")
        command = re.sub(r" -o .*? -c .*$", lambda _: f' -o "{output.as_posix()}" -c "{(ROOT / source).as_posix()}"', row["command"])
        result = subprocess.run(command, cwd=row["directory"], capture_output=True, text=True)
        text = result.stdout + result.stderr
        output.with_suffix(".log").write_text(text, encoding="utf-8")
        return preset, source, result.returncode, text

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(compile_one, selected.items()))
    failed = [r for r in results if r[2] != 0]
    warned = [r for r in results if r[2] == 0 and "warning" in r[3]]
    for preset, source, _, text in failed + warned:
        print(f"== {preset} {source}\n{text}")
    print(f"compiled {len(results)}, failed {len(failed)}, warned {len(warned)}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
