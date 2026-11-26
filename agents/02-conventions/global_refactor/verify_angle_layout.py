"""编译角度与类布局调整涉及的真实 ARM 对象，复用已配置的编译数据库。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "agents/tmp/global_refactor/angle_layout/arm"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True, help="提供已有 build 数据库的工作区")
    parser.add_argument("--mc02-build", type=Path, required=True, help="当前 worktree 的 mc02 构建目录")
    args = parser.parse_args()
    source_root = args.source_root.resolve()
    OUT.mkdir(parents=True, exist_ok=True)
    selected = {}
    for preset in ["dummy", "infantry", "sentry", "leg-leg_down", "leg-leg_up", "examples-motor", "examples-gimbal", "test"]:
        database = source_root / "build" / preset / "compile_commands.json"
        for row in json.loads(database.read_text(encoding="utf-8")):
            relative = Path(row["file"]).relative_to(source_root).as_posix()
            if not (ROOT / relative).is_file():
                continue
            if relative.startswith("lib/") and "/third_party/" not in relative:
                selected[("c_board", relative)] = (row, source_root)
            elif relative.startswith("app/") or relative == "boards/c_board/bsp/imu.cpp":
                selected[("c_board", relative)] = (row, source_root)
    for row in json.loads((args.mc02_build / "compile_commands.json").read_text(encoding="utf-8")):
        relative = Path(row["file"]).relative_to(ROOT).as_posix()
        if (relative.startswith("lib/") and "/third_party/" not in relative) or relative == "boards/mc02/bsp/imu.cpp" or relative.startswith("app/"):
            selected[("mc02", relative)] = (row, ROOT)

    def compile_one(item):
        (board, source), (row, original_root) = item
        output = OUT / (board + "_" + source.replace("/", "_") + ".obj")
        command = row["command"]
        command = command.replace(original_root.as_posix() + "/", ROOT.as_posix() + "/")
        command = command.replace(str(original_root) + "\\", str(ROOT) + "\\")
        command = re.sub(r" -o .*? -c .*$", lambda _: f' -Werror=reorder -o "{output.as_posix()}" -c "{(ROOT / source).as_posix()}"', command)
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        text = result.stdout + result.stderr
        (output.with_suffix(".log")).write_text(text, encoding="utf-8")
        return board, source, result.returncode, text

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(compile_one, selected.items()))
    for board, source, code, text in results:
        if text:
            print(f"{board}: {source}\n{text}")
    failures = [(board, source) for board, source, code, text in results if code]
    print(f"ARM: {len(results) - len(failures)}/{len(results)} objects passed; logs: {OUT}")
    if failures:
        raise SystemExit(f"Failed: {failures}")


if __name__ == "__main__":
    main()
