"""使用与 CLion 相同的 clang-format 整理手写 C/C++；--check 只检查。"""

import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ("lib/mavlink/lib/", "/bsp/flashdb/")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clang-format", default="clang-format")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("paths", nargs="*", help="可选文件或目录，省略时检查全部手写源码")
    args = parser.parse_args()
    selected = [(ROOT / path).resolve() for path in args.paths]
    for path in selected:
        if not path.exists() or not path.is_relative_to(ROOT):
            parser.error(f"路径不存在或不在项目内: {path}")
    files = []
    for directory in (ROOT / "app", ROOT / "lib", ROOT / "tests", *sorted((ROOT / "boards").glob("*/bsp"))):
        for path in directory.rglob("*"):
            if selected and not any(path == item or item in path.parents for item in selected):
                continue
            relative = path.relative_to(ROOT).as_posix()
            if path.suffix not in (".c", ".h", ".cpp", ".hpp") or any(part in relative for part in VENDOR):
                continue
            files.append(str(path))
    mode = ["--dry-run", "--Werror"] if args.check else ["-i"]
    for start in range(0, len(files), 30):
        subprocess.run([args.clang_format, "--style=file", *mode, *sorted(files)[start:start + 30]], cwd=ROOT, check=True)
    print(f"{'Checked' if args.check else 'Formatted'} {len(files)} files")


if __name__ == "__main__":
    main()
