"""使用与 CLion 相同的 clang-format 整理手写 C/C++；--check 只检查。"""

import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ("lib/mavlink/lib/", "lib/ui/lib/", "lib/imu/lib/ekf/", "/storage/flashdb/")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clang-format", default="clang-format")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    files = []
    for directory in (ROOT / "app", ROOT / "lib", ROOT / "tests", *sorted((ROOT / "boards").glob("*/bsp"))):
        for path in directory.rglob("*"):
            relative = path.relative_to(ROOT).as_posix()
            if path.suffix not in (".c", ".h", ".cpp", ".hpp") or any(part in relative for part in VENDOR):
                continue
            # BMI088 原 C 驱动保持第三方排版；通信适配由项目维护。
            if relative.startswith("lib/imu/lib/bmi088/") and not path.name.startswith("bmi088_middleware"):
                continue
            files.append(str(path))
    mode = ["--dry-run", "--Werror"] if args.check else ["-i"]
    for start in range(0, len(files), 30):
        subprocess.run([args.clang_format, "--style=file", *mode, *sorted(files)[start:start + 30]], cwd=ROOT, check=True)
    print(f"{'Checked' if args.check else 'Formatted'} {len(files)} files")


if __name__ == "__main__":
    main()
