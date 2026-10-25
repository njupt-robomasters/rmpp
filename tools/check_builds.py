"""Build every preset; keep complete compiler output under build/checks."""

import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--ninja")
    parser.add_argument("--jobs", type=int, default=8)
    args = parser.parse_args()
    logs = ROOT / "build/checks"
    logs.mkdir(parents=True, exist_ok=True)
    presets = json.loads((ROOT / "CMakePresets.json").read_text(encoding="utf-8"))
    configure_presets = [preset for preset in presets["configurePresets"] if not preset.get("hidden", False)]
    results = []
    for preset in configure_presets:
        name = preset["name"]
        configure = [args.cmake, "--preset", name]
        if args.ninja:
            configure.append("-DCMAKE_MAKE_PROGRAM=" + args.ninja)
        with (logs / (name + ".log")).open("w", encoding="utf-8") as log:
            subprocess.run(configure, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
            subprocess.run([args.cmake, "--build", str(ROOT / "build" / name), "--parallel", str(args.jobs)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        results.append(name)
        print(f"{len(results)}/{len(configure_presets)}: {name} passed", flush=True)
    (logs / "passed.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
