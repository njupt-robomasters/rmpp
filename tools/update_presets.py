"""Generate Debug/Release presets for applications with a CMake entry."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    configure = [{
        "name": "default", "hidden": True, "generator": "Ninja",
        "binaryDir": "${sourceDir}/build/${presetName}",
    }]
    build = []
    for entry in sorted((ROOT / "app").rglob("CMakeLists.txt")):
        app = entry.parent.relative_to(ROOT / "app").as_posix()
        for mode in ("Debug", "Release"):
            name = f"{app.replace('/', '-')}-{mode.lower()}"
            configure.append({
                "name": name, "inherits": "default",
                "cacheVariables": {"RMPP_APP": app, "CMAKE_BUILD_TYPE": mode},
            })
            build.append({"name": name, "configurePreset": name})
    presets = {"version": 3, "configurePresets": configure, "buildPresets": build}
    (ROOT / "CMakePresets.json").write_text(json.dumps(presets, indent=4) + "\n", encoding="utf-8")
    print(f"Generated {len(build)} build presets")


if __name__ == "__main__":
    main()
