"""为含 CMake 入口的应用生成默认 Debug 配置预设"""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    configure = [{
        "name": "default", "hidden": True, "generator": "Ninja",
        "binaryDir": "${sourceDir}/build/${presetName}",
    }]
    for entry in sorted((ROOT / "app").rglob("CMakeLists.txt")):
        app = entry.parent.relative_to(ROOT / "app").as_posix()
        configure.append({
            "name": app.replace('/', '-'), "inherits": "default",
            "cacheVariables": {"RMPP_APP": app, "CMAKE_BUILD_TYPE": "Debug"},
        })
    # CLion 会额外导入构建预设，形成重复的“配置名 - 构建名”条目
    presets = {"version": 3, "configurePresets": configure}
    (ROOT / "CMakePresets.json").write_text(json.dumps(presets, indent=4) + "\n", encoding="utf-8")
    print(f"Generated {len(configure) - 1} configure presets")


if __name__ == "__main__":
    main()
