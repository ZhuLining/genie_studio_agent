#!/usr/bin/env python3
"""真机高精度地图只读探针。

验证链路：
1. import agibot_gdk 并初始化 GDK；
2. 调用 Map.get_all_map() 获取地图下拉候选；
3. 按 --map-id / 当前地图 / 首个地图调用 Map.get_map() 获取详情摘要。

本脚本不调用 switch_map/high_precision_navi/normal_navi，不改变机器人状态。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> int:
    repo_src = Path(__file__).resolve().parents[1] / "src"
    sys.path.insert(0, str(repo_src))

    from gsa_taskflow_executor.gdk.map_probe import (  # noqa: PLC0415
        DEFAULT_MAP_TIMEOUT_MS,
        run_gdk_map_probe,
    )

    parser = argparse.ArgumentParser(description="Read-only GDK map probe")
    parser.add_argument("--map-id", type=int, help="指定 get_map(map_id) 的地图 ID，范围 0-255")
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=DEFAULT_MAP_TIMEOUT_MS,
        help="子进程/现场读取超时时间，默认 5000ms",
    )
    args = parser.parse_args()

    result = run_gdk_map_probe(map_id=args.map_id, timeout_ms=args.timeout_ms)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("available") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
