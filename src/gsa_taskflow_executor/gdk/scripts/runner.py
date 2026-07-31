from __future__ import annotations

import json

from ..control_probe import run_gdk_control_probe


def main(action: str) -> int:
    result = run_gdk_control_probe(action)
    print(json.dumps(result, ensure_ascii=False, separators=(",", ":")))
    return 0
