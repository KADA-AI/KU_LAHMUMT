#!/usr/bin/env python3
"""Convert 0401_1.json into NDJSON in scenario_samples."""

from __future__ import annotations

import json
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
SOURCE = BASE_DIR.parent / "0401_종합과제_1107" / "0401_1.json"
TARGET = BASE_DIR / "0401_1.ndjson"


def convert_to_ndjson(source: Path, target: Path) -> int:
    """Convert a JSON array file into NDJSON; returns number of records."""
    with source.open("r", encoding="utf-8") as source_fp:
        payload = json.load(source_fp)

    if not isinstance(payload, list):
        raise ValueError(
            f"Expected list at top level in {source}, found {type(payload).__name__}"
        )

    with target.open("w", encoding="utf-8") as target_fp:
        for item in payload:
            json.dump(item, target_fp, ensure_ascii=False)
            target_fp.write("\n")

    return len(payload)


def main() -> None:
    count = convert_to_ndjson(SOURCE, TARGET)
    print(f"Wrote {count} records to {TARGET}")


if __name__ == "__main__":
    main()
