#!/usr/bin/env python3
"""Convert 0402.json into NDJSON with fixed timestamps."""

from __future__ import annotations

import json
from pathlib import Path

SOURCE = Path(__file__).with_name("0402.json")
TARGET = Path(__file__).with_name("0402.ndjson")
START_TIMESTAMP = 811517583600
STEP_MS = 100


def convert_to_ndjson(source: Path, target: Path) -> int:
    """Convert a JSON array to NDJSON and return the number of records written."""
    with source.open("r", encoding="utf-8") as fp:
        payload = json.load(fp)

    if not isinstance(payload, list):
        raise ValueError(f"Expected list at top level in {source}, found {type(payload).__name__}")

    with target.open("w", encoding="utf-8") as fp:
        for idx, item in enumerate(payload):
            item["timestamp"] = START_TIMESTAMP + idx * STEP_MS
            json.dump(item, fp)
            fp.write("\n")

    return len(payload)


def main() -> None:
    record_count = convert_to_ndjson(SOURCE, TARGET)
    print(f"Wrote {record_count} records to {TARGET}")


if __name__ == "__main__":
    main()
