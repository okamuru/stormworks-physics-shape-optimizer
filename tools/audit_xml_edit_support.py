#!/usr/bin/env python3
"""Inventory exact-model coverage of XML-edited vehicle Components.

This performs parsing, Definition lookup and physics-voxel expansion only.  It
does not run the ordering search, so large local vehicle collections can be
classified without optimizer-scale CPU or memory use.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from swphysics.xml_edit_audit import (  # noqa: E402
    audit_vehicles,
    vehicle_paths,
    workshop_vehicle_paths,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("vehicles", type=Path, nargs="+")
    parser.add_argument("--definitions", type=Path, required=True)
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="recursively scan directory inputs (top-level only by default)",
    )
    parser.add_argument(
        "--workshop-layout",
        action="store_true",
        help="scan only <Workshop ID>/vehicle.xml payloads",
    )
    parser.add_argument("--detail-limit", type=int, default=200)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.detail_limit < 0:
        parser.error("--detail-limit cannot be negative")

    report = audit_vehicles(
        args.definitions,
        (
            workshop_vehicle_paths(args.vehicles)
            if args.workshop_layout
            else vehicle_paths(args.vehicles, args.recursive)
        ),
        args.detail_limit,
    )
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(
            "wrote {} (vehicles={}, unsupported_components={})".format(
                args.output,
                report["totals"].get("parsed_vehicle_count", 0),
                report["totals"].get("unsupported_component_count", 0),
            )
        )
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
