"""Preserve custom Component MOD definitions beside an optimized vehicle."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import shutil
import tempfile
from typing import Tuple

from .definitions import DefinitionCatalog


@dataclass(frozen=True)
class ComponentPackagePlan:
    output_root: Path
    copies: Tuple[Tuple[Path, Path], ...]

    @property
    def component_bin_count(self) -> int:
        return len(self.copies)


def plan_component_package(
    catalog: DefinitionCatalog,
    output_vehicle: Path,
    force: bool,
) -> ComponentPackagePlan:
    """Preflight the BIN files needed by definitions loaded during analysis."""

    sources = sorted(
        {
            definition.source_path.resolve()
            for definition in catalog.loaded_definitions
            if definition.source_format == "vehicle_component_bin"
        }
    )
    output_root = Path(output_vehicle).with_suffix("")
    copies = []
    for source in sources:
        destination = output_root / source.name
        if source == destination.resolve():
            continue
        if destination.exists():
            if destination.read_bytes() == source.read_bytes():
                continue
            if not force:
                raise FileExistsError(
                    "output Component MOD BIN already exists with different data; "
                    "pass --force to replace it: {}".format(destination)
                )
        copies.append((source, destination))
    return ComponentPackagePlan(output_root, tuple(copies))


def install_component_package(plan: ComponentPackagePlan) -> None:
    """Install preflighted BIN files with per-file atomic replacement."""

    if not plan.copies:
        return
    plan.output_root.mkdir(parents=True, exist_ok=True)
    for source, destination in plan.copies:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=destination.name + ".",
            suffix=".tmp",
            dir=str(plan.output_root),
        )
        os.close(descriptor)
        try:
            shutil.copyfile(source, temporary_name)
            os.replace(temporary_name, destination)
        except Exception:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)
            raise
