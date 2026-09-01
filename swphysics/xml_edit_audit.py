"""Fast, optimizer-free inventory of XML-edit physics coverage."""

from __future__ import annotations

from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable, Sequence

from .definitions import DefinitionCatalog
from .physics_support import classify_component_physics_support
from .portable_merge import STORMWORKS_BUILD_ID
from .vehicle import load_vehicle


class _VehicleDefinitionCatalog:
    """Reuse standard Definitions while retaining per-vehicle BIN fallback."""

    def __init__(
        self,
        standard: DefinitionCatalog,
        packaged: DefinitionCatalog,
    ) -> None:
        self.standard = standard
        self.packaged = packaged

    def load(self, definition_id: str):
        try:
            return self.standard.load(definition_id)
        except FileNotFoundError:
            return self.packaged.load(definition_id)


def _vehicle_label(path: Path) -> str:
    if path.name == "vehicle.xml" and path.parent.name.isdigit():
        return "{}/vehicle.xml".format(path.parent.name)
    return path.name


def vehicle_paths(inputs: Sequence[Path], recursive: bool) -> tuple[Path, ...]:
    """Resolve files and directories into a deterministic, unique worklist."""

    paths = []
    for item in inputs:
        if item.is_file():
            paths.append(item)
        elif item.is_dir():
            pattern = "**/*.xml" if recursive else "*.xml"
            paths.extend(item.glob(pattern))
        else:
            raise FileNotFoundError("vehicle input not found: {}".format(item))
    return tuple(dict.fromkeys(sorted(path.resolve() for path in paths)))


def workshop_vehicle_paths(inputs: Sequence[Path]) -> tuple[Path, ...]:
    """Return only ``<Workshop ID>/vehicle.xml`` payloads.

    Workshop content roots also contain microcontrollers, playlists and MOD
    metadata XML.  Treating those as empty vehicles distorts coverage totals.
    """

    paths = []
    for item in inputs:
        if item.is_file():
            if item.name != "vehicle.xml":
                raise ValueError(
                    "Workshop vehicle file must be named vehicle.xml: {}".format(
                        item
                    )
                )
            paths.append(item)
            continue
        if not item.is_dir():
            raise FileNotFoundError("Workshop input not found: {}".format(item))
        direct = item / "vehicle.xml"
        if direct.is_file():
            paths.append(direct)
        else:
            paths.extend(item.glob("*/vehicle.xml"))
    return tuple(dict.fromkeys(sorted(path.resolve() for path in paths)))


def audit_vehicles(
    definitions_root: Path,
    paths: Iterable[Path],
    detail_limit: int = 200,
) -> dict:
    """Return aggregate coverage and bounded actionable examples.

    This performs parsing, Definition lookup and physics-voxel expansion only;
    it never runs component-order search.
    """

    totals = Counter()
    issues = Counter()
    unsupported_transform_kinds = Counter()
    issue_definitions = defaultdict(Counter)
    unsupported_examples = []
    errors = []
    standard_catalog = DefinitionCatalog(definitions_root)

    for path in paths:
        vehicle_label = _vehicle_label(path)
        totals["vehicle_file_count"] += 1
        try:
            vehicle = load_vehicle(path)
        except Exception as error:
            totals["vehicle_parse_error_count"] += 1
            if len(errors) < detail_limit:
                errors.append(
                    {
                        "vehicle": vehicle_label,
                        "stage": "vehicle_xml",
                        "error": "{}: {}".format(type(error).__name__, error),
                    }
                )
            continue

        totals["parsed_vehicle_count"] += 1
        catalog = _VehicleDefinitionCatalog(
            standard_catalog,
            DefinitionCatalog.for_vehicle(definitions_root, path),
        )
        for body in vehicle.bodies:
            totals["body_count"] += 1
            body_voxels = []
            insertion_index = 0
            unresolved_indices = set()
            for component in body.components:
                totals["component_count"] += 1
                try:
                    definition = catalog.load(component.definition_id)
                    component_voxels = component.world_physics_voxels(
                        definition,
                        body.index,
                        body.body_id,
                        insertion_index,
                    )
                except Exception as error:
                    totals["definition_or_expansion_error_count"] += 1
                    unresolved_indices.add(component.index)
                    issues["definition_or_expansion_error"] += 1
                    issue_definitions["definition_or_expansion_error"][
                        component.definition_id
                    ] += 1
                    if len(errors) < detail_limit:
                        errors.append(
                            {
                                "vehicle": vehicle_label,
                                "body_index": body.index,
                                "component_index": component.index,
                                "definition_id": component.definition_id,
                                "stage": "definition_or_expansion",
                                "error": "{}: {}".format(
                                    type(error).__name__, error
                                ),
                            }
                        )
                    continue
                body_voxels.extend(component_voxels)
                insertion_index += len(component_voxels)

            totals["physics_voxel_count"] += len(body_voxels)
            coverage = classify_component_physics_support(
                body.components, body_voxels
            )
            for result in coverage:
                if result.component_index in unresolved_indices:
                    continue
                if result.supported:
                    totals["supported_component_count"] += 1
                    continue
                totals["unsupported_component_count"] += 1
                unsupported_transform_kinds[result.transform_kind] += 1
                for issue_code in result.issue_codes:
                    issues[issue_code] += 1
                    issue_definitions[issue_code][result.definition_id] += 1
                if len(unsupported_examples) < detail_limit:
                    unsupported_examples.append(
                        {
                            "vehicle": vehicle_label,
                            "body_index": body.index,
                            "component_index": result.component_index,
                            "definition_id": result.definition_id,
                            "physics_voxel_count": result.physics_voxel_count,
                            "non_cube_voxel_count": result.non_cube_voxel_count,
                            "transform_kind": result.transform_kind,
                            "issues": list(result.issue_codes),
                        }
                    )

    return {
        "schema": "swphysics-xml-edit-support-audit-v1",
        "stormworks_build_id": STORMWORKS_BUILD_ID,
        "definitions_root": str(definitions_root),
        "totals": dict(sorted(totals.items())),
        "issue_counts": dict(issues.most_common()),
        "unsupported_transform_kinds": dict(
            unsupported_transform_kinds.most_common()
        ),
        "top_definitions_by_issue": {
            issue: dict(counter.most_common(20))
            for issue, counter in sorted(issue_definitions.items())
        },
        "unsupported_examples_truncated": (
            totals["unsupported_component_count"] > len(unsupported_examples)
        ),
        "error_examples_truncated": (
            totals["vehicle_parse_error_count"]
            + totals["definition_or_expansion_error_count"]
            > len(errors)
        ),
        "unsupported_examples": unsupported_examples,
        "errors": errors,
    }
