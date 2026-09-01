"""Portable reconstruction of Stormworks Definition buoyancy surfaces.

The game does not flood ordinary physics voxels directly.  It converts each
Definition ``<buoyancy_surfaces>/<surface>`` into one or two oriented native
surface types, inserts
the flipped copy in the adjacent cell, and crawls precomputed edge transitions.
The JSON tables consumed here were extracted from the unmodified build
24749959; this module itself does not load or execute the game binary.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import json
from pathlib import Path
from typing import DefaultDict, Dict, FrozenSet, Iterable, Mapping, Optional, Tuple

from .definitions import DefinitionCatalog, DefinitionSurface
from .model import GridPoint, Matrix3, add_points, apply_matrix, multiply_matrices
from .surface_model import rounded_surface_direction
from .vehicle import ComponentPlacement, Vehicle


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SURFACE_TABLE = ROOT / "analysis" / "surface_table_build_24749959.json"
DEFAULT_SURFACE_RESOLUTION = (
    ROOT / "analysis" / "surface_resolution_build_24749959.json"
)
SurfaceNode = Tuple[GridPoint, int]


@dataclass(frozen=True)
class SurfaceResolution:
    type_count: int
    primary: int
    secondary: int


@dataclass(frozen=True)
class SurfaceCrawl:
    search_angle: int
    position_delta: GridPoint
    target_edge: int
    target_surface_type: int


@dataclass(frozen=True)
class SurfaceEdge:
    edge_id: int
    start: GridPoint
    end: GridPoint
    crawls: Tuple[SurfaceCrawl, ...]


@dataclass(frozen=True)
class SurfaceType:
    surface_type: int
    direction: GridPoint
    flipped_surface_type: int
    opposite_position_delta: GridPoint
    opposite_surface_type: int
    area: float
    edges: Tuple[SurfaceEdge, ...]


@dataclass(frozen=True)
class SurfaceGraphComponent:
    nodes: FrozenSet[SurfaceNode]
    flooder_primary_nodes: FrozenSet[SurfaceNode]
    occupied_position_count: int
    outside_body_bounds_position_count: int

    @property
    def position_count(self) -> int:
        return len({position for position, _surface_type in self.nodes})

    @property
    def contains_flooder_surface(self) -> bool:
        return bool(self.flooder_primary_nodes)


@dataclass(frozen=True)
class BodySurfaceGraph:
    body_index: int
    nodes: FrozenSet[SurfaceNode]
    components: Tuple[SurfaceGraphComponent, ...]
    unresolved_surface_count: int
    native_ignored_surface_count: int


@dataclass(frozen=True)
class CompartmentSurfaceCrawl:
    """One native-style closed edge walk over compartment-facing surfaces."""

    nodes: FrozenSet[SurfaceNode]
    sequence: Tuple[SurfaceNode, ...]
    completed: bool
    edge_step_count: int
    error: str = ""


class _SearchEdgeNode:
    __slots__ = ("key", "previous", "next")

    def __init__(self, key: Tuple[GridPoint, int]):
        self.key = key
        self.previous = self
        self.next = self


class _SearchEdgeContainer:
    __slots__ = ("first",)

    def __init__(self):
        self.first: Optional[_SearchEdgeNode] = None


def _insert_search_edge(
    container: _SearchEdgeContainer,
    anchor: Optional[_SearchEdgeNode],
    key: Tuple[GridPoint, int],
) -> _SearchEdgeNode:
    """Mirror ``search_edge_container::insert_edge``.

    The native ``+0x10`` link is the previous edge and ``+0x18`` is the next
    edge.  New edges are inserted after the supplied anchor.
    """

    node = _SearchEdgeNode(key)
    if anchor is None:
        container.first = node
        return node
    node.previous = anchor
    node.next = anchor.next
    anchor.next.previous = node
    anchor.next = node
    return node


def _remove_search_edge(
    container: _SearchEdgeContainer, node: _SearchEdgeNode
) -> None:
    if node.next is node:
        container.first = None
        return
    if container.first is node:
        container.first = node.next
    node.previous.next = node.next
    node.next.previous = node.previous


def set_surface_occupied_explode(bits: int, surface_type: int) -> int:
    """Apply the native overlapping full-face/sub-face expansion rule.

    Types 0..5 are full cardinal faces.  Types 6..29 are four six-type banks
    of quarter faces.  Once a full face and any matching quarter coexist, the
    native surface voxel drops the full face and exposes all four quarters.
    Types 30..57 do not participate in this expansion.
    """

    bits |= 1 << surface_type
    if surface_type >= 30:
        return bits
    cardinal_type = surface_type % 6
    full_bit = 1 << cardinal_type
    quarter_bits = sum(
        1 << (cardinal_type + 6 * bank) for bank in range(1, 5)
    )
    if bits & full_bit and bits & quarter_bits:
        return (bits & ~(full_bit | quarter_bits)) | quarter_bits
    return bits


def build_body_surface_bits(
    vehicle: Vehicle,
    catalog: DefinitionCatalog,
    body_index: int,
    metadata: Optional["SurfaceMetadata"] = None,
) -> Dict[GridPoint, int]:
    """Return the native global surface tree after overlap expansion."""

    native = metadata or SurfaceMetadata()
    bits_by_position: Dict[GridPoint, int] = {}
    for component in vehicle.bodies[body_index].components:
        for primary, flipped in _component_surface_nodes(
            component, catalog, native
        ):
            for position, surface_type in (primary, flipped):
                bits_by_position[position] = set_surface_occupied_explode(
                    bits_by_position.get(position, 0), surface_type
                )
    return bits_by_position


def _rotate_float_vector(
    matrix: Matrix3, vector: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    x, y, z = vector
    return (
        matrix[0] * x + matrix[3] * y + matrix[6] * z,
        matrix[1] * x + matrix[4] * y + matrix[7] * z,
        matrix[2] * x + matrix[5] * y + matrix[8] * z,
    )


class SurfaceMetadata:
    """Parsed native surface tables with constant-time Definition lookup."""

    def __init__(
        self,
        table_path: Path = DEFAULT_SURFACE_TABLE,
        resolution_path: Path = DEFAULT_SURFACE_RESOLUTION,
    ):
        table = json.loads(Path(table_path).read_text(encoding="utf-8"))
        resolution = json.loads(
            Path(resolution_path).read_text(encoding="utf-8")
        )
        if table["stormworks_build_id"] != resolution["stormworks_build_id"]:
            raise ValueError("surface metadata build IDs do not match")
        if table["binary_sha256"] != resolution["binary_sha256"]:
            raise ValueError("surface metadata binary hashes do not match")
        self.stormworks_build_id = table["stormworks_build_id"]
        self.binary_sha256 = table["binary_sha256"]
        self.max_definition_shape = resolution["max_definition_shape"]
        self.descriptors: Dict[
            Tuple[int, int, int], Tuple[Matrix3, Tuple[float, float, float]]
        ] = {
            (row["orientation"], row["rotation"], row.get("shape", 1)): (
                tuple(row["local_rotation"]),
                tuple(row["local_normal"]),
            )
            for row in resolution["descriptors"]
        }
        self.resolutions: Dict[
            Tuple[Matrix3, int, GridPoint], SurfaceResolution
        ] = {
            (
                tuple(row["rotation"]),
                row["shape"],
                tuple(row["direction"]),
            ): SurfaceResolution(
                type_count=row["type_count"],
                primary=row["primary"],
                secondary=row["secondary"],
            )
            for row in resolution["resolutions"]
        }
        self._directional_resolutions: Dict[
            Tuple[int, GridPoint], SurfaceResolution
        ] = {}
        for (_rotation, shape, direction), item in self.resolutions.items():
            if shape not in (6, 7, 8):
                continue
            key = (shape, direction)
            existing = self._directional_resolutions.setdefault(key, item)
            if existing != item:
                raise ValueError(
                    "native directional surface resolution is ambiguous: "
                    f"{key}"
                )
        self.types: Dict[int, SurfaceType] = {}
        for row in table["types"]:
            edges = tuple(
                SurfaceEdge(
                    edge_id=edge["edge_id"],
                    start=tuple(edge["start"]),
                    end=tuple(edge["end"]),
                    crawls=tuple(
                        SurfaceCrawl(
                            search_angle=crawl["search_angle"],
                            position_delta=tuple(crawl["position_delta"]),
                            target_edge=crawl["target_edge"],
                            target_surface_type=crawl["target_surface_type"],
                        )
                        for crawl in edge["crawls"]
                    ),
                )
                for edge in row["edges"]
            )
            self.types[row["surface_type"]] = SurfaceType(
                surface_type=row["surface_type"],
                direction=tuple(row["direction"]),
                flipped_surface_type=row["flipped_surface_type"],
                opposite_position_delta=tuple(
                    row.get("opposite_position_delta", row["direction"])
                ),
                opposite_surface_type=row.get(
                    "opposite_surface_type", row["flipped_surface_type"]
                ),
                area=row["area"],
                edges=edges,
            )
        self.edges_by_id: Dict[int, SurfaceEdge] = {}
        for surface_type in self.types.values():
            for edge in surface_type.edges:
                existing = self.edges_by_id.setdefault(edge.edge_id, edge)
                if existing != edge:
                    raise ValueError(
                        "native surface edge ID has conflicting metadata: "
                        f"{edge.edge_id}"
                    )

    @staticmethod
    def _cardinal_type(direction: GridPoint) -> Optional[int]:
        """Mirror the native X, then Y, then Z direction priority."""

        x, y, z = direction
        if x == 1:
            return 0
        if x == -1:
            return 1
        if y == 1:
            return 2
        if y == -1:
            return 3
        if z == 1:
            return 4
        if z == -1:
            return 5
        return None

    def _fallback_resolution(
        self,
        rotation: Matrix3,
        shape: int,
        direction: GridPoint,
    ) -> SurfaceResolution:
        """Reproduce ``get_surface_type`` for an arbitrary integer matrix.

        The extracted resolution table covers the game's 48 signed grid
        transforms.  XML editing can supply scaling, shear, or a singular
        matrix, but the game still quantizes those inputs to its existing 58
        surface types.  This is a direct transcription of build 24749959's
        shape 1..8 switch, not an affine-polygon approximation.
        """

        empty = SurfaceResolution(type_count=0, primary=0, secondary=0)
        if shape in (1, 4, 5):
            primary = self._cardinal_type(direction)
            return (
                empty
                if primary is None
                else SurfaceResolution(
                    type_count=1, primary=primary, secondary=0
                )
            )
        if shape == 2:
            base = self._cardinal_type(direction)
            if base is None:
                return empty

            # Native get_surface_type multiplies these two local vectors by
            # the supplied matrix, then compares integer dot products with a
            # cardinal face reference.  Matrix3 is column-major.
            axis_a = (rotation[0], rotation[1], rotation[2])
            axis_b = (-rotation[6], -rotation[7], -rotation[8])
            reference = (
                (0, 0, -1),
                (0, 0, -1),
                (0, 0, -1),
                (0, 0, -1),
                (0, 1, 0),
                (0, -1, 0),
            )[base]

            def dot(left: GridPoint, right: GridPoint) -> int:
                # The native routine uses signed 32-bit imul/add.  Wrapping
                # here also keeps extreme edited values deterministic.
                value = sum(left[index] * right[index] for index in range(3))
                return ((value + 0x80000000) & 0xFFFFFFFF) - 0x80000000

            facing = dot(axis_b, reference)
            if facing == 1:
                quarter = 1
            elif facing == -1:
                quarter = 3
            elif dot(axis_a, reference) == 1:
                quarter = 2
            else:
                quarter = 4
            return SurfaceResolution(
                type_count=2,
                primary=base + 6 * quarter,
                secondary=base + 6 * (quarter % 4) + 6,
            )
        if shape in (6, 7, 8):
            return self._directional_resolutions.get(
                (shape, direction), empty
            )
        return empty

    def lookup(
        self, component_rotation: Matrix3, surface: DefinitionSurface
    ) -> Optional[SurfaceResolution]:
        """Return the build-pinned lookup result, including a zero type count.

        A zero ``type_count`` is an intentional native result for Definition
        records that do not create a buoyancy barrier.  ``None`` is reserved
        for missing metadata, which must not be mistaken for an open face.
        """

        descriptor = self.descriptors.get(
            (surface.orientation, surface.rotation, surface.shape)
        )
        if descriptor is None:
            return None
        local_rotation, local_normal = descriptor
        world_rotation = multiply_matrices(component_rotation, local_rotation)
        direction = rounded_surface_direction(
            _rotate_float_vector(component_rotation, local_normal)
        )
        exact = self.resolutions.get(
            (world_rotation, surface.shape, direction)
        )
        if exact is not None:
            return exact
        return self._fallback_resolution(
            world_rotation, surface.shape, direction
        )

    def resolve(
        self, component_rotation: Matrix3, surface: DefinitionSurface
    ) -> Optional[SurfaceResolution]:
        """Return a native buoyancy surface, or ``None`` for a zero-count row."""

        result = self.lookup(component_rotation, surface)
        if result is None or not result.type_count:
            return None
        return result


def crawl_compartment_surface_nodes(
    surface_bits: Mapping[GridPoint, int],
    seed: SurfaceNode,
    metadata: Optional[SurfaceMetadata] = None,
    max_edge_steps: int = 1_000_000,
) -> CompartmentSurfaceCrawl:
    """Run the native sealed-compartment boundary-ring algorithm.

    The walk follows the extracted edge crawl table.  Closing two edges can
    remove an adjacent pair, split one cyclic boundary into two, or merge two
    boundaries.  Containers are processed last-in-first-out, matching the
    build's ``mm_vector_ptr`` loop.
    """

    native = metadata or SurfaceMetadata()

    def present(node: SurfaceNode) -> bool:
        position, surface_type = node
        return bool(surface_bits.get(position, 0) & (1 << surface_type))

    if not present(seed):
        return CompartmentSurfaceCrawl(
            nodes=frozenset(),
            sequence=(),
            completed=False,
            edge_step_count=0,
            error="seed_surface_missing",
        )

    used = {seed}
    sequence = [seed]
    initial = _SearchEdgeContainer()
    anchor: Optional[_SearchEdgeNode] = None
    seed_position, seed_type = seed
    for edge in native.types[seed_type].edges:
        anchor = _insert_search_edge(
            initial, anchor, (seed_position, edge.edge_id)
        )
    containers = [initial]
    edge_steps = 0

    while containers and edge_steps < max_edge_steps:
        container = containers[-1]
        current = container.first
        if current is None:
            containers.pop()
            continue
        edge_steps += 1
        position, edge_id = current.key
        edge = native.edges_by_id[edge_id]
        chosen = None
        for crawl in edge.crawls:
            target = (
                add_points(position, crawl.position_delta),
                crawl.target_surface_type,
            )
            if present(target):
                chosen = (crawl, target)
                break
        if chosen is None:
            return CompartmentSurfaceCrawl(
                nodes=frozenset(used),
                sequence=tuple(sequence),
                completed=False,
                edge_step_count=edge_steps,
                error="open_search_edge",
            )

        crawl, target = chosen
        target_position, target_type = target
        matching_key = (target_position, crawl.target_edge)
        if target not in used:
            target_edges = native.types[target_type].edges
            matching_index = next(
                index
                for index, target_edge in enumerate(target_edges)
                if target_edge.edge_id == crawl.target_edge
            )
            anchor = current
            index = (matching_index + 1) % len(target_edges)
            while index != matching_index:
                anchor = _insert_search_edge(
                    container,
                    anchor,
                    (target_position, target_edges[index].edge_id),
                )
                index = (index + 1) % len(target_edges)
            _remove_search_edge(container, current)
            used.add(target)
            sequence.append(target)
            continue

        # The native fast path tests the previous edge first, then next.
        matching = None
        if current.previous.key == matching_key:
            matching = current.previous
        elif current.next.key == matching_key:
            matching = current.next
        if matching is not None:
            _remove_search_edge(container, matching)
            _remove_search_edge(container, current)
            continue

        # Search the remaining current ring in forward-link order.
        matching = current.next.next
        while matching is not current.previous:
            if matching.key == matching_key:
                break
            matching = matching.next
        if matching is not current.previous:
            current_previous = current.previous
            current_next = current.next
            matching_previous = matching.previous
            matching_next = matching.next
            _remove_search_edge(container, matching)
            _remove_search_edge(container, current)

            current_previous.next = matching_next
            matching_next.previous = current_previous
            matching_previous.next = current_next
            current_next.previous = matching_previous

            split = _SearchEdgeContainer()
            split.first = current_previous
            containers.append(split)
            continue

        # Otherwise the matching edge may close a different search ring.
        other_match = None
        for container_index, other in enumerate(containers):
            if other is container or other.first is None:
                continue
            candidate = other.first
            while True:
                if candidate.key == matching_key:
                    other_match = (container_index, other, candidate)
                    break
                candidate = candidate.next
                if candidate is other.first:
                    break
            if other_match is not None:
                break
        if other_match is None:
            return CompartmentSurfaceCrawl(
                nodes=frozenset(used),
                sequence=tuple(sequence),
                completed=False,
                edge_step_count=edge_steps,
                error="used_surface_matching_edge_missing",
            )

        container_index, other, matching = other_match
        current_previous = current.previous
        current_next = current.next
        matching_previous = matching.previous
        matching_next = matching.next
        _remove_search_edge(other, matching)
        _remove_search_edge(container, current)

        current_previous.next = matching_next
        matching_next.previous = current_previous
        matching_previous.next = current_next
        current_next.previous = matching_previous
        other.first = None
        containers.pop(container_index)

    completed = not containers
    return CompartmentSurfaceCrawl(
        nodes=frozenset(used),
        sequence=tuple(sequence),
        completed=completed,
        edge_step_count=edge_steps,
        error="" if completed else "edge_step_limit_exceeded",
    )


def _component_surface_nodes(
    component: ComponentPlacement,
    catalog: DefinitionCatalog,
    metadata: SurfaceMetadata,
) -> Iterable[Tuple[SurfaceNode, SurfaceNode]]:
    """Yield ``(primary-side node, flipped adjacent-side node)`` pairs."""

    definition = catalog.load(component.definition_id)
    for surface in component.buoyancy_definition_surfaces(definition):
        resolution = metadata.resolve(component.effective_transform, surface)
        if resolution is None:
            continue
        position = add_points(
            component.position,
            apply_matrix(component.effective_transform, surface.position),
        )
        surface_types = [resolution.primary]
        if resolution.type_count == 2:
            surface_types.append(resolution.secondary)
        for surface_type in surface_types:
            native_type = metadata.types[surface_type]
            yield (
                (position, surface_type),
                (
                    add_points(
                        position, native_type.opposite_position_delta
                    ),
                    native_type.opposite_surface_type,
                ),
            )


def build_body_surface_graph(
    vehicle: Vehicle,
    catalog: DefinitionCatalog,
    body_index: int,
    metadata: Optional[SurfaceMetadata] = None,
) -> BodySurfaceGraph:
    """Reconstruct the build's first-choice surface-edge connectivity graph."""

    native = metadata or SurfaceMetadata()
    body = vehicle.bodies[body_index]
    nodes = set()
    flooder_primary = set()
    unresolved = 0
    native_ignored = 0
    for component in body.components:
        definition = catalog.load(component.definition_id)
        for surface in component.buoyancy_definition_surfaces(definition):
            resolution = native.lookup(component.effective_transform, surface)
            if resolution is None:
                unresolved += 1
            elif not resolution.type_count:
                native_ignored += 1
        for primary, flipped in _component_surface_nodes(
            component, catalog, native
        ):
            nodes.add(primary)
            nodes.add(flipped)
            if definition.water_component_type == 19:
                flooder_primary.add(primary)

    adjacency: DefaultDict[SurfaceNode, set[SurfaceNode]] = defaultdict(set)
    for node in nodes:
        position, surface_type = node
        for edge in native.types[surface_type].edges:
            # Native build_step examines crawls in stored search-angle order
            # and commits to the first surface type present at the target cell.
            for crawl in edge.crawls:
                target = (
                    add_points(position, crawl.position_delta),
                    crawl.target_surface_type,
                )
                if target in nodes:
                    adjacency[node].add(target)
                    adjacency[target].add(node)
                    break

    physics_positions = {
        voxel.position for voxel in vehicle.physics_voxels(catalog, body_index)
    }
    if physics_positions:
        minimum = tuple(
            min(position[axis] for position in physics_positions)
            for axis in range(3)
        )
        maximum = tuple(
            max(position[axis] for position in physics_positions)
            for axis in range(3)
        )
    else:
        minimum = maximum = (0, 0, 0)

    components = []
    seen = set()
    for seed in nodes:
        if seed in seen:
            continue
        connected = {seed}
        seen.add(seed)
        queue = deque((seed,))
        while queue:
            current = queue.popleft()
            for target in adjacency[current]:
                if target not in seen:
                    seen.add(target)
                    connected.add(target)
                    queue.append(target)
        positions = {position for position, _surface_type in connected}
        outside_count = sum(
            any(
                position[axis] < minimum[axis]
                or position[axis] > maximum[axis]
                for axis in range(3)
            )
            for position in positions
        )
        components.append(
            SurfaceGraphComponent(
                nodes=frozenset(connected),
                flooder_primary_nodes=frozenset(connected & flooder_primary),
                occupied_position_count=len(positions & physics_positions),
                outside_body_bounds_position_count=outside_count,
            )
        )
    components.sort(key=lambda item: (-len(item.nodes), sorted(item.nodes)))
    return BodySurfaceGraph(
        body_index=body_index,
        nodes=frozenset(nodes),
        components=tuple(components),
        unresolved_surface_count=unresolved,
        native_ignored_surface_count=native_ignored,
    )
