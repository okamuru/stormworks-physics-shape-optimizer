"""Portable sealed-volume reconstruction from Definition buoyancy surfaces.

Build 24749959 converts each
``Definition/<buoyancy_surfaces>/surface`` record into a
native polygon type.  Those polygons, after component rotation and placement,
are the barriers used by the Physics Flooder.  This module reconstructs the
same barriers from the extracted static Surface Table and floods a half-cell
lattice without loading or executing the game binary.

Surface vertices use one eighth of a metre as their integer unit.  The lattice
uses centres halfway between those vertices, so an axis-aligned move crosses a
surface exactly when its segment intersects one of the polygon triangles.
"""

from __future__ import annotations

from array import array
from dataclasses import dataclass, replace
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .definitions import ComponentDefinition, DefinitionCatalog
from .model import GridPoint, IDENTITY_MATRIX, WorldVoxel, add_points, apply_matrix
from .surface_graph import (
    SurfaceMetadata,
    build_body_surface_bits,
    crawl_compartment_surface_nodes,
    set_surface_occupied_explode,
)
from .vehicle import Vehicle


Point3 = Tuple[int, int, int]
Triangle = Tuple[Point3, Point3, Point3]


@dataclass(frozen=True)
class SurfaceFloodFillResult:
    supported: bool
    status: str
    static_voxels: Tuple[WorldVoxel, ...]
    static_voxels_after_fill: Tuple[WorldVoxel, ...]
    new_fill_voxels: Tuple[WorldVoxel, ...]
    fill_call_positions: Tuple[GridPoint, ...]
    sealed_compartment_count: int
    open_flooder_count: int
    scan_microcell_count: int
    surface_polygon_count: int
    surface_triangle_count: int
    blocked_microedge_count: int
    partial_volume_excluded_count: int
    metadata_missing_surface_count: int
    native_ignored_surface_count: int
    stormworks_build_id: str
    binary_sha256: str
    metadata_missing_component_indices: Tuple[int, ...] = ()

    @property
    def all_voxels(self) -> Tuple[WorldVoxel, ...]:
        return self.static_voxels_after_fill + self.new_fill_voxels

    @property
    def filled_positions(self) -> Tuple[GridPoint, ...]:
        """Compatibility alias for callers of the earlier cube-only model."""

        return self.fill_call_positions


@dataclass(frozen=True)
class _BodySurfaceAnalysis:
    bits: Dict[GridPoint, int]
    vertex_bounds: Optional[Tuple[Point3, Point3]]
    metadata_missing_count: int
    metadata_missing_component_indices: Tuple[int, ...]
    native_ignored_count: int


def _build_body_surface_analysis(
    vehicle: Vehicle,
    catalog: DefinitionCatalog,
    body_index: int,
    metadata: SurfaceMetadata,
    component_definitions: Optional[Sequence[ComponentDefinition]] = None,
) -> _BodySurfaceAnalysis:
    """Resolve repeated Definition/rotation pairs once and translate them."""

    bits_by_position: Dict[GridPoint, int] = {}
    template_cache = {}
    minimum: Optional[List[int]] = None
    maximum: Optional[List[int]] = None
    metadata_missing_count = 0
    metadata_missing_component_indices: Set[int] = set()
    native_ignored_count = 0

    for component_index, component in enumerate(
        vehicle.bodies[body_index].components
    ):
        transform = component.effective_transform
        key = component.buoyancy_surface_template_key()
        template = template_cache.get(key)
        if template is None:
            definition = (
                component_definitions[component_index]
                if component_definitions is not None
                else catalog.load(component.definition_id)
            )
            pairs = []
            local_minimum: Optional[List[int]] = None
            local_maximum: Optional[List[int]] = None
            # A standard door without a complete openable-surface record
            # cannot be treated as an ordinary open hole.  Mark that case
            # unsupported so callers exclude only the Flooder prediction
            # instead of inventing a wrong filled volume.  Custom doors keep
            # the pre-existing Definition-surface path: excluding an entire
            # Body merely because it contains a custom-door part would be a
            # large and unrelated optimization regression.
            missing = int(bool(definition.standard_door_seal_error))
            ignored = 0
            for surface in component.buoyancy_definition_surfaces(definition):
                resolution = metadata.lookup(transform, surface)
                if resolution is None:
                    missing += 1
                    continue
                if not resolution.type_count:
                    ignored += 1
                    continue
                local_position = apply_matrix(transform, surface.position)
                surface_types = [resolution.primary]
                if resolution.type_count == 2:
                    surface_types.append(resolution.secondary)
                for surface_type in surface_types:
                    native_type = metadata.types[surface_type]
                    flipped_position = add_points(
                        local_position,
                        native_type.opposite_position_delta,
                    )
                    pairs.append(
                        (
                            (local_position, surface_type),
                            (
                                flipped_position,
                                native_type.opposite_surface_type,
                            ),
                        )
                    )
                    for edge in native_type.edges:
                        vertex = tuple(
                            4 * local_position[axis] + 2 * edge.start[axis]
                            for axis in range(3)
                        )
                        if local_minimum is None:
                            local_minimum = list(vertex)
                            local_maximum = list(vertex)
                        else:
                            for axis in range(3):
                                local_minimum[axis] = min(
                                    local_minimum[axis], vertex[axis]
                                )
                                local_maximum[axis] = max(
                                    local_maximum[axis], vertex[axis]
                                )
            template = (
                tuple(pairs),
                missing,
                ignored,
                tuple(local_minimum) if local_minimum is not None else None,
                tuple(local_maximum) if local_maximum is not None else None,
            )
            template_cache[key] = template

        pairs, missing, ignored, local_minimum, local_maximum = template
        metadata_missing_count += missing
        if missing:
            metadata_missing_component_indices.add(component_index)
        native_ignored_count += ignored
        for primary, flipped in pairs:
            for local_position, surface_type in (primary, flipped):
                position = add_points(component.position, local_position)
                bits_by_position[position] = set_surface_occupied_explode(
                    bits_by_position.get(position, 0), surface_type
                )
        if local_minimum is not None and local_maximum is not None:
            translated_minimum = tuple(
                local_minimum[axis] + 4 * component.position[axis]
                for axis in range(3)
            )
            translated_maximum = tuple(
                local_maximum[axis] + 4 * component.position[axis]
                for axis in range(3)
            )
            if minimum is None:
                minimum = list(translated_minimum)
                maximum = list(translated_maximum)
            else:
                for axis in range(3):
                    minimum[axis] = min(minimum[axis], translated_minimum[axis])
                    maximum[axis] = max(maximum[axis], translated_maximum[axis])

    bounds = (
        (tuple(minimum), tuple(maximum))
        if minimum is not None and maximum is not None
        else None
    )
    return _BodySurfaceAnalysis(
        bits=bits_by_position,
        vertex_bounds=bounds,  # type: ignore[arg-type]
        metadata_missing_count=metadata_missing_count,
        metadata_missing_component_indices=tuple(
            sorted(metadata_missing_component_indices)
        ),
        native_ignored_count=native_ignored_count,
    )


def _subtract(left: Point3, right: Point3) -> Point3:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def _cross(left: Point3, right: Point3) -> Point3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _segment_intersects_triangle(
    start: Point3, end: Point3, triangle: Triangle
) -> bool:
    """Moller-Trumbore segment test with boundary-inclusive triangle edges."""

    vertex0, vertex1, vertex2 = triangle
    direction = _subtract(end, start)
    edge1 = _subtract(vertex1, vertex0)
    edge2 = _subtract(vertex2, vertex0)
    h = _cross(direction, edge2)
    determinant = _dot(edge1, h)
    epsilon = 1e-9
    if abs(determinant) < epsilon:
        return False
    inverse = 1.0 / determinant
    offset = _subtract(start, vertex0)
    u = inverse * _dot(offset, h)
    if u < -epsilon or u > 1.0 + epsilon:
        return False
    q = _cross(offset, edge1)
    v = inverse * _dot(direction, q)
    if v < -epsilon or u + v > 1.0 + epsilon:
        return False
    distance = inverse * _dot(edge2, q)
    # Lattice nodes are deliberately off the polygon vertices.  Excluding the
    # segment endpoints avoids blocking two edges for one shared node.
    return epsilon < distance < 1.0 - epsilon


def _interior_odd_values(minimum: int, maximum: int) -> range:
    """Odd lattice coordinates strictly inside even polygon bounds."""

    return range(minimum + 1, maximum, 2)


def _crossing_odd_values(minimum: int, maximum: int) -> range:
    """Odd segment starts whose two-unit edge can cross the given span."""

    return range(minimum - 1, maximum + 1, 2)


def _lattice_axis(minimum: int, maximum: int) -> range:
    lower = minimum - 1
    upper = maximum + 1
    start = lower if lower % 2 else lower + 1
    return range(start, upper + 1, 2)


def _grid_position(microcell: Point3) -> GridPoint:
    return tuple((value + 2) // 4 for value in microcell)  # type: ignore[return-value]


_VOLUME_ENTER_PREVIOUS_MASK = 0x10410410
_VOLUME_EXIT_CURRENT_MASK = 0xF30C4C40000000
_VOLUME_ENTER_CURRENT_MASK = 0x120CF31100000000
_VOLUME_EXIT_NEXT_MASK = 0x20820820


def _native_volume_scan(
    candidates: Set[GridPoint], surface_bits: Dict[GridPoint, int]
) -> Set[GridPoint]:
    """Return candidate cells passing native two-half Z-column volume scan."""

    accepted: Set[GridPoint] = set()
    columns: Dict[Tuple[int, int], List[int]] = {}
    for x, y, z in candidates:
        bounds = columns.get((x, y))
        if bounds is None:
            columns[(x, y)] = [z, z]
        else:
            if z < bounds[0]:
                bounds[0] = z
            if z > bounds[1]:
                bounds[1] = z

    for (x, y), (minimum_z, maximum_z) in columns.items():
        previous_two = surface_bits.get((x, y, minimum_z - 2), 0)
        previous_one = surface_bits.get((x, y, minimum_z - 1), 0)
        inside = False
        for z in range(minimum_z - 2, maximum_z + 2):
            next_bits = surface_bits.get((x, y, z + 1), 0)

            if previous_two & _VOLUME_ENTER_PREVIOUS_MASK:
                inside = True
            first_half_inside = inside

            if previous_one & _VOLUME_EXIT_CURRENT_MASK:
                inside = False
            if previous_one & _VOLUME_ENTER_CURRENT_MASK:
                inside = True
            second_half_inside = inside

            if (
                first_half_inside
                and second_half_inside
                and (x, y, z) in candidates
            ):
                accepted.add((x, y, z))

            if next_bits & _VOLUME_EXIT_NEXT_MASK:
                inside = False
            previous_two = previous_one
            previous_one = next_bits
    return accepted


def _set_packed_bit(storage: bytearray, bit_index: int) -> None:
    storage[bit_index >> 3] |= 1 << (bit_index & 7)


def _packed_bit(storage: bytearray, bit_index: int) -> bool:
    return bool(storage[bit_index >> 3] & (1 << (bit_index & 7)))


@dataclass(frozen=True)
class _CompactSurfaceLattice:
    """Half-cell lattice stored as packed bits instead of Python point sets."""

    minimum: Point3
    dimensions: Point3
    blocked_edges: bytearray
    polygon_count: int
    triangle_count: int
    blocked_edge_count: int

    @property
    def node_count(self) -> int:
        return self.dimensions[0] * self.dimensions[1] * self.dimensions[2]


def _surface_block_patterns(
    metadata: SurfaceMetadata,
) -> Tuple[Tuple[Tuple[Point3, int], ...], ...]:
    """Precompute crossed half-cell edges for each of 58 native surface types."""

    patterns = []
    for surface_type in range(58):
        native_type = metadata.types[surface_type]
        vertices = tuple(
            tuple(2 * coordinate for coordinate in edge.start)
            for edge in native_type.edges
        )
        blocked: Set[Tuple[Point3, int]] = set()
        if len(vertices) >= 3:
            for triangle_index in range(1, len(vertices) - 1):
                triangle = (
                    vertices[0],
                    vertices[triangle_index],
                    vertices[triangle_index + 1],
                )
                triangle_minimum = [
                    min(point[axis] for point in triangle)
                    for axis in range(3)
                ]
                triangle_maximum = [
                    max(point[axis] for point in triangle)
                    for axis in range(3)
                ]
                for move_axis in range(3):
                    side_axis = (move_axis + 1) % 3
                    depth_axis = (move_axis + 2) % 3
                    for side in _interior_odd_values(
                        triangle_minimum[side_axis],
                        triangle_maximum[side_axis],
                    ):
                        for depth in _interior_odd_values(
                            triangle_minimum[depth_axis],
                            triangle_maximum[depth_axis],
                        ):
                            for moving in _crossing_odd_values(
                                triangle_minimum[move_axis],
                                triangle_maximum[move_axis],
                            ):
                                start = [0, 0, 0]
                                end = [0, 0, 0]
                                start[move_axis] = moving
                                end[move_axis] = moving + 2
                                start[side_axis] = end[side_axis] = side
                                start[depth_axis] = end[depth_axis] = depth
                                start_point = tuple(start)
                                if _segment_intersects_triangle(
                                    start_point, tuple(end), triangle
                                ):
                                    blocked.add((start_point, move_axis))
        patterns.append(tuple(sorted(blocked)))
    return tuple(patterns)


def _compact_surface_lattice(
    surface_bits: Dict[GridPoint, int],
    metadata: SurfaceMetadata,
    axes: Tuple[range, range, range],
) -> _CompactSurfaceLattice:
    """Rasterize only crossed micro-edges into a compact packed bit field.

    The previous implementation materialized every lattice coordinate and
    every outside coordinate as nested Python tuples.  A large but mostly
    empty body therefore used hundreds of bytes per half-cell.  Native code
    stores spatial state compactly and visits only useful regions; this keeps
    the same geometric intersection contract while using three bits per node.
    """

    minimum = tuple(axis.start for axis in axes)
    dimensions = tuple(len(axis) for axis in axes)
    node_count = dimensions[0] * dimensions[1] * dimensions[2]
    blocked = bytearray((node_count * 3 + 7) // 8)
    polygon_count = 0
    triangle_count = 0
    blocked_count = 0
    maximum = tuple(
        minimum[axis] + 2 * (dimensions[axis] - 1) for axis in range(3)
    )
    stride = (dimensions[1] * dimensions[2], dimensions[2], 1)
    block_patterns = _surface_block_patterns(metadata)

    for position, bits in surface_bits.items():
        remaining_bits = bits
        while remaining_bits:
            lowest_bit = remaining_bits & -remaining_bits
            surface_type = lowest_bit.bit_length() - 1
            remaining_bits ^= lowest_bit
            native_type = metadata.types[surface_type]
            opposite_position = (
                position[0] + native_type.opposite_position_delta[0],
                position[1] + native_type.opposite_position_delta[1],
                position[2] + native_type.opposite_position_delta[2],
            )
            opposite_node = (
                opposite_position,
                native_type.opposite_surface_type,
            )
            if (
                surface_bits.get(opposite_position, 0)
                & (1 << native_type.opposite_surface_type)
                and (position, surface_type) > opposite_node
            ):
                continue
            edge_count = len(native_type.edges)
            if edge_count < 3:
                continue
            polygon_count += 1
            triangle_count += edge_count - 2
            origin_x = 4 * position[0]
            origin_y = 4 * position[1]
            origin_z = 4 * position[2]
            for relative_start, move_axis in block_patterns[surface_type]:
                start_x = origin_x + relative_start[0]
                start_y = origin_y + relative_start[1]
                start_z = origin_z + relative_start[2]
                if (
                    start_x < minimum[0]
                    or start_y < minimum[1]
                    or start_z < minimum[2]
                    or start_x + (2 if move_axis == 0 else 0) > maximum[0]
                    or start_y + (2 if move_axis == 1 else 0) > maximum[1]
                    or start_z + (2 if move_axis == 2 else 0) > maximum[2]
                ):
                    continue
                start_index = (
                    ((start_x - minimum[0]) // 2) * stride[0]
                    + ((start_y - minimum[1]) // 2) * stride[1]
                    + ((start_z - minimum[2]) // 2)
                )
                bit_index = start_index * 3 + move_axis
                byte_index = bit_index >> 3
                mask = 1 << (bit_index & 7)
                if not blocked[byte_index] & mask:
                    blocked[byte_index] |= mask
                    blocked_count += 1

    return _CompactSurfaceLattice(
        minimum=minimum,
        dimensions=dimensions,
        blocked_edges=blocked,
        polygon_count=polygon_count,
        triangle_count=triangle_count,
        blocked_edge_count=blocked_count,
    )


def _compact_region_from_seed(
    lattice: _CompactSurfaceLattice,
    seed: Point3,
) -> Tuple[bool, Set[GridPoint], int, bytearray]:
    """Visit one sample region, stopping as soon as it reaches the exterior."""

    nx, ny, nz = lattice.dimensions
    minimum = lattice.minimum
    offsets = tuple(seed[axis] - minimum[axis] for axis in range(3))
    visited = bytearray((lattice.node_count + 7) // 8)
    if any(
        offset < 0
        or offset % 2
        or offset // 2 >= lattice.dimensions[axis]
        for axis, offset in enumerate(offsets)
    ):
        return True, set(), 0, visited

    ix, iy, iz = (offset // 2 for offset in offsets)
    seed_index = (ix * ny + iy) * nz + iz
    queue_type = "I" if lattice.node_count <= 0xFFFFFFFF else "Q"
    queue = array(queue_type, (seed_index,))
    _set_packed_bit(visited, seed_index)
    candidates: Set[GridPoint] = set()
    cursor = 0
    reached_count = 0
    stride_x = ny * nz
    stride_y = nz

    while cursor < len(queue):
        node_index = queue[cursor]
        cursor += 1
        reached_count += 1
        ix, remainder = divmod(node_index, stride_x)
        iy, iz = divmod(remainder, nz)
        if ix == 0 or iy == 0 or iz == 0 or ix == nx - 1 or iy == ny - 1 or iz == nz - 1:
            return True, set(), reached_count, visited

        point = (
            minimum[0] + 2 * ix,
            minimum[1] + 2 * iy,
            minimum[2] + 2 * iz,
        )
        candidates.add(_grid_position(point))
        neighbors = (
            (node_index + stride_x, node_index * 3),
            (node_index - stride_x, (node_index - stride_x) * 3),
            (node_index + stride_y, node_index * 3 + 1),
            (node_index - stride_y, (node_index - stride_y) * 3 + 1),
            (node_index + 1, node_index * 3 + 2),
            (node_index - 1, (node_index - 1) * 3 + 2),
        )
        for neighbor, blocked_bit in neighbors:
            if (
                not _packed_bit(lattice.blocked_edges, blocked_bit)
                and not _packed_bit(visited, neighbor)
            ):
                _set_packed_bit(visited, neighbor)
                queue.append(neighbor)

        # Keep the packed queue bounded by the active frontier.  Processed
        # indices are represented by ``visited`` and need not remain as Python
        # objects until the search finishes.
        if cursor >= 1_048_576:
            del queue[:cursor]
            cursor = 0

    return False, candidates, reached_count, visited


def _compartment_surface_bits(
    global_surface_bits: Dict[GridPoint, int],
    candidates: Set[GridPoint],
    metadata: SurfaceMetadata,
    preferred_seeds: Sequence[Tuple[GridPoint, int]] = (),
) -> Tuple[Dict[GridPoint, int], Set[GridPoint]]:
    """Select the closed oriented surface ring that encloses candidates.

    A vehicle's global tree contains both sides of Definition buoyancy faces
    plus many small closed interfaces between neighboring components.  Native
    code seeds the ring adjacent to the compartment sample.  The portable
    geometric flood already identifies that sample's region, so equivalent
    candidate rings are ranked by how many cells pass the native volume scan.
    """

    if not candidates:
        return {}, set()
    minimum = tuple(min(point[axis] for point in candidates) for axis in range(3))
    maximum = tuple(max(point[axis] for point in candidates) for axis in range(3))
    fallback_seeds = sorted(
        (position, surface_type)
        for position, bits in global_surface_bits.items()
        if all(
            minimum[axis] - 1 <= position[axis] <= maximum[axis] + 1
            for axis in range(3)
        )
        for surface_type in range(58)
        if bits & (1 << surface_type)
    )
    tried = set()
    best_bits: Dict[GridPoint, int] = {}
    best_accepted: Set[GridPoint] = set()
    best_score = (-1, -1)

    def try_seeds(seeds: Iterable[Tuple[GridPoint, int]]) -> None:
        nonlocal best_bits, best_accepted, best_score
        for seed in seeds:
            if seed in tried:
                continue
            crawl = crawl_compartment_surface_nodes(
                global_surface_bits, seed, metadata
            )
            tried.update(crawl.nodes or (seed,))
            if not crawl.completed:
                continue
            local_bits: Dict[GridPoint, int] = {}
            for position, surface_type in crawl.nodes:
                local_bits[position] = (
                    local_bits.get(position, 0) | (1 << surface_type)
                )
            accepted = _native_volume_scan(candidates, local_bits)
            score = (len(accepted), len(crawl.nodes))
            if score > best_score:
                best_score = score
                best_bits = local_bits
                best_accepted = accepted

    try_seeds(preferred_seeds)
    if best_accepted:
        return best_bits, best_accepted
    for seed in fallback_seeds:
        if seed in tried:
            continue
        crawl = crawl_compartment_surface_nodes(
            global_surface_bits, seed, metadata
        )
        tried.update(crawl.nodes or (seed,))
        if not crawl.completed:
            continue
        local_bits: Dict[GridPoint, int] = {}
        for position, surface_type in crawl.nodes:
            local_bits[position] = (
                local_bits.get(position, 0) | (1 << surface_type)
            )
        accepted = _native_volume_scan(candidates, local_bits)
        score = (len(accepted), len(crawl.nodes))
        if score > best_score:
            best_score = score
            best_bits = local_bits
            best_accepted = accepted
    return best_bits, best_accepted


def _flooder_surface_seeds(
    component,
    definition: ComponentDefinition,
    metadata: SurfaceMetadata,
    sample: GridPoint,
) -> Tuple[Tuple[GridPoint, int], ...]:
    """Return the Flooder's own faces pointing at its compartment sample."""

    seeds = []
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
            primary = (position, surface_type)
            flipped = (
                add_points(position, native_type.opposite_position_delta),
                native_type.opposite_surface_type,
            )
            for node in (primary, flipped):
                node_position, node_surface_type = node
                if (
                    add_points(
                        node_position,
                        metadata.types[node_surface_type].direction,
                    )
                    == sample
                ):
                    seeds.append(node)
    return tuple(sorted(set(seeds)))


def _apply_native_full_volume_threshold(
    vehicle: Vehicle,
    catalog: DefinitionCatalog,
    body_index: int,
    metadata: SurfaceMetadata,
    geometric_fill_positions: Tuple[GridPoint, ...],
    geometric_compartment_positions: Optional[
        Tuple[Tuple[GridPoint, ...], ...]
    ] = None,
    preferred_seeds_by_group: Optional[
        Tuple[Tuple[Tuple[GridPoint, int], ...], ...]
    ] = None,
    global_surface_bits: Optional[Dict[GridPoint, int]] = None,
) -> Tuple[Tuple[GridPoint, ...], int]:
    """Mirror ``c_sealed_volume_compartment::calculate_volume``.

    The native routine scans each X/Y column along Z. Four build-pinned surface
    type masks update the inside state on the previous, current, and next
    surface-tree cells. A candidate receives two half-volume contributions and
    passes the native ``> 0.99`` test only when both halves are inside. This is
    why some occupied sloped cells are filled while visually similar cells are
    excluded; testing only whether a boundary surface points inward is not
    sufficient.
    """

    candidates = set(geometric_fill_positions)
    if not candidates:
        return geometric_fill_positions, 0

    if global_surface_bits is None:
        global_surface_bits = build_body_surface_bits(
            vehicle, catalog, body_index, metadata
        )
    compartment_groups = geometric_compartment_positions or (
        tuple(sorted(candidates)),
    )
    accepted: Set[GridPoint] = set()
    preferred_groups = preferred_seeds_by_group or tuple(
        () for _group in compartment_groups
    )
    for group_index, group in enumerate(compartment_groups):
        _local_bits, group_accepted = _compartment_surface_bits(
            global_surface_bits,
            set(group),
            metadata,
            preferred_groups[group_index]
            if group_index < len(preferred_groups)
            else (),
        )
        accepted.update(group_accepted)
    return tuple(sorted(accepted)), len(candidates - accepted)


def model_surface_physics_flood_fill(
    vehicle: Vehicle,
    catalog: DefinitionCatalog,
    body_index: int,
    metadata: Optional[SurfaceMetadata] = None,
    max_scan_microcells: int = 2_000_000,
    progress_callback: Optional[Callable[[float, str], None]] = None,
    static_voxels: Optional[Sequence[WorldVoxel]] = None,
    component_definitions: Optional[Sequence[ComponentDefinition]] = None,
) -> SurfaceFloodFillResult:
    """Return portable fill calls and the post-fill physics voxel sequence.

    ``static_voxels`` and ``component_definitions`` are optional prepared
    inputs for callers that already expanded the body.  Keeping them optional
    preserves the public API while avoiding a second full voxel expansion in
    the application analysis path.
    """

    def report(fraction: float, message: str) -> None:
        if progress_callback is not None:
            progress_callback(max(0.0, min(1.0, fraction)), message)

    report(0.0, "Physics Flooder入力を準備中…")
    native = metadata or SurfaceMetadata()
    body = vehicle.bodies[body_index]
    resolved_static_voxels = (
        tuple(static_voxels)
        if static_voxels is not None
        else vehicle.physics_voxels(catalog, body_index)
    )
    resolved_component_definitions = (
        tuple(component_definitions)
        if component_definitions is not None
        else tuple(
            catalog.load(component.definition_id) for component in body.components
        )
    )
    if len(resolved_component_definitions) != len(body.components):
        raise ValueError(
            "component_definitions must match the body's component count"
        )
    flooders = tuple(
        (component, definition)
        for component, definition in zip(
            body.components, resolved_component_definitions
        )
        if definition.water_component_type == 19
    )
    metadata_missing_count = 0
    metadata_missing_component_indices: Tuple[int, ...] = ()
    native_ignored_count = 0

    def result(
        *,
        supported: bool,
        status: str,
        static_after: Tuple[WorldVoxel, ...] = resolved_static_voxels,
        new_fill: Tuple[WorldVoxel, ...] = (),
        fill_positions: Tuple[GridPoint, ...] = (),
        compartment_count: int = 0,
        open_count: int = 0,
        scan_count: int = 0,
        polygon_count: int = 0,
        triangle_count: int = 0,
        blocked_count: int = 0,
        partial_excluded_count: int = 0,
    ) -> SurfaceFloodFillResult:
        return SurfaceFloodFillResult(
            supported=supported,
            status=status,
            static_voxels=resolved_static_voxels,
            static_voxels_after_fill=static_after,
            new_fill_voxels=new_fill,
            fill_call_positions=fill_positions,
            sealed_compartment_count=compartment_count,
            open_flooder_count=open_count,
            scan_microcell_count=scan_count,
            surface_polygon_count=polygon_count,
            surface_triangle_count=triangle_count,
            blocked_microedge_count=blocked_count,
            partial_volume_excluded_count=partial_excluded_count,
            metadata_missing_surface_count=metadata_missing_count,
            metadata_missing_component_indices=(
                metadata_missing_component_indices
            ),
            native_ignored_surface_count=native_ignored_count,
            stormworks_build_id=native.stormworks_build_id,
            binary_sha256=native.binary_sha256,
        )

    if not flooders:
        report(1.0, "Physics Flooderなし")
        return result(supported=True, status="no_physics_flooder")

    report(0.08, "buoyancy Surfaceを解決中…")
    surface_analysis = _build_body_surface_analysis(
        vehicle,
        catalog,
        body_index,
        native,
        resolved_component_definitions,
    )
    metadata_missing_count = surface_analysis.metadata_missing_count
    metadata_missing_component_indices = (
        surface_analysis.metadata_missing_component_indices
    )
    native_ignored_count = surface_analysis.native_ignored_count
    if metadata_missing_count:
        report(1.0, "未知のbuoyancy Surfaceを検出")
        return result(
            supported=False,
            status="unsupported_surface_metadata_missing",
            open_count=len(flooders),
        )

    global_surface_bits = surface_analysis.bits
    vertex_bounds = surface_analysis.vertex_bounds
    if vertex_bounds is None:
        report(1.0, "密閉Surfaceなし")
        return result(
            supported=True,
            status="surface_volume_open",
            open_count=len(flooders),
        )
    minimum_vertex, maximum_vertex = vertex_bounds
    axes = tuple(
        _lattice_axis(
            minimum_vertex[axis],
            maximum_vertex[axis],
        )
        for axis in range(3)
    )
    scan_count = len(axes[0]) * len(axes[1]) * len(axes[2])
    # Kept as an API compatibility argument.  The old implementation rejected
    # bodies above this theoretical count because it allocated every point as
    # Python tuples.  Packed edges and sample-directed traversal no longer need
    # that artificial limit.
    _ = max_scan_microcells
    report(0.34, "native Surface境界を圧縮中…")
    lattice = _compact_surface_lattice(
        global_surface_bits, native, axes
    )

    report(0.68, "Flooderサンプル区画を探索中…")
    compartment_groups: List[Set[GridPoint]] = []
    preferred_seed_groups: List[Tuple[Tuple[GridPoint, int], ...]] = []
    open_count = 0
    for flooder_index, (component, definition) in enumerate(flooders):
        report(
            0.68 + 0.16 * flooder_index / max(1, len(flooders)),
            "Flooderサンプル区画 {}/{}を探索中…".format(
                flooder_index + 1, len(flooders)
            ),
        )
        sample = add_points(
            component.position,
            apply_matrix(
                component.effective_transform,
                definition.compartment_sample_position,
            ),
        )
        sample_center = tuple(4 * value for value in sample)
        sample_nodes = tuple(
            (
                sample_center[0] + x,
                sample_center[1] + y,
                sample_center[2] + z,
            )
            for x in (-1, 1)
            for y in (-1, 1)
            for z in (-1, 1)
        )
        processed_samples: Set[Point3] = set()
        found_sealed_region = False
        preferred_seeds = _flooder_surface_seeds(
            component, definition, native, sample
        )
        for sample_node in sample_nodes:
            if sample_node in processed_samples:
                continue
            is_open, candidates, _visited_count, visited = (
                _compact_region_from_seed(lattice, sample_node)
            )
            for other_sample in sample_nodes:
                offsets = tuple(
                    other_sample[axis] - lattice.minimum[axis]
                    for axis in range(3)
                )
                if any(
                    offset < 0
                    or offset % 2
                    or offset // 2 >= lattice.dimensions[axis]
                    for axis, offset in enumerate(offsets)
                ):
                    continue
                ox, oy, oz = (offset // 2 for offset in offsets)
                other_index = (
                    (ox * lattice.dimensions[1] + oy)
                    * lattice.dimensions[2]
                    + oz
                )
                if _packed_bit(visited, other_index):
                    processed_samples.add(other_sample)
            if is_open or not candidates:
                continue
            found_sealed_region = True
            existing_index = next(
                (
                    index
                    for index, existing in enumerate(compartment_groups)
                    if existing == candidates
                ),
                None,
            )
            if existing_index is None:
                compartment_groups.append(candidates)
                preferred_seed_groups.append(preferred_seeds)
            else:
                preferred_seed_groups[existing_index] = tuple(
                    sorted(
                        set(preferred_seed_groups[existing_index])
                        | set(preferred_seeds)
                    )
                )
        if not found_sealed_region:
            open_count += 1

    geometric_compartment_positions = tuple(
        tuple(sorted(compartment)) for compartment in compartment_groups
    )
    geometric_fill_positions = tuple(
        sorted(
            {
                position
                for compartment in geometric_compartment_positions
                for position in compartment
            }
        )
    )
    report(0.86, "native edge crawlと体積走査を実行中…")
    fill_positions, partial_excluded_count = _apply_native_full_volume_threshold(
        vehicle,
        catalog,
        body_index,
        native,
        geometric_fill_positions,
        geometric_compartment_positions,
        tuple(preferred_seed_groups),
        global_surface_bits,
    )

    latest_by_position: Dict[GridPoint, int] = {}
    for index, voxel in enumerate(resolved_static_voxels):
        latest_by_position[voxel.position] = index
    static_after_list = list(resolved_static_voxels)
    for position in fill_positions:
        existing_index = latest_by_position.get(position)
        if existing_index is not None:
            static_after_list[existing_index] = replace(
                static_after_list[existing_index],
                physics_shape=0,
                physics_rotation=IDENTITY_MATRIX,
            )

    first_flooder_index = flooders[0][0].index
    new_positions = tuple(
        position for position in fill_positions if position not in latest_by_position
    )
    new_fill = tuple(
        WorldVoxel(
            body_index=body.index,
            body_id=body.body_id,
            component_index=first_flooder_index,
            component_definition="__physics_flood_fill__",
            definition_voxel_index=-(index + 1),
            insertion_index=len(resolved_static_voxels) + index,
            position=position,
            physics_shape=0,
            physics_rotation=IDENTITY_MATRIX,
        )
        for index, position in enumerate(new_positions)
    )
    status = "surface_volume_filled" if fill_positions else "surface_volume_open"
    report(1.0, "Physics Flooder解析完了")
    return result(
        supported=True,
        status=status,
        static_after=tuple(static_after_list),
        new_fill=new_fill,
        fill_positions=fill_positions,
        compartment_count=len(compartment_groups),
        open_count=open_count,
        scan_count=scan_count,
        polygon_count=lattice.polygon_count,
        triangle_count=lattice.triangle_count,
        blocked_count=lattice.blocked_edge_count,
        partial_excluded_count=partial_excluded_count,
    )
