//! Native score-only Stormworks physics-shape merger.
//!
//! The public boundary is deliberately a small C ABI so the desktop app can
//! load this library with Python's standard `ctypes` module.  The Python
//! implementation remains the preview/reference path and the fallback when a
//! compatible native library is unavailable.

use std::ffi::c_void;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::slice;

const ABI_VERSION: u32 = 3;
const MAX_SAMPLES: usize = 12;
const MAX_SAMPLE_COORDS: usize = MAX_SAMPLES * 3;
const NO_PLANE: u32 = u32::MAX;
const FINALIZATION_MARGIN: f32 = 0.041999999433755875;
const FINALIZATION_PARALLEL_LIMIT: f64 = 0.9900000095367432;
const FINALIZATION_EPSILON: f64 = 9.999999747378752e-06;

const OK: i32 = 0;
const ERR_NULL: i32 = 1;
const ERR_LAYOUT: i32 = 2;
const ERR_OVERLAP: i32 = 3;
const ERR_ORDER: i32 = 4;
const ERR_PANIC: i32 = 5;
const ERR_COUNT_OVERFLOW: i32 = 6;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct Point {
    x: i32,
    y: i32,
    z: i32,
}

impl Point {
    fn from_slice(values: &[i32]) -> Self {
        Self {
            x: values[0],
            y: values[1],
            z: values[2],
        }
    }

    fn axis(self, axis: usize) -> i32 {
        match axis {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }

    fn with_axis(self, axis: usize, value: i32) -> Self {
        match axis {
            0 => Self { x: value, ..self },
            1 => Self { y: value, ..self },
            _ => Self { z: value, ..self },
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Plane {
    anchor: Point,
    normal: Point,
}

#[derive(Clone, Copy)]
struct FinalizationPlane {
    normal: [f64; 3],
    distance: f64,
}

#[derive(Clone)]
struct Voxel {
    position: Point,
    plane_index: u32,
    physics_shape: u8,
}

#[derive(Clone)]
struct PlaneVoxel {
    plane: Plane,
    sample_pattern: u32,
}

#[derive(Clone, Eq, Hash, PartialEq)]
struct SamplePattern {
    sample_count: u8,
    sample_offsets: [i32; MAX_SAMPLE_COORDS],
    collision_threshold: u8,
}

#[derive(Clone)]
struct PositionMap {
    slots: Vec<usize>,
    mask: usize,
}

impl PositionMap {
    const EMPTY: usize = usize::MAX;

    fn new(item_count: usize) -> Result<Self, i32> {
        let minimum = item_count.checked_mul(2).ok_or(ERR_LAYOUT)?.max(1);
        let slot_count = minimum.checked_next_power_of_two().ok_or(ERR_LAYOUT)?;
        Ok(Self {
            slots: vec![Self::EMPTY; slot_count],
            mask: slot_count - 1,
        })
    }

    fn hash(position: Point) -> usize {
        let mut value = u64::from(position.x as u32)
            .wrapping_mul(0x9e37_79b1_85eb_ca87);
        value ^= u64::from(position.y as u32)
            .wrapping_mul(0xc2b2_ae3d_27d4_eb4f)
            .rotate_left(21);
        value ^= u64::from(position.z as u32)
            .wrapping_mul(0x1656_67b1_9e37_79f9)
            .rotate_left(42);
        value ^= value >> 30;
        value = value.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value ^= value >> 27;
        value = value.wrapping_mul(0x94d0_49bb_1331_11eb);
        (value ^ (value >> 31)) as usize
    }

    fn get(&self, position: Point, voxels: &[Voxel]) -> Option<usize> {
        let mut slot = Self::hash(position) & self.mask;
        loop {
            let voxel_index = self.slots[slot];
            if voxel_index == Self::EMPTY {
                return None;
            }
            if voxels[voxel_index].position == position {
                return Some(voxel_index);
            }
            slot = (slot + 1) & self.mask;
        }
    }

    fn insert(
        &mut self,
        position: Point,
        voxel_index: usize,
        voxels: &[Voxel],
    ) -> Option<usize> {
        let mut slot = Self::hash(position) & self.mask;
        loop {
            let previous = self.slots[slot];
            if previous == Self::EMPTY {
                self.slots[slot] = voxel_index;
                return None;
            }
            if voxels[previous].position == position {
                self.slots[slot] = voxel_index;
                return Some(previous);
            }
            slot = (slot + 1) & self.mask;
        }
    }
}

struct Evaluator {
    voxels: Vec<Voxel>,
    plane_voxels: Vec<PlaneVoxel>,
    sample_patterns: Vec<SamplePattern>,
    component_offsets: Vec<usize>,
    trailing_start: usize,
    base_lookup: PositionMap,
    has_overlaps: bool,
    ordered_indices: Vec<usize>,
    processed: Vec<u8>,
    overlap_lookup: PositionMap,
}

enum Expansion {
    Expanded,
    RetryAfterPerpendicularExpansion,
    Blocked,
}

fn set_error(out_error: *mut i32, value: i32) {
    if !out_error.is_null() {
        unsafe { *out_error = value };
    }
}

fn checked_slice<'a, T>(pointer: *const T, length: usize) -> Result<&'a [T], i32> {
    if length == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() {
        return Err(ERR_NULL);
    }
    Ok(unsafe { slice::from_raw_parts(pointer, length) })
}

fn finalization_dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn finalization_determinant(rows: [[f64; 3]; 3]) -> f64 {
    rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
}

fn finalization_intersection(
    first: FinalizationPlane,
    second: FinalizationPlane,
    third: FinalizationPlane,
) -> Option<[f64; 3]> {
    let normals = [first.normal, second.normal, third.normal];
    for (left, right) in [(0usize, 1usize), (0, 2), (1, 2)] {
        if finalization_dot(normals[left], normals[right]).abs()
            >= FINALIZATION_PARALLEL_LIMIT
        {
            return None;
        }
    }
    let determinant = finalization_determinant(normals);
    if determinant == 0.0 {
        return None;
    }
    let distances = [first.distance, second.distance, third.distance];
    let mut result = [0.0; 3];
    for column in 0..3 {
        let mut rows = normals;
        for row in 0..3 {
            rows[row][column] = distances[row];
        }
        result[column] = finalization_determinant(rows) / determinant;
    }
    Some(result)
}

fn finalization_normal(normal: Point) -> [f32; 3] {
    let values = [normal.x as f32, normal.y as f32, normal.z as f32];
    let xy_squared = values[0] * values[0] + values[1] * values[1];
    let length = (xy_squared + values[2] * values[2]).sqrt();
    if length <= 0.0 {
        [1.0, 0.0, 0.0]
    } else {
        [
            values[0] / length,
            values[1] / length,
            values[2] / length,
        ]
    }
}

fn zero_custom_plane_margin(physics_shape: u8) -> bool {
    matches!(
        physics_shape,
        6 | 7
            | 8
            | 9
            | 12
            | 13
            | 14
            | 15
            | 18
            | 19
            | 20
            | 21
            | 22
            | 23
            | 24
            | 25
            | 28
            | 29
            | 30
            | 31
    )
}

fn finalization_planes(
    minimum: Point,
    maximum: Point,
    planes: &[Plane],
    seed_physics_shape: u8,
) -> Vec<FinalizationPlane> {
    let minimum_world = [
        minimum.x as f32 * 0.25 - 0.125,
        minimum.y as f32 * 0.25 - 0.125,
        minimum.z as f32 * 0.25 - 0.125,
    ];
    let maximum_world = [
        maximum.x as f32 * 0.25 + 0.125,
        maximum.y as f32 * 0.25 + 0.125,
        maximum.z as f32 * 0.25 + 0.125,
    ];
    let center = [
        (minimum_world[0] + maximum_world[0]) * 0.5,
        (minimum_world[1] + maximum_world[1]) * 0.5,
        (minimum_world[2] + maximum_world[2]) * 0.5,
    ];
    let mut result = Vec::with_capacity(6 + planes.len());
    for axis in 0..3 {
        let mut negative = [0.0; 3];
        let mut positive = [0.0; 3];
        negative[axis] = -1.0;
        positive[axis] = 1.0;
        result.push(FinalizationPlane {
            normal: negative,
            distance: f64::from(
                center[axis] - minimum_world[axis] - FINALIZATION_MARGIN,
            ),
        });
        result.push(FinalizationPlane {
            normal: positive,
            distance: f64::from(
                maximum_world[axis] - center[axis] - FINALIZATION_MARGIN,
            ),
        });
    }

    let custom_margin_scale = if zero_custom_plane_margin(seed_physics_shape) {
        0.0f32
    } else {
        1.0f32
    };
    for plane in planes {
        let normal_f32 = finalization_normal(plane.normal);
        let normal = [
            f64::from(normal_f32[0]),
            f64::from(normal_f32[1]),
            f64::from(normal_f32[2]),
        ];
        let anchors = [plane.anchor.x, plane.anchor.y, plane.anchor.z];
        let mut point_f32 = [0.0f32; 3];
        for axis in 0..3 {
            let coordinate = ((anchors[axis] as f32 - 0.5) * 0.25) - center[axis];
            let margin =
                normal_f32[axis] * FINALIZATION_MARGIN * custom_margin_scale;
            point_f32[axis] = coordinate - margin;
        }
        let point = [
            f64::from(point_f32[0]),
            f64::from(point_f32[1]),
            f64::from(point_f32[2]),
        ];
        result.push(FinalizationPlane {
            normal,
            distance: finalization_dot(normal, point),
        });
    }
    result
}

fn finalization_vertex_count(
    minimum: Point,
    maximum: Point,
    planes: &[Plane],
    seed_physics_shape: u8,
) -> usize {
    if planes.is_empty() {
        return 8;
    }
    let final_planes =
        finalization_planes(minimum, maximum, planes, seed_physics_shape);
    let mut vertices = Vec::new();
    for first in 0..final_planes.len() {
        for second in first + 1..final_planes.len() {
            for third in second + 1..final_planes.len() {
                if let Some(point) = finalization_intersection(
                    final_planes[first],
                    final_planes[second],
                    final_planes[third],
                ) {
                    vertices.push(point);
                }
            }
        }
    }

    let mut index = 0usize;
    while index < vertices.len() {
        let point = vertices[index];
        if final_planes.iter().any(|plane| {
            finalization_dot(plane.normal, point)
                > plane.distance + FINALIZATION_EPSILON
        }) {
            vertices.swap_remove(index);
        } else {
            index += 1;
        }
    }

    let mut first = 0usize;
    while first < vertices.len() {
        let mut second = first + 1;
        while second < vertices.len() {
            let distance_squared = (vertices[first][0] - vertices[second][0]).powi(2)
                + (vertices[first][1] - vertices[second][1]).powi(2)
                + (vertices[first][2] - vertices[second][2]).powi(2);
            if distance_squared < FINALIZATION_EPSILON {
                vertices.swap_remove(second);
            } else {
                second += 1;
            }
        }
        first += 1;
    }
    vertices.len()
}

fn planes_coplanar(left: Plane, right: Plane) -> bool {
    if left.normal != right.normal {
        return false;
    }
    let dx = i64::from(left.anchor.x) - i64::from(right.anchor.x);
    let dy = i64::from(left.anchor.y) - i64::from(right.anchor.y);
    let dz = i64::from(left.anchor.z) - i64::from(right.anchor.z);
    dx * i64::from(left.normal.x)
        + dy * i64::from(left.normal.y)
        + dz * i64::from(left.normal.z)
        == 0
}

fn voxel_collide_plane(position: Point, plane: Plane) -> i32 {
    let nx = i64::from(plane.normal.x);
    let ny = i64::from(plane.normal.y);
    let nz = i64::from(plane.normal.z);
    let corner_distance =
        (i64::from(position.x) - i64::from(plane.anchor.x)) * nx
            + (i64::from(position.y) - i64::from(plane.anchor.y)) * ny
            + (i64::from(position.z) - i64::from(plane.anchor.z)) * nz;
    let minimum_distance = corner_distance + nx.min(0) + ny.min(0) + nz.min(0);
    let maximum_distance = corner_distance + nx.max(0) + ny.max(0) + nz.max(0);
    if minimum_distance >= 0 {
        1
    } else if maximum_distance <= 0 {
        -1
    } else {
        0
    }
}

fn voxel_collide_planes(position: Point, planes: &[Plane]) -> i32 {
    let mut collision = -1;
    for &plane in planes {
        let value = voxel_collide_plane(position, plane);
        if value > 0 {
            return 1;
        }
        if value == 0 {
            collision = 0;
        }
    }
    collision
}

fn physics_voxel_collide_plane(
    voxel: &Voxel,
    plane_voxels: &[PlaneVoxel],
    sample_patterns: &[SamplePattern],
    plane: Plane,
) -> i32 {
    let nx = i64::from(plane.normal.x);
    let ny = i64::from(plane.normal.y);
    let nz = i64::from(plane.normal.z);
    let corner_distance =
        (i64::from(voxel.position.x) - i64::from(plane.anchor.x)) * nx
            + (i64::from(voxel.position.y) - i64::from(plane.anchor.y)) * ny
            + (i64::from(voxel.position.z) - i64::from(plane.anchor.z)) * nz;
    let minimum_distance = corner_distance + nx.min(0) + ny.min(0) + nz.min(0);
    let maximum_distance = corner_distance + nx.max(0) + ny.max(0) + nz.max(0);
    if minimum_distance >= 0 {
        return 1;
    }
    if maximum_distance < 0 {
        return -1;
    }
    if maximum_distance == 0 {
        return if voxel.plane_index != NO_PLANE { -1 } else { 0 };
    }
    if voxel.plane_index == NO_PLANE {
        return 1;
    }

    let plane_voxel = &plane_voxels[voxel.plane_index as usize];
    let native_plane = plane_voxel.plane;
    if native_plane.normal == plane.normal {
        let offset =
            (i64::from(native_plane.anchor.x) - i64::from(plane.anchor.x)) * nx
                + (i64::from(native_plane.anchor.y) - i64::from(plane.anchor.y)) * ny
                + (i64::from(native_plane.anchor.z) - i64::from(plane.anchor.z)) * nz;
        return if offset < 0 {
            1
        } else if offset > 0 {
            -1
        } else {
            0
        };
    }

    let normal_squared = i128::from(nx) * i128::from(nx)
        + i128::from(ny) * i128::from(ny)
        + i128::from(nz) * i128::from(nz);
    if normal_squared <= 0 {
        return -1;
    }
    let pattern = &sample_patterns[plane_voxel.sample_pattern as usize];
    let mut positive = 0usize;
    let mut negative = 0usize;
    let mut near = 0usize;
    for sample_index in 0..usize::from(pattern.sample_count) {
        let base = sample_index * 3;
        // Python's reference point is position + 0.5 + quarter_offset / 4.
        // Keeping the numerator integral avoids coordinate-rounding drift;
        // the final native 0.01 tolerance comparison is integral too.
        let dx4 = 4 * (i64::from(voxel.position.x) - i64::from(plane.anchor.x))
            + 2
            + i64::from(pattern.sample_offsets[base]);
        let dy4 = 4 * (i64::from(voxel.position.y) - i64::from(plane.anchor.y))
            + 2
            + i64::from(pattern.sample_offsets[base + 1]);
        let dz4 = 4 * (i64::from(voxel.position.z) - i64::from(plane.anchor.z))
            + 2
            + i64::from(pattern.sample_offsets[base + 2]);
        let distance_numerator = dx4 * nx + dy4 * ny + dz4 * nz;
        // abs(D / (4 * |normal|)) > 0.01 is equivalent to
        // 625 * D^2 > |normal|^2.  This exact integer comparison preserves
        // the reference tolerance without CPU/OS floating-point drift.
        let outside_near_band = 625i128
            * i128::from(distance_numerator)
            * i128::from(distance_numerator)
            > normal_squared;
        if distance_numerator > 0 && outside_near_band {
            positive += 1;
        } else if distance_numerator < 0 && outside_near_band {
            negative += 1;
        } else {
            near += 1;
        }
    }
    if positive > 0 {
        return 1;
    }
    let count = usize::from(pattern.sample_count);
    let threshold = usize::from(pattern.collision_threshold);
    if negative > count.saturating_sub(threshold) {
        return -1;
    }
    if near == threshold { 0 } else { 1 }
}

fn check_layer_position(
    position: Point,
    minimum: Point,
    maximum: Point,
    planes: &[Plane],
    voxels: &[Voxel],
    plane_voxels: &[PlaneVoxel],
    sample_patterns: &[SamplePattern],
    by_position: &PositionMap,
    processed: &[u8],
    additions: &mut Vec<usize>,
    temporary_planes: &mut Vec<Plane>,
) -> bool {
    let shape_collision = if planes.is_empty() {
        -1
    } else {
        voxel_collide_planes(position, planes)
    };
    if shape_collision > 0 {
        return true;
    }
    let Some(index) = by_position.get(position, voxels) else {
        return false;
    };
    if processed[index] != 0 {
        return false;
    }
    let voxel = &voxels[index];
    if voxel.plane_index == NO_PLANE {
        if shape_collision >= 0 {
            return false;
        }
        additions.push(index);
        return true;
    }

    let own_plane = plane_voxels[voxel.plane_index as usize].plane;
    if planes.iter().any(|&plane| planes_coplanar(own_plane, plane)) {
        additions.push(index);
        return true;
    }

    let mut collided_positive = false;
    let mut collided_boundary = false;
    for &plane in planes {
        if voxel_collide_plane(position, plane) != 0 {
            continue;
        }
        match physics_voxel_collide_plane(
            voxel,
            plane_voxels,
            sample_patterns,
            plane,
        ) {
            0 => collided_boundary = true,
            value if value > 0 => collided_positive = true,
            _ => {}
        }
    }
    if collided_positive {
        return false;
    }

    if !collided_boundary {
        for x in minimum.x..=maximum.x {
            for y in minimum.y..=maximum.y {
                for z in minimum.z..=maximum.z {
                    let old_position = Point { x, y, z };
                    if voxel_collide_plane(old_position, own_plane) < 0 {
                        continue;
                    }
                    let Some(old_index) = by_position.get(old_position, voxels) else {
                        continue;
                    };
                    let old_voxel = &voxels[old_index];
                    if old_voxel.plane_index == NO_PLANE {
                        return false;
                    }
                    if physics_voxel_collide_plane(
                        old_voxel,
                        plane_voxels,
                        sample_patterns,
                        own_plane,
                    ) > 0
                    {
                        return false;
                    }
                }
            }
        }
        temporary_planes.push(own_plane);
    }
    additions.push(index);
    true
}

fn try_expand(
    minimum: &mut Point,
    maximum: &mut Point,
    planes: &mut Vec<Plane>,
    direction_index: usize,
    voxels: &[Voxel],
    plane_voxels: &[PlaneVoxel],
    sample_patterns: &[SamplePattern],
    by_position: &PositionMap,
    processed: &mut [u8],
    additions: &mut Vec<usize>,
    temporary_planes: &mut Vec<Plane>,
) -> Expansion {
    additions.clear();
    temporary_planes.clear();
    let axis = direction_index % 3;
    let negative = direction_index >= 3;
    let edge = if negative {
        minimum.axis(axis).checked_sub(1)
    } else {
        maximum.axis(axis).checked_add(1)
    };
    let Some(edge) = edge else {
        return Expansion::Blocked;
    };

    let mut accepted = true;
    match axis {
        0 => {
            'layer: for y in minimum.y..=maximum.y {
                for z in minimum.z..=maximum.z {
                    if !check_layer_position(
                        Point { x: edge, y, z },
                        *minimum,
                        *maximum,
                        planes,
                        voxels,
                        plane_voxels,
                        sample_patterns,
                        by_position,
                        processed,
                        additions,
                        temporary_planes,
                    ) {
                        accepted = false;
                        break 'layer;
                    }
                }
            }
        }
        1 => {
            'layer: for x in minimum.x..=maximum.x {
                for z in minimum.z..=maximum.z {
                    if !check_layer_position(
                        Point { x, y: edge, z },
                        *minimum,
                        *maximum,
                        planes,
                        voxels,
                        plane_voxels,
                        sample_patterns,
                        by_position,
                        processed,
                        additions,
                        temporary_planes,
                    ) {
                        accepted = false;
                        break 'layer;
                    }
                }
            }
        }
        _ => {
            'layer: for x in minimum.x..=maximum.x {
                for y in minimum.y..=maximum.y {
                    if !check_layer_position(
                        Point { x, y, z: edge },
                        *minimum,
                        *maximum,
                        planes,
                        voxels,
                        plane_voxels,
                        sample_patterns,
                        by_position,
                        processed,
                        additions,
                        temporary_planes,
                    ) {
                        accepted = false;
                        break 'layer;
                    }
                }
            }
        }
    }
    if !accepted {
        return Expansion::Blocked;
    }
    if additions.is_empty() {
        return Expansion::RetryAfterPerpendicularExpansion;
    }

    // Native second pass uses temporary planes before the group's existing
    // planes.  Only a newly proposed plane triggers rejection; existing planes
    // may still provide the shape-intersection evidence.  Preserve the full
    // order because duplicate samples and thresholds are build-pinned.
    for &index in additions.iter() {
        let voxel = &voxels[index];
        let cell_intersects = temporary_planes
            .iter()
            .any(|&plane| voxel_collide_plane(voxel.position, plane) == 0);
        let mut shape_intersects = false;
        for &plane in temporary_planes.iter().chain(planes.iter()) {
            if voxel_collide_plane(voxel.position, plane) != 0 {
                continue;
            }
            if voxel.plane_index != NO_PLANE
                && physics_voxel_collide_plane(
                    voxel,
                    plane_voxels,
                    sample_patterns,
                    plane,
                ) == 0
            {
                shape_intersects = true;
            }
        }
        if cell_intersects && !shape_intersects {
            return Expansion::Blocked;
        }
    }

    for &plane in temporary_planes.iter() {
        if !planes.iter().any(|&item| planes_coplanar(plane, item)) {
            planes.push(plane);
        }
    }
    if negative {
        *minimum = minimum.with_axis(axis, edge);
    } else {
        *maximum = maximum.with_axis(axis, edge);
    }
    for &index in additions.iter() {
        processed[index] = 1;
    }
    Expansion::Expanded
}

impl Evaluator {
    fn score(&mut self, order: &[u32]) -> Result<u32, i32> {
        let component_count = self.component_offsets.len().saturating_sub(1);
        if order.len() != component_count {
            return Err(ERR_ORDER);
        }
        let mut seen = vec![0u8; component_count];
        self.ordered_indices.clear();
        for &raw_component_index in order {
            let component_index = raw_component_index as usize;
            if component_index >= component_count || seen[component_index] != 0 {
                return Err(ERR_ORDER);
            }
            seen[component_index] = 1;
            self.ordered_indices.extend(
                self.component_offsets[component_index]
                    ..self.component_offsets[component_index + 1],
            );
        }
        self.ordered_indices
            .extend(self.trailing_start..self.voxels.len());

        if self.has_overlaps {
            self.overlap_lookup.clone_from(&self.base_lookup);
            for &index in &self.ordered_indices {
                self.overlap_lookup.insert(
                    self.voxels[index].position,
                    index,
                    &self.voxels,
                );
            }
        }
        let by_position = if self.has_overlaps {
            &self.overlap_lookup
        } else {
            &self.base_lookup
        };

        self.processed.fill(0);
        let mut shape_count = 0usize;
        let mut planes = Vec::with_capacity(8);
        let mut additions = Vec::new();
        let mut temporary_planes = Vec::with_capacity(8);
        for &seed_index in &self.ordered_indices {
            if self.processed[seed_index] != 0 {
                continue;
            }
            self.processed[seed_index] = 1;
            let seed = &self.voxels[seed_index];
            let mut minimum = seed.position;
            let mut maximum = seed.position;
            planes.clear();
            if seed.plane_index != NO_PLANE {
                planes.push(self.plane_voxels[seed.plane_index as usize].plane);
            }
            // The source vector retains duplicate voxels, but the game's
            // octree lookup retains the latest write at each position.  An
            // older duplicate seed consumes that lookup winner only when the
            // position is inside or crossing its initialized hull; only the
            // seed contributes the group's initial clip plane.
            if self.has_overlaps
                && voxel_collide_planes(seed.position, &planes) <= 0
            {
                if let Some(overlap_winner) =
                    by_position.get(seed.position, &self.voxels)
                {
                    if overlap_winner != seed_index
                        && self.processed[overlap_winner] == 0
                    {
                        self.processed[overlap_winner] = 1;
                    }
                }
            }
            let mut blocked = [false; 6];
            loop {
                let mut changed = false;
                for direction_index in 0..6 {
                    if blocked[direction_index] {
                        continue;
                    }
                    match try_expand(
                        &mut minimum,
                        &mut maximum,
                        &mut planes,
                        direction_index,
                        &self.voxels,
                        &self.plane_voxels,
                        &self.sample_patterns,
                        by_position,
                        &mut self.processed,
                        &mut additions,
                        &mut temporary_planes,
                    ) {
                        Expansion::Expanded => changed = true,
                        Expansion::RetryAfterPerpendicularExpansion => {}
                        Expansion::Blocked => blocked[direction_index] = true,
                    }
                }
                if !changed {
                    break;
                }
            }
            if planes.is_empty()
                || finalization_vertex_count(
                    minimum,
                    maximum,
                    &planes,
                    seed.physics_shape,
                ) >= 4
            {
                shape_count += 1;
            }
        }
        u32::try_from(shape_count).map_err(|_| ERR_COUNT_OVERFLOW)
    }
}

#[no_mangle]
pub extern "C" fn swp_native_abi_version() -> u32 {
    ABI_VERSION
}

#[allow(clippy::too_many_arguments)]
#[no_mangle]
pub extern "C" fn swp_prepared_create(
    positions: *const i32,
    plane_values: *const i32,
    plane_present: *const u8,
    physics_shapes: *const u8,
    voxel_sample_patterns: *const u32,
    sample_counts: *const u8,
    sample_offsets: *const i32,
    sample_stride: usize,
    collision_thresholds: *const u8,
    sample_pattern_count: usize,
    voxel_count: usize,
    component_offsets: *const u32,
    component_count: usize,
    trailing_start: usize,
    allow_overlaps: u8,
    out_error: *mut i32,
) -> *mut c_void {
    set_error(out_error, OK);
    let result = catch_unwind(AssertUnwindSafe(|| -> Result<*mut c_void, i32> {
        if sample_stride > MAX_SAMPLES || trailing_start > voxel_count {
            return Err(ERR_LAYOUT);
        }
        let positions = checked_slice(positions, voxel_count.checked_mul(3).ok_or(ERR_LAYOUT)?)?;
        let plane_values =
            checked_slice(plane_values, voxel_count.checked_mul(6).ok_or(ERR_LAYOUT)?)?;
        let plane_present = checked_slice(plane_present, voxel_count)?;
        let physics_shapes = checked_slice(physics_shapes, voxel_count)?;
        let voxel_sample_patterns = checked_slice(voxel_sample_patterns, voxel_count)?;
        let sample_counts = checked_slice(sample_counts, sample_pattern_count)?;
        let sample_offsets = checked_slice(
            sample_offsets,
            sample_pattern_count
                .checked_mul(sample_stride)
                .and_then(|value| value.checked_mul(3))
                .ok_or(ERR_LAYOUT)?,
        )?;
        let collision_thresholds = checked_slice(collision_thresholds, sample_pattern_count)?;
        let raw_component_offsets = checked_slice(component_offsets, component_count + 1)?;
        if raw_component_offsets.first().copied().unwrap_or(0) != 0
            || raw_component_offsets.last().copied().map(usize::try_from).transpose().map_err(|_| ERR_LAYOUT)?
                != Some(trailing_start)
        {
            return Err(ERR_LAYOUT);
        }
        let mut offsets = Vec::with_capacity(component_count + 1);
        let mut previous = 0usize;
        for &raw_offset in raw_component_offsets {
            let offset = usize::try_from(raw_offset).map_err(|_| ERR_LAYOUT)?;
            if offset < previous || offset > trailing_start {
                return Err(ERR_LAYOUT);
            }
            offsets.push(offset);
            previous = offset;
        }

        let mut sample_patterns = Vec::with_capacity(sample_pattern_count);
        for pattern_index in 0..sample_pattern_count {
            let count = usize::from(sample_counts[pattern_index]);
            if count > sample_stride || count > MAX_SAMPLES {
                return Err(ERR_LAYOUT);
            }
            let mut offsets_for_voxel = [0i32; MAX_SAMPLE_COORDS];
            let source_base = pattern_index * sample_stride * 3;
            offsets_for_voxel[..count * 3]
                .copy_from_slice(&sample_offsets[source_base..source_base + count * 3]);
            sample_patterns.push(SamplePattern {
                sample_count: sample_counts[pattern_index],
                sample_offsets: offsets_for_voxel,
                collision_threshold: collision_thresholds[pattern_index],
            });
        }
        let mut voxels = Vec::with_capacity(voxel_count);
        let mut plane_voxels = Vec::new();
        for index in 0..voxel_count {
            let position = Point::from_slice(&positions[index * 3..index * 3 + 3]);
            let plane_base = index * 6;
            let plane = Plane {
                anchor: Point::from_slice(&plane_values[plane_base..plane_base + 3]),
                normal: Point::from_slice(&plane_values[plane_base + 3..plane_base + 6]),
            };
            let sample_pattern = voxel_sample_patterns[index];
            if sample_pattern as usize >= sample_pattern_count {
                return Err(ERR_LAYOUT);
            }
            let plane_index = if plane_present[index] != 0 {
                let plane_index =
                    u32::try_from(plane_voxels.len()).map_err(|_| ERR_LAYOUT)?;
                plane_voxels.push(PlaneVoxel {
                    plane,
                    sample_pattern,
                });
                plane_index
            } else {
                NO_PLANE
            };
            let voxel = Voxel {
                position,
                plane_index,
                physics_shape: physics_shapes[index],
            };
            voxels.push(voxel);
        }
        let mut base_lookup = PositionMap::new(voxel_count)?;
        let mut has_overlaps = false;
        for index in 0..voxel_count {
            if base_lookup
                .insert(voxels[index].position, index, &voxels)
                .is_some()
            {
                has_overlaps = true;
            }
        }
        if has_overlaps && allow_overlaps == 0 {
            return Err(ERR_OVERLAP);
        }
        let overlap_lookup = PositionMap::new(if has_overlaps { voxel_count } else { 0 })?;
        let evaluator = Evaluator {
            voxels,
            plane_voxels,
            sample_patterns,
            component_offsets: offsets,
            trailing_start,
            base_lookup,
            has_overlaps,
            ordered_indices: Vec::with_capacity(voxel_count),
            processed: vec![0u8; voxel_count],
            overlap_lookup,
        };
        Ok(Box::into_raw(Box::new(evaluator)).cast::<c_void>())
    }));
    match result {
        Ok(Ok(handle)) => handle,
        Ok(Err(error)) => {
            set_error(out_error, error);
            std::ptr::null_mut()
        }
        Err(_) => {
            set_error(out_error, ERR_PANIC);
            std::ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn swp_prepared_score(
    handle: *mut c_void,
    component_order: *const u32,
    order_length: usize,
    out_shape_count: *mut u32,
) -> i32 {
    if handle.is_null() || out_shape_count.is_null() {
        return ERR_NULL;
    }
    let result = catch_unwind(AssertUnwindSafe(|| -> Result<u32, i32> {
        let order = checked_slice(component_order, order_length)?;
        let evaluator = unsafe { &mut *handle.cast::<Evaluator>() };
        evaluator.score(order)
    }));
    match result {
        Ok(Ok(shape_count)) => {
            unsafe { *out_shape_count = shape_count };
            OK
        }
        Ok(Err(error)) => error,
        Err(_) => ERR_PANIC,
    }
}

#[no_mangle]
pub extern "C" fn swp_prepared_destroy(handle: *mut c_void) {
    if !handle.is_null() {
        unsafe { drop(Box::from_raw(handle.cast::<Evaluator>())) };
    }
}
