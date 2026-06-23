from __future__ import annotations

from dataclasses import dataclass

from .config import Cell, MissionConfig, TemporalNoFlyZoneConfig


@dataclass(frozen=True)
class Box3D:
    min_cell: Cell
    max_cell: Cell

    @property
    def valid(self) -> bool:
        return (
            self.min_cell[0] <= self.max_cell[0]
            and self.min_cell[1] <= self.max_cell[1]
            and self.min_cell[2] <= self.max_cell[2]
        )


def add_cell(a: Cell, b: Cell) -> Cell:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def chebyshev_distance(a: Cell, b: Cell) -> int:
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def manhattan_distance(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def make_union_box(left: Box3D | None, right: Box3D | None) -> Box3D | None:
    if left is None or not left.valid:
        return right
    if right is None or not right.valid:
        return left
    return Box3D(
        min_cell=(
            min(left.min_cell[0], right.min_cell[0]),
            min(left.min_cell[1], right.min_cell[1]),
            min(left.min_cell[2], right.min_cell[2]),
        ),
        max_cell=(
            max(left.max_cell[0], right.max_cell[0]),
            max(left.max_cell[1], right.max_cell[1]),
            max(left.max_cell[2], right.max_cell[2]),
        ),
    )


def boxes_overlap(left: Box3D | None, right: Box3D | None) -> bool:
    if left is None or right is None or not left.valid or not right.valid:
        return False
    return (
        left.min_cell[0] <= right.max_cell[0]
        and right.min_cell[0] <= left.max_cell[0]
        and left.min_cell[1] <= right.max_cell[1]
        and right.min_cell[1] <= left.max_cell[1]
        and left.min_cell[2] <= right.max_cell[2]
        and right.min_cell[2] <= left.max_cell[2]
    )


def protection_box(cell: Cell, mission: MissionConfig) -> Box3D:
    return Box3D(
        min_cell=(
            cell[0] - mission.protection_xy_radius,
            cell[1] - mission.protection_xy_radius,
            cell[2] - mission.protection_z_down,
        ),
        max_cell=(
            cell[0] + mission.protection_xy_radius,
            cell[1] + mission.protection_xy_radius,
            cell[2] + mission.protection_z_up,
        ),
    )


def downwash_box(cell: Cell, mission: MissionConfig) -> Box3D | None:
    if mission.downwash_z_below <= 0:
        return None
    return Box3D(
        min_cell=(
            cell[0] - mission.downwash_xy_radius,
            cell[1] - mission.downwash_xy_radius,
            cell[2] - mission.downwash_z_below,
        ),
        max_cell=(
            cell[0] + mission.downwash_xy_radius,
            cell[1] + mission.downwash_xy_radius,
            cell[2] - 1,
        ),
    )


def has_downwash_conflict(
    from_upper: Cell,
    to_upper: Cell,
    upper: MissionConfig,
    from_other: Cell,
    to_other: Cell,
    other: MissionConfig,
) -> bool:
    if upper.downwash_z_below <= 0:
        return False

    if to_upper[2] > to_other[2] and boxes_overlap(
        downwash_box(to_upper, upper), protection_box(to_other, other)
    ):
        return True

    upper_during_transition = max(from_upper[2], to_upper[2]) > min(
        from_other[2], to_other[2]
    )
    if not upper_during_transition:
        return False

    swept_downwash = make_union_box(
        downwash_box(from_upper, upper), downwash_box(to_upper, upper)
    )
    swept_body = make_union_box(
        protection_box(from_other, other), protection_box(to_other, other)
    )
    return boxes_overlap(swept_downwash, swept_body)


def transition_conflict(
    from_a: Cell,
    to_a: Cell,
    mission_a: MissionConfig,
    from_b: Cell,
    to_b: Cell,
    mission_b: MissionConfig,
) -> str | None:
    if boxes_overlap(protection_box(to_a, mission_a), protection_box(to_b, mission_b)):
        return "protection"

    if from_a == to_b and from_b == to_a and to_a != to_b:
        return "edge_swap"

    if has_downwash_conflict(from_a, to_a, mission_a, from_b, to_b, mission_b):
        return "downwash"
    if has_downwash_conflict(from_b, to_b, mission_b, from_a, to_a, mission_a):
        return "downwash"

    return None


def zone_contains(zone: TemporalNoFlyZoneConfig, cell: Cell, time_step: int) -> bool:
    if not zone.enabled:
        return False
    if time_step < zone.start_time_step or time_step > zone.end_time_step:
        return False
    return (
        zone.min_cell[0] <= cell[0] <= zone.max_cell[0]
        and zone.min_cell[1] <= cell[1] <= zone.max_cell[1]
        and zone.min_cell[2] <= cell[2] <= zone.max_cell[2]
    )
