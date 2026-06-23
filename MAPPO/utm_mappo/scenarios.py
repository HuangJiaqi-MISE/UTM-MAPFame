from __future__ import annotations

from .config import GridConfig, MissionConfig, TemporalNoFlyZoneConfig, UTMScenario


def crossing_scenario() -> UTMScenario:
    blocked = []
    for z in range(3):
        for y in range(3, 9):
            blocked.append((6, y, z))
    blocked.remove((6, 6, 1))

    return UTMScenario(
        grid=GridConfig(dimensions=(12, 12, 4), blocked_cells=tuple(blocked)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(1, 6, 1),
                goal=(10, 6, 1),
                protection_xy_radius=0,
                downwash_xy_radius=1,
                downwash_z_below=1,
            ),
            MissionConfig(
                mission_id=2,
                start=(10, 6, 1),
                goal=(1, 6, 1),
                protection_xy_radius=0,
                downwash_xy_radius=1,
                downwash_z_below=1,
            ),
            MissionConfig(
                mission_id=3,
                start=(6, 1, 1),
                goal=(6, 10, 1),
                protection_xy_radius=0,
                downwash_xy_radius=1,
                downwash_z_below=1,
            ),
            MissionConfig(
                mission_id=4,
                start=(6, 10, 1),
                goal=(6, 1, 1),
                protection_xy_radius=0,
                downwash_xy_radius=1,
                downwash_z_below=1,
            ),
        ),
        no_fly_zones=(
            TemporalNoFlyZoneConfig(
                zone_id=1,
                min_cell=(5, 5, 1),
                max_cell=(7, 7, 1),
                start_time_step=8,
                end_time_step=12,
            ),
        ),
        max_time_steps=80,
        observation_radius=2,
    )
