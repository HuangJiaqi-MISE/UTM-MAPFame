from __future__ import annotations

from .config import GridConfig, MissionConfig, TemporalNoFlyZoneConfig, UTMScenario


def crossing_scenario() -> UTMScenario:
    blocked = []
    for z in range(5):
        for y in range(2, 8):
            blocked.append((5, y, z))
    blocked.remove((5, 5, 2))

    return UTMScenario(
        grid=GridConfig(dimensions=(10, 10, 5), blocked_cells=tuple(blocked)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(1, 5, 2),
                goal=(8, 5, 2),
            ),
            MissionConfig(
                mission_id=2,
                start=(8, 5, 2),
                goal=(1, 5, 2),
            ),
            MissionConfig(
                mission_id=3,
                start=(5, 1, 2),
                goal=(5, 8, 2),
            ),
            MissionConfig(
                mission_id=4,
                start=(5, 8, 2),
                goal=(5, 1, 2),
            ),
        ),
        no_fly_zones=(
            TemporalNoFlyZoneConfig(
                zone_id=1,
                min_cell=(4, 4, 2),
                max_cell=(6, 6, 2),
                start_time_step=8,
                end_time_step=12,
            ),
        ),
        max_time_steps=80,
        observation_radius=2,
    )
