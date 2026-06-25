from __future__ import annotations

import random
from argparse import Namespace

from curriculum.generate_scenarios import (
    has_pairwise_cell_conflict,
    manhattan_distance,
    sample_box,
    try_make_scenario,
)
from utm_mappo.config import MissionConfig


def test_sample_box_respects_3d_size_range() -> None:
    rng = random.Random(7)
    dimensions = (10, 10, 5)

    for _ in range(20):
        min_cell, max_cell = sample_box(
            dimensions=dimensions,
            size_min=(1, 2, 1),
            size_max=(3, 4, 2),
            rng=rng,
        )
        size = tuple(max_cell[index] - min_cell[index] + 1 for index in range(3))

        assert all(
            0 <= min_cell[index] <= max_cell[index] < dimensions[index]
            for index in range(3)
        )
        assert 1 <= size[0] <= 3
        assert 2 <= size[1] <= 4
        assert 1 <= size[2] <= 2


def test_generated_missions_can_enforce_disjoint_distance_constraints() -> None:
    args = Namespace(
        grid=(10, 10, 5),
        obstacle_rate=0.0,
        obstacle_count=0,
        obstacle_size_min=(1, 1, 1),
        obstacle_size_max=(1, 1, 1),
        agents=4,
        pattern="random",
        spawn_band=2,
        disjoint_start_goal=True,
        min_goal_distance=2,
        max_goal_distance=6,
        no_fly_zones=0,
        no_fly_size_min=(1, 1, 1),
        no_fly_size_max=(1, 1, 1),
        max_time_steps=80,
        observation_radius=2,
    )

    scenario = try_make_scenario(args, random.Random(11))

    assert scenario is not None
    starts = [mission.start for mission in scenario.missions]
    goals = [mission.goal for mission in scenario.missions]
    assert len(set(starts + goals)) == len(starts) + len(goals)
    for mission in scenario.missions:
        distance = manhattan_distance(mission.start, mission.goal)
        assert 2 <= distance <= 6
    assert not has_pairwise_cell_conflict(scenario.missions, "goal")


def test_pairwise_cell_conflict_detects_goal_downwash_conflict() -> None:
    missions = (
        MissionConfig(
            mission_id=1,
            start=(0, 0, 2),
            goal=(1, 1, 2),
        ),
        MissionConfig(
            mission_id=2,
            start=(2, 2, 0),
            goal=(1, 1, 1),
        ),
    )

    assert has_pairwise_cell_conflict(missions, "goal")
