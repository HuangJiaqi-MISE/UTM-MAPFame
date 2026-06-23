from __future__ import annotations

import argparse
import random
import sys
from collections import deque
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from utm_mappo import UTMScenario  # noqa: E402
from utm_mappo.env import ACTION_DELTAS, UTMMAPFEnv, UTMAction  # noqa: E402

Cell = tuple[int, int, int]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate randomized UTM MAPF curriculum scenarios."
    )
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--agents", type=int, required=True)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--grid", type=int, nargs=3, default=(20, 20, 4))
    parser.add_argument("--max-time-steps", type=int, default=160)
    parser.add_argument("--observation-radius", type=int, default=2)
    parser.add_argument("--obstacle-rate", type=float, default=0.04)
    parser.add_argument("--spawn-band", type=int, default=2)
    parser.add_argument(
        "--pattern",
        choices=("flows", "random", "mixed"),
        default="mixed",
        help="flows creates opposing boundary traffic; random samples arbitrary pairs.",
    )
    parser.add_argument("--no-fly-zones", type=int, default=1)
    parser.add_argument("--protection-xy-radius", type=int, default=0)
    parser.add_argument("--downwash-xy-radius", type=int, default=1)
    parser.add_argument("--downwash-z-below", type=int, default=1)
    parser.add_argument("--prefix", type=str, default="scenario")
    parser.add_argument("--max-scenario-attempts", type=int, default=1000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rng = random.Random(args.seed)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    generated = 0
    attempts = 0
    while generated < args.count and attempts < args.max_scenario_attempts:
        attempts += 1
        scenario = try_make_scenario(args, rng)
        if scenario is None:
            continue

        path = args.out_dir / f"{args.prefix}_{args.agents:03d}_{generated:04d}.yaml"
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(
                scenario.to_dict(),
                handle,
                sort_keys=False,
                allow_unicode=False,
            )
        generated += 1

    if generated != args.count:
        raise RuntimeError(
            f"generated {generated}/{args.count} scenarios after {attempts} attempts"
        )

    print(f"generated {generated} scenarios in {args.out_dir}")


def try_make_scenario(args: argparse.Namespace, rng: random.Random) -> UTMScenario | None:
    dimensions = tuple(args.grid)
    blocked = make_blocked_cells(dimensions, args.obstacle_rate, rng)
    missions = []
    used_starts: set[Cell] = set()
    used_goals: set[Cell] = set()

    for mission_id in range(1, args.agents + 1):
        pair = choose_mission_pair(
            dimensions=dimensions,
            blocked=blocked,
            used_starts=used_starts,
            used_goals=used_goals,
            mission_index=mission_id - 1,
            pattern=args.pattern,
            spawn_band=args.spawn_band,
            rng=rng,
        )
        if pair is None:
            return None
        start, goal = pair
        if not static_reachable(start, goal, dimensions, blocked):
            return None

        used_starts.add(start)
        used_goals.add(goal)
        missions.append(
            {
                "mission_id": mission_id,
                "start": list(start),
                "goal": list(goal),
                "protection_xy_radius": args.protection_xy_radius,
                "downwash_xy_radius": args.downwash_xy_radius,
                "downwash_z_below": args.downwash_z_below,
            }
        )

    scenario_data = {
        "grid": {
            "dimensions": list(dimensions),
            "blocked_cells": [list(cell) for cell in sorted(blocked)],
        },
        "missions": missions,
        "no_fly_zones": make_no_fly_zones(
            dimensions, args.no_fly_zones, args.max_time_steps, rng
        ),
        "max_time_steps": args.max_time_steps,
        "observation_radius": args.observation_radius,
    }

    try:
        scenario = UTMScenario.from_dict(scenario_data)
        UTMMAPFEnv(scenario)
    except ValueError:
        return None
    return scenario


def make_blocked_cells(
    dimensions: Cell, obstacle_rate: float, rng: random.Random
) -> set[Cell]:
    obstacle_rate = min(max(obstacle_rate, 0.0), 0.4)
    blocked: set[Cell] = set()
    for x in range(dimensions[0]):
        for y in range(dimensions[1]):
            for z in range(dimensions[2]):
                if rng.random() < obstacle_rate:
                    blocked.add((x, y, z))
    return blocked


def choose_mission_pair(
    dimensions: Cell,
    blocked: set[Cell],
    used_starts: set[Cell],
    used_goals: set[Cell],
    mission_index: int,
    pattern: str,
    spawn_band: int,
    rng: random.Random,
) -> tuple[Cell, Cell] | None:
    active_pattern = pattern
    if pattern == "mixed":
        active_pattern = "flows" if rng.random() < 0.7 else "random"

    for _ in range(200):
        if active_pattern == "random":
            start = random_cell(dimensions, rng)
            goal = random_cell(dimensions, rng)
        else:
            start, goal = flow_pair(dimensions, mission_index, spawn_band, rng)

        if start == goal:
            continue
        if start in blocked or goal in blocked:
            continue
        if start in used_starts or goal in used_goals:
            continue
        return start, goal
    return None


def random_cell(dimensions: Cell, rng: random.Random) -> Cell:
    return (
        rng.randrange(dimensions[0]),
        rng.randrange(dimensions[1]),
        rng.randrange(dimensions[2]),
    )


def flow_pair(
    dimensions: Cell,
    mission_index: int,
    spawn_band: int,
    rng: random.Random,
) -> tuple[Cell, Cell]:
    x_max, y_max, z_max = dimensions[0] - 1, dimensions[1] - 1, dimensions[2] - 1
    band = max(1, min(spawn_band, dimensions[0], dimensions[1]))
    z = rng.randrange(max(1, z_max + 1))
    direction = mission_index % 4

    if direction == 0:
        return (
            (rng.randrange(band), rng.randrange(dimensions[1]), z),
            (x_max - rng.randrange(band), rng.randrange(dimensions[1]), z),
        )
    if direction == 1:
        return (
            (x_max - rng.randrange(band), rng.randrange(dimensions[1]), z),
            (rng.randrange(band), rng.randrange(dimensions[1]), z),
        )
    if direction == 2:
        return (
            (rng.randrange(dimensions[0]), rng.randrange(band), z),
            (rng.randrange(dimensions[0]), y_max - rng.randrange(band), z),
        )
    return (
        (rng.randrange(dimensions[0]), y_max - rng.randrange(band), z),
        (rng.randrange(dimensions[0]), rng.randrange(band), z),
    )


def make_no_fly_zones(
    dimensions: Cell,
    count: int,
    max_time_steps: int,
    rng: random.Random,
) -> list[dict[str, object]]:
    zones = []
    for zone_id in range(1, count + 1):
        width = max(1, dimensions[0] // 8)
        height = max(1, dimensions[1] // 8)
        x0 = rng.randrange(0, max(1, dimensions[0] - width))
        y0 = rng.randrange(0, max(1, dimensions[1] - height))
        z = rng.randrange(dimensions[2])
        start = rng.randrange(max(1, max_time_steps // 5), max(2, max_time_steps // 2))
        duration = rng.randrange(3, max(4, max_time_steps // 8))
        zones.append(
            {
                "zone_id": zone_id,
                "enabled": True,
                "min_cell": [x0, y0, z],
                "max_cell": [min(dimensions[0] - 1, x0 + width), min(dimensions[1] - 1, y0 + height), z],
                "start_time_step": start,
                "end_time_step": min(max_time_steps - 1, start + duration),
            }
        )
    return zones


def static_reachable(
    start: Cell,
    goal: Cell,
    dimensions: Cell,
    blocked: set[Cell],
) -> bool:
    queue: deque[Cell] = deque([start])
    visited = {start}
    while queue:
        cell = queue.popleft()
        if cell == goal:
            return True

        for action, delta in ACTION_DELTAS.items():
            if action == UTMAction.WAIT:
                continue
            candidate = (
                cell[0] + delta[0],
                cell[1] + delta[1],
                cell[2] + delta[2],
            )
            if candidate in visited or candidate in blocked:
                continue
            if not (
                0 <= candidate[0] < dimensions[0]
                and 0 <= candidate[1] < dimensions[1]
                and 0 <= candidate[2] < dimensions[2]
            ):
                continue
            visited.add(candidate)
            queue.append(candidate)
    return False


if __name__ == "__main__":
    main()
