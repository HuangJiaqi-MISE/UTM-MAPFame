from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from common import load_env, rollout_metrics, scenario_paths

from utm_mappo.mappo import DiscreteMAPPO  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate a trained MAPPO model across many scenarios."
    )
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--stochastic", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = scenario_paths(args.scenario_dir)
    if args.limit > 0:
        paths = paths[: args.limit]

    rows = []
    model_cache: dict[tuple[int, tuple[int, ...]], DiscreteMAPPO] = {}
    for path in paths:
        env = load_env(path)
        signature = (
            len(env.possible_agents),
            env.observation_space(env.possible_agents[0]).shape,
        )
        model = model_cache.get(signature)
        if model is None:
            model = DiscreteMAPPO.load(env, args.model_dir)
            model_cache[signature] = model
        else:
            model.env = env

        row = rollout_metrics(env, model, stochastic=args.stochastic)
        row["scenario"] = str(path)
        rows.append(row)
        print(
            f"{path.name}: reached={row['reached_count']}/{row['agents']} "
            f"time={row['time_steps']} unsafe={row['unsafe']} "
            f"osc={row['oscillations']}"
        )

    print_summary(rows)
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        write_csv(args.csv, rows)
        print(f"wrote {args.csv}")


def print_summary(rows: list[dict[str, object]]) -> None:
    if not rows:
        print("no scenarios evaluated")
        return

    success = np.asarray([bool(row["all_reached"]) for row in rows], dtype=np.float32)
    print("summary:")
    print(f"  scenarios={len(rows)}")
    print(f"  success_rate={float(success.mean()):.3f}")
    print(f"  mean_time_steps={mean(rows, 'time_steps'):.2f}")
    print(f"  mean_reached={mean(rows, 'reached_count'):.2f}")
    print(f"  mean_unsafe={mean(rows, 'unsafe'):.2f}")
    print(f"  mean_invalid={mean(rows, 'invalid'):.2f}")
    print(f"  mean_no_fly={mean(rows, 'no_fly'):.2f}")
    print(f"  mean_oscillations={mean(rows, 'oscillations'):.2f}")


def mean(rows: list[dict[str, object]], key: str) -> float:
    return float(np.mean([float(row[key]) for row in rows]))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        "scenario",
        "agents",
        "all_reached",
        "reached_count",
        "time_steps",
        "moves",
        "waits",
        "oscillations",
        "invalid",
        "no_fly",
        "unsafe",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in fieldnames})


if __name__ == "__main__":
    main()
