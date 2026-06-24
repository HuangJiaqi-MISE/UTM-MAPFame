from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from common import load_env, rollout_metrics, rollout_metrics_with_policy, scenario_paths

from utm_mappo.expert import prioritized_shortest_path_actions  # noqa: E402
from utm_mappo.mappo import DiscreteMAPPO  # noqa: E402
from utm_mappo.space_time_expert import (  # noqa: E402
    action_from_transition,
    planned_cell,
    prioritized_space_time_plan,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate a trained MAPPO model across many scenarios."
    )
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, default=None)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--stochastic", action="store_true")
    parser.add_argument(
        "--expert-only",
        action="store_true",
        help="Evaluate a teacher instead of a model checkpoint.",
    )
    parser.add_argument(
        "--teacher",
        choices=("space-time", "pibt"),
        default="space-time",
        help=(
            "Teacher for --expert-only. space-time plans once per scenario; "
            "pibt is the older online one-step heuristic."
        ),
    )
    parser.add_argument(
        "--planner-retries",
        type=int,
        default=64,
        help="Priority-order attempts for the space-time teacher.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = scenario_paths(args.scenario_dir)
    if args.limit > 0:
        paths = paths[: args.limit]

    rows = []
    model_cache: dict[tuple[int, tuple[int, ...]], DiscreteMAPPO] = {}
    if not args.expert_only and args.model_dir is None:
        raise ValueError("--model-dir is required unless --expert-only is set")

    for path in paths:
        env = load_env(path)
        if args.expert_only:
            row = evaluate_teacher(env, args.teacher, args.planner_retries)
        else:
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
            f"{' planner_failed' if row.get('planner_failed') else ''}"
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


def evaluate_teacher(
    env,
    teacher: str,
    planner_retries: int,
) -> dict[str, object]:
    if teacher == "pibt":
        return rollout_metrics_with_policy(
            env,
            lambda observations, active_env=env: prioritized_shortest_path_actions(
                active_env, sorted(observations)
            ),
        )

    env.reset()
    plan = prioritized_space_time_plan(
        env,
        max_retries=planner_retries,
        seed=1,
    )
    if plan is None:
        return failed_teacher_row(env)

    return rollout_metrics_with_policy(
        env,
        lambda observations, active_env=env, active_plan=plan: planned_actions(
            active_env,
            active_plan,
            observations,
        ),
    )


def planned_actions(env, plan, observations: dict[str, np.ndarray]) -> dict[str, int]:
    actions = {}
    for agent in observations:
        state = env._state_by_agent[agent]
        next_cell = planned_cell(plan[agent], env.time_step + 1)
        actions[agent] = action_from_transition(state.cell, next_cell)
    return actions


def failed_teacher_row(env) -> dict[str, object]:
    env.reset()
    render_state = env.render()
    reached = [
        bool(agent_state["reached_goal"])
        for agent_state in render_state["agents"].values()
    ]
    return {
        "time_steps": int(render_state["time_step"]),
        "agents": len(reached),
        "all_reached": all(reached),
        "reached_count": int(np.sum(reached)),
        "moves": 0,
        "waits": 0,
        "oscillations": 0,
        "invalid": 0,
        "no_fly": 0,
        "unsafe": 0,
        "planner_failed": True,
    }


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
