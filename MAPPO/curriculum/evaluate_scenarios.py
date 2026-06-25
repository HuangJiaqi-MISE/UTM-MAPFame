from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

from common import load_env, rollout_metrics, rollout_metrics_with_policy, scenario_paths

from utm_mappo.cbs_expert import cbs_plan  # noqa: E402
from utm_mappo.env import UTMAction  # noqa: E402
from utm_mappo.expert import prioritized_pibt_action_plan  # noqa: E402
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
        "--diagnostics",
        action="store_true",
        help=(
            "Print per-scenario timing for scenario loading, teacher planning, "
            "and environment rollout."
        ),
    )
    parser.add_argument(
        "--expert-only",
        action="store_true",
        help="Evaluate a teacher instead of a model checkpoint.",
    )
    parser.add_argument(
        "--teacher",
        choices=("space-time", "pibt", "cbs"),
        default="space-time",
        help=(
            "Teacher for --expert-only. space-time plans once per scenario; "
            "pibt precomputes the online one-step heuristic; cbs is the CPU "
            "Conflict-Based Search teacher."
        ),
    )
    parser.add_argument(
        "--cbs-max-nodes",
        type=int,
        default=0,
        help="Maximum CBS high-level nodes. Use 0 for no explicit node limit.",
    )
    parser.add_argument(
        "--cbs-max-low-level-expansions",
        type=int,
        default=0,
        help="Maximum total low-level A* expansions per CBS solve. Use 0 for no limit.",
    )
    parser.add_argument(
        "--cbs-max-seconds",
        type=float,
        default=0.0,
        help="Wall-clock seconds allowed per CBS solve. Use 0 for no time limit.",
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
        scenario_started = time.perf_counter()
        load_started = time.perf_counter()
        env = load_env(path)
        load_seconds = time.perf_counter() - load_started
        if args.expert_only:
            row = evaluate_teacher(
                env,
                args.teacher,
                args.planner_retries,
                args.cbs_max_nodes,
                args.cbs_max_low_level_expansions,
                args.cbs_max_seconds,
            )
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
            rollout_started = time.perf_counter()
            row = rollout_metrics(env, model, stochastic=args.stochastic)
            row["planning_seconds"] = 0.0
            row["rollout_seconds"] = time.perf_counter() - rollout_started

        row["load_seconds"] = load_seconds
        row["scenario_wall_seconds"] = time.perf_counter() - scenario_started
        row["scenario"] = str(path)
        rows.append(row)
        print(
            f"{path.name}: reached={row['reached_count']}/{row['agents']} "
            f"time={row['time_steps']} unsafe={row['unsafe']} "
            f"osc={row['oscillations']}"
            f"{' planner_failed' if row.get('planner_failed') else ''}"
        )
        if args.diagnostics:
            print_diagnostics(path, row)

    print_summary(rows, show_timing=args.diagnostics)
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        write_csv(args.csv, rows)
        print(f"wrote {args.csv}")


def print_summary(rows: list[dict[str, object]], show_timing: bool = False) -> None:
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
    if show_timing:
        print(f"  mean_load_seconds={mean(rows, 'load_seconds'):.4f}")
        print(f"  mean_planning_seconds={mean(rows, 'planning_seconds'):.4f}")
        print(f"  mean_rollout_seconds={mean(rows, 'rollout_seconds'):.4f}")
        print(f"  mean_wall_seconds={mean(rows, 'scenario_wall_seconds'):.4f}")


def print_diagnostics(path: Path, row: dict[str, object]) -> None:
    print("  diagnostics:")
    print(f"    scenario={path}")
    print(f"    load_seconds={float(row.get('load_seconds', 0.0)):.4f}")
    print(f"    planning_seconds={float(row.get('planning_seconds', 0.0)):.4f}")
    print(f"    rollout_seconds={float(row.get('rollout_seconds', 0.0)):.4f}")
    print(f"    wall_seconds={float(row.get('scenario_wall_seconds', 0.0)):.4f}")
    if "planner_reason" in row:
        print(f"    planner_reason={row.get('planner_reason')}")
    if "cbs_elapsed_seconds" in row:
        print(f"    cbs_elapsed_seconds={float(row.get('cbs_elapsed_seconds', 0.0)):.4f}")
        print(f"    cbs_expanded_nodes={int(row.get('cbs_expanded_nodes', 0))}")
        print(f"    cbs_generated_nodes={int(row.get('cbs_generated_nodes', 0))}")
        print(f"    cbs_low_level_searches={int(row.get('cbs_low_level_searches', 0))}")
        print(
            "    cbs_low_level_expansions="
            f"{int(row.get('cbs_low_level_expansions', 0))}"
        )


def evaluate_teacher(
    env,
    teacher: str,
    planner_retries: int,
    cbs_max_nodes: int,
    cbs_max_low_level_expansions: int,
    cbs_max_seconds: float,
) -> dict[str, object]:
    if teacher == "pibt":
        planning_started = time.perf_counter()
        action_plan = prioritized_pibt_action_plan(env)
        planning_seconds = time.perf_counter() - planning_started

        rollout_started = time.perf_counter()
        row = rollout_metrics_with_policy(
            env,
            lambda observations, active_env=env, active_plan=action_plan: action_plan_actions(
                active_env,
                active_plan,
                observations,
            ),
        )
        row["planning_seconds"] = planning_seconds
        row["rollout_seconds"] = time.perf_counter() - rollout_started
        row["planner_failed"] = False
        return row

    if teacher == "cbs":
        env.reset()
        planning_started = time.perf_counter()
        result = cbs_plan(
            env,
            max_high_level_nodes=cbs_max_nodes,
            max_low_level_expansions=cbs_max_low_level_expansions,
            max_seconds=cbs_max_seconds,
        )
        planning_seconds = time.perf_counter() - planning_started
        if result.plan is None:
            row = failed_teacher_row(env)
            row["planning_seconds"] = planning_seconds
            row["rollout_seconds"] = 0.0
            row["planner_reason"] = result.reason
            row["cbs_expanded_nodes"] = result.expanded_nodes
            row["cbs_generated_nodes"] = result.generated_nodes
            row["cbs_low_level_searches"] = result.low_level_searches
            row["cbs_low_level_expansions"] = result.low_level_expansions
            row["cbs_elapsed_seconds"] = result.elapsed_seconds
            return row

        rollout_started = time.perf_counter()
        row = rollout_metrics_with_policy(
            env,
            lambda observations, active_env=env, active_plan=result.plan: planned_actions(
                active_env,
                active_plan,
                observations,
            ),
        )
        row["planning_seconds"] = planning_seconds
        row["rollout_seconds"] = time.perf_counter() - rollout_started
        row["planner_failed"] = False
        row["planner_reason"] = result.reason
        row["cbs_expanded_nodes"] = result.expanded_nodes
        row["cbs_generated_nodes"] = result.generated_nodes
        row["cbs_low_level_searches"] = result.low_level_searches
        row["cbs_low_level_expansions"] = result.low_level_expansions
        row["cbs_elapsed_seconds"] = result.elapsed_seconds
        return row

    env.reset()
    planning_started = time.perf_counter()
    plan = prioritized_space_time_plan(
        env,
        max_retries=planner_retries,
        seed=1,
    )
    planning_seconds = time.perf_counter() - planning_started
    if plan is None:
        row = failed_teacher_row(env)
        row["planning_seconds"] = planning_seconds
        row["rollout_seconds"] = 0.0
        return row

    rollout_started = time.perf_counter()
    row = rollout_metrics_with_policy(
        env,
        lambda observations, active_env=env, active_plan=plan: planned_actions(
            active_env,
            active_plan,
            observations,
        ),
    )
    row["planning_seconds"] = planning_seconds
    row["rollout_seconds"] = time.perf_counter() - rollout_started
    return row


def planned_actions(env, plan, observations: dict[str, np.ndarray]) -> dict[str, int]:
    actions = {}
    for agent in observations:
        state = env._state_by_agent[agent]
        next_cell = planned_cell(plan[agent], env.time_step + 1)
        actions[agent] = action_from_transition(state.cell, next_cell)
    return actions


def action_plan_actions(
    env,
    action_plan: dict[str, list[int]],
    observations: dict[str, np.ndarray],
) -> dict[str, int]:
    actions = {}
    for agent in observations:
        agent_actions = action_plan.get(agent, [])
        if env.time_step < len(agent_actions):
            actions[agent] = int(agent_actions[env.time_step])
        else:
            actions[agent] = int(UTMAction.WAIT)
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
        "planner_failed",
        "planner_reason",
        "load_seconds",
        "planning_seconds",
        "rollout_seconds",
        "scenario_wall_seconds",
        "cbs_expanded_nodes",
        "cbs_generated_nodes",
        "cbs_low_level_searches",
        "cbs_low_level_expansions",
        "cbs_elapsed_seconds",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


if __name__ == "__main__":
    main()
