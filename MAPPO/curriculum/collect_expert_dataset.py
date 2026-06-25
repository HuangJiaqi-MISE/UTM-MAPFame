from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np
from tqdm import tqdm

from common import load_env, scenario_paths

from utm_mappo import UTMAction, UTMMAPFEnv  # noqa: E402
from utm_mappo.cbs_expert import cbs_plan  # noqa: E402
from utm_mappo.expert import prioritized_pibt_action_plan  # noqa: E402
from utm_mappo.space_time_expert import (  # noqa: E402
    action_from_transition,
    planned_cell,
    prioritized_space_time_plan,
)


METRIC_KEYS = (
    "moves",
    "waits",
    "oscillations",
    "invalid",
    "no_fly",
    "unsafe",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect offline behavior-cloning data from the UTM expert."
    )
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument(
        "--teacher",
        choices=("space-time", "pibt", "cbs"),
        default="space-time",
        help=(
            "Teacher used to generate demonstrations. space-time is an offline "
            "prioritized reservation planner; pibt precomputes the online heuristic; "
            "cbs is a bounded CPU Conflict-Based Search teacher for comparison."
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
        help="Priority-order attempts for the offline space-time teacher.",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--include-failures",
        action="store_true",
        help=(
            "Keep trajectories even when the teacher does not produce a clean "
            "success. By default, failed or unsafe rollouts are skipped."
        ),
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing dataset directory.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = scenario_paths(args.scenario_dir)
    if args.limit > 0:
        paths = paths[: args.limit]

    prepare_dataset_dir(args.dataset_dir, overwrite=args.overwrite)

    expected_signature: tuple[tuple[str, ...], tuple[int, ...], int] | None = None
    rows: list[dict[str, Any]] = []
    shards: list[dict[str, Any]] = []
    stored_index = 0
    total_samples = 0

    progress = tqdm(paths, desc="collect expert")
    for scenario_index, path in enumerate(progress):
        env = load_env(path)
        expected_signature = validate_signature(env, expected_signature, path)

        if args.teacher == "cbs":
            arrays, metrics = collect_cbs_rollout(
                env,
                max_high_level_nodes=args.cbs_max_nodes,
                max_low_level_expansions=args.cbs_max_low_level_expansions,
                max_seconds=args.cbs_max_seconds,
            )
        elif args.teacher == "space-time":
            arrays, metrics = collect_space_time_rollout(
                env,
                planner_retries=args.planner_retries,
                seed=args.seed + scenario_index,
            )
        else:
            arrays, metrics = collect_pibt_rollout(env)
        metrics["scenario"] = str(path)
        metrics["samples"] = int(arrays["actions"].shape[0])
        metrics["clean"] = is_clean_success(metrics)
        rows.append(metrics)

        keep = bool(metrics["clean"]) or args.include_failures
        if keep and metrics["samples"] > 0:
            shard_name = f"shard_{stored_index:06d}_{path.stem}.npz"
            np.savez_compressed(args.dataset_dir / shard_name, **arrays)
            shards.append(
                {
                    "file": shard_name,
                    "scenario": str(path),
                    "scenario_index": scenario_index,
                    "samples": int(metrics["samples"]),
                    "all_reached": bool(metrics["all_reached"]),
                    "clean": bool(metrics["clean"]),
                    "reached_count": int(metrics["reached_count"]),
                    "time_steps": int(metrics["time_steps"]),
                    "unsafe": int(metrics["unsafe"]),
                    "oscillations": int(metrics["oscillations"]),
                    "teacher": args.teacher,
                    "planner_reason": metrics.get("planner_reason"),
                    "cbs_expanded_nodes": int(metrics.get("cbs_expanded_nodes", 0)),
                    "cbs_generated_nodes": int(metrics.get("cbs_generated_nodes", 0)),
                    "cbs_low_level_searches": int(
                        metrics.get("cbs_low_level_searches", 0)
                    ),
                    "cbs_low_level_expansions": int(
                        metrics.get("cbs_low_level_expansions", 0)
                    ),
                    "cbs_elapsed_seconds": float(
                        metrics.get("cbs_elapsed_seconds", 0.0)
                    ),
                }
            )
            stored_index += 1
            total_samples += int(metrics["samples"])

        progress.set_postfix(
            kept=stored_index,
            reached=f"{metrics['reached_count']}/{metrics['agents']}",
            unsafe=metrics["unsafe"],
        )
        print(
            f"{path.name}: reached={metrics['reached_count']}/{metrics['agents']} "
            f"time={metrics['time_steps']} unsafe={metrics['unsafe']} "
            f"osc={metrics['oscillations']} "
            f"teacher={args.teacher} "
            f"{'clean' if metrics['clean'] else 'not-clean'} "
            f"{'kept' if keep else 'skipped'}"
        )

    if not shards:
        raise RuntimeError(
            "no expert trajectories were stored; use --include-failures to keep "
            "failed/unsafe rollouts for debugging, or generate easier scenarios."
        )

    metadata = build_metadata(
        args=args,
        rows=rows,
        shards=shards,
        total_samples=total_samples,
        expected_signature=expected_signature,
    )
    with (args.dataset_dir / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)

    print_summary(rows, shards, total_samples)
    print(f"wrote expert dataset to {args.dataset_dir}")


def prepare_dataset_dir(path: Path, overwrite: bool) -> None:
    if path.exists() and any(path.iterdir()):
        if not overwrite:
            raise FileExistsError(
                f"{path} is not empty; choose a new --dataset-dir or pass --overwrite"
            )
        for child in path.iterdir():
            if child.is_dir():
                shutil.rmtree(child)
            else:
                child.unlink()
    path.mkdir(parents=True, exist_ok=True)


def validate_signature(
    env: UTMMAPFEnv,
    expected: tuple[tuple[str, ...], tuple[int, ...], int] | None,
    path: Path,
) -> tuple[tuple[str, ...], tuple[int, ...], int]:
    agents = tuple(env.possible_agents)
    obs_shape = env.observation_space(agents[0]).shape
    action_dim = int(env.action_space(agents[0]).n)
    signature = (agents, obs_shape, action_dim)
    if expected is not None and signature != expected:
        raise ValueError(
            "offline BC dataset requires identical mission ids and observation "
            f"shape per stage; {path} has {signature}, expected {expected}"
        )
    return signature


def collect_space_time_rollout(
    env: UTMMAPFEnv,
    planner_retries: int,
    seed: int,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    env.reset()
    plan = prioritized_space_time_plan(
        env,
        max_retries=planner_retries,
        seed=seed,
    )
    if plan is None:
        metrics = empty_metrics(env)
        metrics["planner_failed"] = True
        return empty_arrays(env), metrics
    return collect_planned_rollout(env, plan)


def collect_cbs_rollout(
    env: UTMMAPFEnv,
    max_high_level_nodes: int,
    max_low_level_expansions: int,
    max_seconds: float,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    env.reset()
    result = cbs_plan(
        env,
        max_high_level_nodes=max_high_level_nodes,
        max_low_level_expansions=max_low_level_expansions,
        max_seconds=max_seconds,
    )
    if result.plan is None:
        metrics = empty_metrics(env)
        metrics["planner_failed"] = True
        metrics["planner_reason"] = result.reason
        metrics["cbs_expanded_nodes"] = result.expanded_nodes
        metrics["cbs_generated_nodes"] = result.generated_nodes
        metrics["cbs_low_level_searches"] = result.low_level_searches
        metrics["cbs_low_level_expansions"] = result.low_level_expansions
        metrics["cbs_elapsed_seconds"] = result.elapsed_seconds
        return empty_arrays(env), metrics

    arrays, metrics = collect_planned_rollout(env, result.plan)
    metrics["planner_failed"] = False
    metrics["planner_reason"] = result.reason
    metrics["cbs_expanded_nodes"] = result.expanded_nodes
    metrics["cbs_generated_nodes"] = result.generated_nodes
    metrics["cbs_low_level_searches"] = result.low_level_searches
    metrics["cbs_low_level_expansions"] = result.low_level_expansions
    metrics["cbs_elapsed_seconds"] = result.elapsed_seconds
    return arrays, metrics


def collect_planned_rollout(
    env: UTMMAPFEnv,
    plan: dict[str, list[tuple[int, int, int]]],
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    observations, _ = env.reset()
    agent_to_index = {
        agent: index for index, agent in enumerate(env.possible_agents)
    }
    observation_rows: list[np.ndarray] = []
    mask_rows: list[np.ndarray] = []
    action_rows: list[int] = []
    time_rows: list[int] = []
    agent_rows: list[int] = []
    totals = {key: 0 for key in METRIC_KEYS}

    while observations:
        agents = sorted(observations)
        masks = {agent: env.action_mask(agent).copy() for agent in agents}
        actions: dict[str, int] = {}

        for agent in agents:
            state = env._state_by_agent[agent]
            next_cell = planned_cell(plan[agent], env.time_step + 1)
            action = action_from_transition(state.cell, next_cell)
            if action >= masks[agent].shape[0] or not masks[agent][action]:
                action = int(UTMAction.WAIT)
            actions[agent] = action
            append_sample(
                observations=observations,
                masks=masks,
                agent=agent,
                action=action,
                env=env,
                agent_to_index=agent_to_index,
                observation_rows=observation_rows,
                mask_rows=mask_rows,
                action_rows=action_rows,
                time_rows=time_rows,
                agent_rows=agent_rows,
            )

        before = {
            agent: env.render()["agents"][agent]["cell"] for agent in agents
        }
        observations, _, _, _, infos = env.step(actions)
        update_totals(totals, before, infos)

    metrics = final_metrics(env, totals)
    metrics["planner_failed"] = False
    return build_arrays(env, observation_rows, mask_rows, action_rows, time_rows, agent_rows), metrics


def collect_pibt_rollout(env: UTMMAPFEnv) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    action_plan = prioritized_pibt_action_plan(env)
    return collect_action_plan_rollout(env, action_plan)


def collect_action_plan_rollout(
    env: UTMMAPFEnv,
    action_plan: dict[str, list[int]],
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    observations, _ = env.reset()
    agent_to_index = {
        agent: index for index, agent in enumerate(env.possible_agents)
    }
    observation_rows: list[np.ndarray] = []
    mask_rows: list[np.ndarray] = []
    action_rows: list[int] = []
    time_rows: list[int] = []
    agent_rows: list[int] = []
    totals = {key: 0 for key in METRIC_KEYS}

    while observations:
        agents = sorted(observations)
        masks = {agent: env.action_mask(agent).copy() for agent in agents}
        actions: dict[str, int] = {}

        for agent in agents:
            agent_actions = action_plan.get(agent, [])
            if env.time_step < len(agent_actions):
                action = int(agent_actions[env.time_step])
            else:
                action = int(UTMAction.WAIT)
            if action >= masks[agent].shape[0] or not masks[agent][action]:
                action = int(UTMAction.WAIT)
            actions[agent] = action
            append_sample(
                observations=observations,
                masks=masks,
                agent=agent,
                action=action,
                env=env,
                agent_to_index=agent_to_index,
                observation_rows=observation_rows,
                mask_rows=mask_rows,
                action_rows=action_rows,
                time_rows=time_rows,
                agent_rows=agent_rows,
            )

        before = {
            agent: env.render()["agents"][agent]["cell"] for agent in agents
        }
        observations, _, _, _, infos = env.step(actions)
        update_totals(totals, before, infos)

    metrics = final_metrics(env, totals)
    metrics["planner_failed"] = False
    return build_arrays(env, observation_rows, mask_rows, action_rows, time_rows, agent_rows), metrics


def append_sample(
    observations: dict[str, np.ndarray],
    masks: dict[str, np.ndarray],
    agent: str,
    action: int,
    env: UTMMAPFEnv,
    agent_to_index: dict[str, int],
    observation_rows: list[np.ndarray],
    mask_rows: list[np.ndarray],
    action_rows: list[int],
    time_rows: list[int],
    agent_rows: list[int],
) -> None:
    observation_rows.append(
        np.asarray(observations[agent], dtype=np.float32).copy()
    )
    mask_rows.append(masks[agent].astype(np.bool_, copy=True))
    action_rows.append(action)
    time_rows.append(env.time_step)
    agent_rows.append(agent_to_index[agent])


def update_totals(
    totals: dict[str, int],
    before: dict[str, tuple[int, int, int]],
    infos: dict[str, dict[str, Any]],
) -> None:
    for agent, info in infos.items():
        if info["cell"] == before[agent]:
            totals["waits"] += 1
        else:
            totals["moves"] += 1
        if info["oscillated"]:
            totals["oscillations"] += 1
        if info["invalid_action"]:
            totals["invalid"] += 1
        if info["no_fly_hold"]:
            totals["no_fly"] += 1
        if info["unsafe_hold"]:
            totals["unsafe"] += 1


def final_metrics(env: UTMMAPFEnv, totals: dict[str, int]) -> dict[str, Any]:
    render_state = env.render()
    reached = [
        bool(agent_state["reached_goal"])
        for agent_state in render_state["agents"].values()
    ]
    metrics: dict[str, Any] = {
        "time_steps": int(render_state["time_step"]),
        "agents": len(reached),
        "all_reached": all(reached),
        "reached_count": int(np.sum(reached)),
    }
    metrics.update(totals)
    return metrics


def empty_metrics(env: UTMMAPFEnv) -> dict[str, Any]:
    render_state = env.render()
    reached = [
        bool(agent_state["reached_goal"])
        for agent_state in render_state["agents"].values()
    ]
    metrics: dict[str, Any] = {
        "time_steps": int(render_state["time_step"]),
        "agents": len(reached),
        "all_reached": all(reached),
        "reached_count": int(np.sum(reached)),
    }
    metrics.update({key: 0 for key in METRIC_KEYS})
    return metrics


def empty_arrays(env: UTMMAPFEnv) -> dict[str, np.ndarray]:
    return build_arrays(env, [], [], [], [], [])


def build_arrays(
    env: UTMMAPFEnv,
    observation_rows: list[np.ndarray],
    mask_rows: list[np.ndarray],
    action_rows: list[int],
    time_rows: list[int],
    agent_rows: list[int],
) -> dict[str, np.ndarray]:
    if observation_rows:
        observations_array = np.stack(observation_rows).astype(np.float32)
        masks_array = np.stack(mask_rows).astype(np.bool_)
    else:
        agent = env.possible_agents[0]
        observations_array = np.zeros(
            (0, *env.observation_space(agent).shape), dtype=np.float32
        )
        masks_array = np.zeros(
            (0, int(env.action_space(agent).n)), dtype=np.bool_
        )

    arrays = {
        "observations": observations_array,
        "action_masks": masks_array,
        "actions": np.asarray(action_rows, dtype=np.int64),
        "time_steps": np.asarray(time_rows, dtype=np.int32),
        "agent_indices": np.asarray(agent_rows, dtype=np.int32),
    }
    return arrays


def build_metadata(
    args: argparse.Namespace,
    rows: list[dict[str, Any]],
    shards: list[dict[str, Any]],
    total_samples: int,
    expected_signature: tuple[tuple[str, ...], tuple[int, ...], int] | None,
) -> dict[str, Any]:
    agents: tuple[str, ...] = ()
    obs_shape: tuple[int, ...] = ()
    action_dim = 0
    if expected_signature is not None:
        agents, obs_shape, action_dim = expected_signature

    success_flags = np.asarray(
        [bool(row["all_reached"]) for row in rows], dtype=np.float32
    )
    clean_flags = np.asarray(
        [bool(row.get("clean", False)) for row in rows], dtype=np.float32
    )
    return {
        "version": 1,
        "source": teacher_source(args.teacher),
        "teacher": args.teacher,
        "cbs_max_nodes": int(args.cbs_max_nodes),
        "cbs_max_low_level_expansions": int(args.cbs_max_low_level_expansions),
        "cbs_max_seconds": float(args.cbs_max_seconds),
        "planner_retries": int(args.planner_retries),
        "scenario_dir": str(args.scenario_dir),
        "scenario_dir_resolved": str(args.scenario_dir.resolve()),
        "success_only": not bool(args.include_failures),
        "clean_success_only": not bool(args.include_failures),
        "scenario_count": len(rows),
        "stored_scenario_count": len(shards),
        "skipped_scenario_count": len(rows) - len(shards),
        "samples": int(total_samples),
        "agents": list(agents),
        "n_agents": len(agents),
        "obs_shape": list(obs_shape),
        "action_dim": int(action_dim),
        "expert_success_rate": float(success_flags.mean()) if rows else 0.0,
        "expert_clean_success_rate": float(clean_flags.mean()) if rows else 0.0,
        "expert_mean_reached": mean(rows, "reached_count"),
        "expert_mean_time_steps": mean(rows, "time_steps"),
        "expert_mean_unsafe": mean(rows, "unsafe"),
        "expert_mean_oscillations": mean(rows, "oscillations"),
        "cbs_mean_expanded_nodes": mean(rows, "cbs_expanded_nodes"),
        "cbs_mean_generated_nodes": mean(rows, "cbs_generated_nodes"),
        "cbs_mean_low_level_searches": mean(rows, "cbs_low_level_searches"),
        "cbs_mean_low_level_expansions": mean(rows, "cbs_low_level_expansions"),
        "cbs_mean_elapsed_seconds": mean(rows, "cbs_elapsed_seconds"),
        "shards": shards,
    }


def teacher_source(teacher: str) -> str:
    if teacher == "cbs":
        return "utm_mappo.cbs_expert.cbs_plan"
    if teacher == "space-time":
        return "utm_mappo.space_time_expert.prioritized_space_time_plan"
    return "utm_mappo.expert.prioritized_pibt_action_plan"


def is_clean_success(metrics: dict[str, Any]) -> bool:
    return (
        bool(metrics["all_reached"])
        and int(metrics["unsafe"]) == 0
        and int(metrics["invalid"]) == 0
        and int(metrics["no_fly"]) == 0
    )


def print_summary(
    rows: list[dict[str, Any]], shards: list[dict[str, Any]], total_samples: int
) -> None:
    print("summary:")
    print(f"  scenarios={len(rows)}")
    print(f"  stored_scenarios={len(shards)}")
    print(f"  samples={total_samples}")
    print(f"  expert_success_rate={mean_bool(rows, 'all_reached'):.3f}")
    print(f"  expert_clean_success_rate={mean_bool(rows, 'clean'):.3f}")
    print(f"  expert_mean_reached={mean(rows, 'reached_count'):.2f}")
    print(f"  expert_mean_time_steps={mean(rows, 'time_steps'):.2f}")
    print(f"  expert_mean_unsafe={mean(rows, 'unsafe'):.2f}")
    print(f"  expert_mean_oscillations={mean(rows, 'oscillations'):.2f}")


def mean(rows: list[dict[str, Any]], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([float(row.get(key, 0.0)) for row in rows]))


def mean_bool(rows: list[dict[str, Any]], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([bool(row[key]) for row in rows]))


if __name__ == "__main__":
    main()
