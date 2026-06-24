from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np
from tqdm import tqdm

from collect_expert_dataset import (  # noqa: E402
    METRIC_KEYS,
    append_sample,
    build_arrays,
    final_metrics,
    is_clean_success,
    mean,
    mean_bool,
    prepare_dataset_dir,
    update_totals,
    validate_signature,
)
from common import load_env, scenario_paths

from utm_mappo import UTMAction, UTMMAPFEnv  # noqa: E402
from utm_mappo.expert import prioritized_shortest_path_actions  # noqa: E402
from utm_mappo.mappo import DiscreteMAPPO  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Collect DAgger-style recovery data by rolling in with a student "
            "policy and labeling visited states with the online UTM teacher."
        )
    )
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--policy-model-dir", type=Path, required=True)
    parser.add_argument(
        "--base-dataset-dir",
        type=Path,
        default=None,
        help=(
            "Optional clean expert dataset to copy into the output before "
            "appending recovery shards."
        ),
    )
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--episodes-per-scenario", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--epsilon",
        type=float,
        default=0.0,
        help="Per-agent probability of taking a random valid action during roll-in.",
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Sample from the student policy instead of taking argmax actions.",
    )
    parser.add_argument(
        "--teacher-rollin-prob",
        type=float,
        default=0.0,
        help=(
            "Per-agent probability of executing the teacher action instead of "
            "the student action. Labels are always teacher actions."
        ),
    )
    parser.add_argument(
        "--only-disagreements",
        action="store_true",
        help="Store only states where the student and teacher actions differ.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing dataset directory.",
    )
    parser.add_argument(
        "--device",
        default="auto",
        help="Torch device for policy inference: auto, cuda, cuda:0, or cpu.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = scenario_paths(args.scenario_dir)
    if args.limit > 0:
        paths = paths[: args.limit]
    if args.episodes_per_scenario <= 0:
        raise ValueError("--episodes-per-scenario must be positive")
    if not 0.0 <= args.epsilon <= 1.0:
        raise ValueError("--epsilon must be in [0, 1]")
    if not 0.0 <= args.teacher_rollin_prob <= 1.0:
        raise ValueError("--teacher-rollin-prob must be in [0, 1]")

    prepare_dataset_dir(args.dataset_dir, overwrite=args.overwrite)
    expected_signature = validate_signature(load_env(paths[0]), None, paths[0])

    shards: list[dict[str, Any]] = []
    base_metadata: dict[str, Any] | None = None
    total_samples = 0
    if args.base_dataset_dir is not None:
        base_metadata, base_shards, base_samples = copy_base_dataset(
            base_dataset_dir=args.base_dataset_dir,
            dataset_dir=args.dataset_dir,
            expected_signature=expected_signature,
        )
        shards.extend(base_shards)
        total_samples += base_samples

    rows: list[dict[str, Any]] = []
    model_cache: dict[tuple[int, tuple[int, ...]], DiscreteMAPPO] = {}
    rng = np.random.default_rng(args.seed)
    recovery_index = 0
    progress = tqdm(paths, desc="collect dagger")

    for scenario_index, path in enumerate(progress):
        env = load_env(path)
        expected_signature = validate_signature(env, expected_signature, path)
        model = load_policy_model(
            env=env,
            model_dir=args.policy_model_dir,
            model_cache=model_cache,
            device=parse_device(args.device),
        )

        for episode_index in range(args.episodes_per_scenario):
            episode_seed = args.seed + scenario_index * args.episodes_per_scenario
            episode_seed += episode_index
            arrays, metrics = collect_recovery_rollout(
                env=env,
                model=model,
                rng=rng,
                seed=episode_seed,
                stochastic=args.stochastic,
                epsilon=args.epsilon,
                teacher_rollin_prob=args.teacher_rollin_prob,
                only_disagreements=args.only_disagreements,
            )
            metrics["scenario"] = str(path)
            metrics["scenario_index"] = scenario_index
            metrics["episode_index"] = episode_index
            metrics["samples"] = int(arrays["actions"].shape[0])
            metrics["clean"] = is_clean_success(metrics)
            rows.append(metrics)

            if metrics["samples"] > 0:
                shard_name = (
                    f"dagger_{recovery_index:06d}_{path.stem}_"
                    f"ep{episode_index:02d}.npz"
                )
                np.savez_compressed(args.dataset_dir / shard_name, **arrays)
                shards.append(
                    {
                        "file": shard_name,
                        "kind": "dagger_recovery",
                        "scenario": str(path),
                        "scenario_index": scenario_index,
                        "episode_index": episode_index,
                        "samples": int(metrics["samples"]),
                        "all_reached": bool(metrics["all_reached"]),
                        "clean": bool(metrics["clean"]),
                        "reached_count": int(metrics["reached_count"]),
                        "time_steps": int(metrics["time_steps"]),
                        "unsafe": int(metrics["unsafe"]),
                        "oscillations": int(metrics["oscillations"]),
                        "disagreements": int(metrics["disagreements"]),
                        "disagreement_rate": float(metrics["disagreement_rate"]),
                    }
                )
                recovery_index += 1
                total_samples += int(metrics["samples"])

            progress.set_postfix(
                recovery=recovery_index,
                reached=f"{metrics['reached_count']}/{metrics['agents']}",
                disagree=f"{metrics['disagreement_rate']:.2f}",
            )
            print(
                f"{path.name} ep={episode_index}: "
                f"reached={metrics['reached_count']}/{metrics['agents']} "
                f"time={metrics['time_steps']} unsafe={metrics['unsafe']} "
                f"osc={metrics['oscillations']} "
                f"samples={metrics['samples']} "
                f"disagree={metrics['disagreement_rate']:.3f}"
            )

    if total_samples <= 0 or not shards:
        raise RuntimeError("no DAgger samples were stored")

    metadata = build_dagger_metadata(
        args=args,
        rows=rows,
        shards=shards,
        total_samples=total_samples,
        expected_signature=expected_signature,
        base_metadata=base_metadata,
    )
    with (args.dataset_dir / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)

    print_summary(rows, shards, total_samples, base_metadata)
    print(f"wrote DAgger dataset to {args.dataset_dir}")


def parse_device(value: str):
    if value == "auto":
        return None
    import torch

    return torch.device(value)


def copy_base_dataset(
    base_dataset_dir: Path,
    dataset_dir: Path,
    expected_signature: tuple[tuple[str, ...], tuple[int, ...], int],
) -> tuple[dict[str, Any], list[dict[str, Any]], int]:
    if base_dataset_dir.resolve() == dataset_dir.resolve():
        raise ValueError("--base-dataset-dir and --dataset-dir must be different")

    metadata_path = base_dataset_dir / "metadata.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"missing base dataset metadata: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as handle:
        metadata = json.load(handle)

    base_signature = (
        tuple(metadata.get("agents", ())),
        tuple(metadata.get("obs_shape", ())),
        int(metadata.get("action_dim", 0)),
    )
    if base_signature != expected_signature:
        raise ValueError(
            "base dataset signature does not match current scenarios: "
            f"{base_signature} != {expected_signature}. Regenerate the base "
            "dataset if observation features changed."
        )

    copied_shards: list[dict[str, Any]] = []
    total_samples = 0
    for index, shard in enumerate(metadata.get("shards", [])):
        source = base_dataset_dir / shard["file"]
        target_name = f"base_{index:06d}_{source.name}"
        shutil.copy2(source, dataset_dir / target_name)
        copied = dict(shard)
        copied["file"] = target_name
        copied["kind"] = "base_expert"
        copied["source_dataset"] = str(base_dataset_dir)
        copied_shards.append(copied)
        total_samples += int(copied.get("samples", 0))

    return metadata, copied_shards, total_samples


def load_policy_model(
    env: UTMMAPFEnv,
    model_dir: Path,
    model_cache: dict[tuple[int, tuple[int, ...]], DiscreteMAPPO],
    device: Any,
) -> DiscreteMAPPO:
    signature = (
        len(env.possible_agents),
        env.observation_space(env.possible_agents[0]).shape,
    )
    model = model_cache.get(signature)
    if model is None:
        model = DiscreteMAPPO.load(env, model_dir, device=device)
        model_cache[signature] = model
    else:
        model.env = env
    return model


def collect_recovery_rollout(
    env: UTMMAPFEnv,
    model: DiscreteMAPPO,
    rng: np.random.Generator,
    seed: int,
    stochastic: bool,
    epsilon: float,
    teacher_rollin_prob: float,
    only_disagreements: bool,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    observations, _ = env.reset(seed=seed)
    agent_to_index = {
        agent: index for index, agent in enumerate(env.possible_agents)
    }
    observation_rows: list[np.ndarray] = []
    mask_rows: list[np.ndarray] = []
    action_rows: list[int] = []
    time_rows: list[int] = []
    agent_rows: list[int] = []
    totals = {key: 0 for key in METRIC_KEYS}
    comparisons = 0
    disagreements = 0
    random_rollin_actions = 0
    teacher_rollin_actions = 0

    while observations:
        agents = sorted(observations)
        masks = {agent: env.action_mask(agent).copy() for agent in agents}
        teacher_actions = prioritized_shortest_path_actions(env, agents)
        student_actions = model.predict(
            observations,
            deterministic=not stochastic,
        )
        actions: dict[str, int] = {}

        for agent in agents:
            teacher_action = sanitize_action(
                int(teacher_actions.get(agent, int(UTMAction.WAIT))),
                masks[agent],
            )
            student_action = sanitize_action(
                int(student_actions.get(agent, int(UTMAction.WAIT))),
                masks[agent],
            )
            comparisons += 1
            disagreed = student_action != teacher_action
            if disagreed:
                disagreements += 1

            if disagreed or not only_disagreements:
                append_sample(
                    observations=observations,
                    masks=masks,
                    agent=agent,
                    action=teacher_action,
                    env=env,
                    agent_to_index=agent_to_index,
                    observation_rows=observation_rows,
                    mask_rows=mask_rows,
                    action_rows=action_rows,
                    time_rows=time_rows,
                    agent_rows=agent_rows,
                )

            rollin_action = student_action
            if rng.random() < epsilon:
                rollin_action = random_valid_action(masks[agent], rng)
                random_rollin_actions += 1
            if rng.random() < teacher_rollin_prob:
                rollin_action = teacher_action
                teacher_rollin_actions += 1
            actions[agent] = rollin_action

        before = {
            agent: env.render()["agents"][agent]["cell"] for agent in agents
        }
        observations, _, _, _, infos = env.step(actions)
        update_totals(totals, before, infos)

    metrics = final_metrics(env, totals)
    metrics["comparisons"] = comparisons
    metrics["disagreements"] = disagreements
    metrics["disagreement_rate"] = (
        float(disagreements / comparisons) if comparisons else 0.0
    )
    metrics["random_rollin_actions"] = random_rollin_actions
    metrics["teacher_rollin_actions"] = teacher_rollin_actions
    return (
        build_arrays(
            env,
            observation_rows,
            mask_rows,
            action_rows,
            time_rows,
            agent_rows,
        ),
        metrics,
    )


def sanitize_action(action: int, action_mask: np.ndarray) -> int:
    if 0 <= action < action_mask.shape[0] and bool(action_mask[action]):
        return action

    valid_actions = np.flatnonzero(action_mask)
    if valid_actions.size == 0:
        return int(UTMAction.WAIT)
    return int(valid_actions[0])


def random_valid_action(
    action_mask: np.ndarray,
    rng: np.random.Generator,
) -> int:
    valid_actions = np.flatnonzero(action_mask)
    if valid_actions.size == 0:
        return int(UTMAction.WAIT)
    return int(rng.choice(valid_actions))


def build_dagger_metadata(
    args: argparse.Namespace,
    rows: list[dict[str, Any]],
    shards: list[dict[str, Any]],
    total_samples: int,
    expected_signature: tuple[tuple[str, ...], tuple[int, ...], int],
    base_metadata: dict[str, Any] | None,
) -> dict[str, Any]:
    agents, obs_shape, action_dim = expected_signature
    success_flags = np.asarray(
        [bool(row["all_reached"]) for row in rows], dtype=np.float32
    )
    clean_flags = np.asarray(
        [bool(row.get("clean", False)) for row in rows], dtype=np.float32
    )
    comparisons = int(sum(int(row["comparisons"]) for row in rows))
    disagreements = int(sum(int(row["disagreements"]) for row in rows))
    base_samples = int(base_metadata.get("samples", 0)) if base_metadata else 0

    return {
        "version": 1,
        "source": "curriculum.collect_dagger_dataset",
        "dataset_type": "dagger_recovery",
        "label_teacher": "pibt",
        "rollin_policy_model_dir": str(args.policy_model_dir),
        "base_dataset_dir": (
            str(args.base_dataset_dir) if args.base_dataset_dir is not None else None
        ),
        "scenario_dir": str(args.scenario_dir),
        "scenario_dir_resolved": str(args.scenario_dir.resolve()),
        "success_only": False,
        "clean_success_only": False,
        "episodes_per_scenario": int(args.episodes_per_scenario),
        "stochastic": bool(args.stochastic),
        "epsilon": float(args.epsilon),
        "teacher_rollin_prob": float(args.teacher_rollin_prob),
        "only_disagreements": bool(args.only_disagreements),
        "scenario_count": len(rows),
        "stored_scenario_count": len(shards),
        "recovery_shard_count": sum(
            1 for shard in shards if shard.get("kind") == "dagger_recovery"
        ),
        "base_shard_count": sum(
            1 for shard in shards if shard.get("kind") == "base_expert"
        ),
        "samples": int(total_samples),
        "base_samples": base_samples,
        "recovery_samples": int(total_samples - base_samples),
        "agents": list(agents),
        "n_agents": len(agents),
        "obs_shape": list(obs_shape),
        "action_dim": int(action_dim),
        "rollin_success_rate": float(success_flags.mean()) if rows else 0.0,
        "rollin_clean_success_rate": float(clean_flags.mean()) if rows else 0.0,
        "rollin_mean_reached": mean(rows, "reached_count"),
        "rollin_mean_time_steps": mean(rows, "time_steps"),
        "rollin_mean_unsafe": mean(rows, "unsafe"),
        "rollin_mean_oscillations": mean(rows, "oscillations"),
        "comparisons": comparisons,
        "disagreements": disagreements,
        "disagreement_rate": float(disagreements / comparisons)
        if comparisons
        else 0.0,
        "shards": shards,
    }


def print_summary(
    rows: list[dict[str, Any]],
    shards: list[dict[str, Any]],
    total_samples: int,
    base_metadata: dict[str, Any] | None,
) -> None:
    comparisons = int(sum(int(row["comparisons"]) for row in rows))
    disagreements = int(sum(int(row["disagreements"]) for row in rows))
    base_samples = int(base_metadata.get("samples", 0)) if base_metadata else 0
    recovery_samples = total_samples - base_samples

    print("summary:")
    print(f"  rollouts={len(rows)}")
    print(f"  stored_shards={len(shards)}")
    print(f"  base_samples={base_samples}")
    print(f"  recovery_samples={recovery_samples}")
    print(f"  samples={total_samples}")
    print(f"  rollin_success_rate={mean_bool(rows, 'all_reached'):.3f}")
    print(f"  rollin_clean_success_rate={mean_bool(rows, 'clean'):.3f}")
    print(f"  rollin_mean_reached={mean(rows, 'reached_count'):.2f}")
    print(f"  rollin_mean_time_steps={mean(rows, 'time_steps'):.2f}")
    print(f"  rollin_mean_unsafe={mean(rows, 'unsafe'):.2f}")
    print(f"  rollin_mean_oscillations={mean(rows, 'oscillations'):.2f}")
    disagreement_rate = float(disagreements / comparisons) if comparisons else 0.0
    print(f"  disagreement_rate={disagreement_rate:.3f}")


if __name__ == "__main__":
    main()
