from __future__ import annotations

import argparse
import random
from pathlib import Path

import numpy as np
from tqdm import trange

from common import load_env, rollout_metrics, scenario_paths, validate_env_compatibility

from utm_mappo.mappo import DiscreteMAPPO  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train MAPPO by sampling a directory of compatible scenarios."
    )
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--init-model-dir", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--rollout-steps", type=int, default=1024)
    parser.add_argument("--batch-size", type=int, default=1024)
    parser.add_argument("--epochs", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=5e-5)
    parser.add_argument("--entropy-coef", type=float, default=0.001)
    parser.add_argument("--bc-anchor-coef", type=float, default=0.05)
    parser.add_argument("--freeze-actor-encoder", action="store_true")
    parser.add_argument("--eval-interval", type=int, default=0)
    parser.add_argument("--eval-episodes", type=int, default=8)
    parser.add_argument("--save-interval", type=int, default=0)
    parser.add_argument("--goal-reward", type=float, default=25.0)
    parser.add_argument("--progress-reward-scale", type=float, default=1.0)
    parser.add_argument("--step-penalty", type=float, default=-0.02)
    parser.add_argument("--wait-penalty", type=float, default=-0.02)
    parser.add_argument("--oscillation-penalty", type=float, default=-0.2)
    parser.add_argument("--unsafe-hold-penalty", type=float, default=-3.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rng = random.Random(args.seed)
    np.random.seed(args.seed)

    paths = scenario_paths(args.scenario_dir)
    env_kwargs = {
        "goal_reward": args.goal_reward,
        "progress_reward_scale": args.progress_reward_scale,
        "step_penalty": args.step_penalty,
        "wait_penalty": args.wait_penalty,
        "oscillation_penalty": args.oscillation_penalty,
        "unsafe_hold_penalty": args.unsafe_hold_penalty,
    }
    envs = [load_env(path, **env_kwargs) for path in paths]
    validate_env_compatibility(envs)

    model = DiscreteMAPPO(
        envs[0],
        learning_rate=args.learning_rate,
        rollout_steps=args.rollout_steps,
        batch_size=args.batch_size,
        update_epochs=args.epochs,
        entropy_coef=args.entropy_coef,
        bc_anchor_coef=args.bc_anchor_coef,
        freeze_actor_encoder=args.freeze_actor_encoder,
    )
    if args.init_model_dir is not None:
        model.load_weights(args.init_model_dir)

    iterations = max(1, args.timesteps // args.rollout_steps)
    progress = trange(iterations, desc="multi-scenario training")
    for iteration in progress:
        index = rng.randrange(len(envs))
        model.env = envs[index]
        rollout = model.collect_rollout()
        train = model.update()
        progress.set_postfix(
            scenario=paths[index].name,
            reward=f"{rollout['mean_episode_reward']:.3f}",
            loss=f"{train['loss']:.3f}",
            entropy=f"{train['entropy']:.3f}",
        )

        iteration_number = iteration + 1
        if args.eval_interval > 0 and iteration_number % args.eval_interval == 0:
            metrics = evaluate_sample(model, envs, args.eval_episodes, rng)
            progress.write(
                "eval "
                f"success={metrics['success_rate']:.3f} "
                f"unsafe={metrics['mean_unsafe']:.2f} "
                f"time={metrics['mean_time_steps']:.1f}"
            )

        if args.save_interval > 0 and iteration_number % args.save_interval == 0:
            model.save(args.model_dir / f"checkpoint_{iteration_number:05d}")

    model.save(args.model_dir)
    print(f"saved model to {args.model_dir}")


def evaluate_sample(
    model: DiscreteMAPPO,
    envs: list,
    episodes: int,
    rng: random.Random,
) -> dict[str, float]:
    results = []
    for _ in range(max(1, episodes)):
        env = envs[rng.randrange(len(envs))]
        model.env = env
        results.append(rollout_metrics(env, model))

    return {
        "success_rate": float(np.mean([item["all_reached"] for item in results])),
        "mean_unsafe": float(np.mean([item["unsafe"] for item in results])),
        "mean_time_steps": float(np.mean([item["time_steps"] for item in results])),
    }


if __name__ == "__main__":
    main()
