from __future__ import annotations

import argparse
from pathlib import Path

from utm_mappo import UTMMAPFEnv, UTMScenario
from utm_mappo.mappo import DiscreteMAPPO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train UTM discrete MAPPO policy.")
    parser.add_argument(
        "scenario",
        type=Path,
        nargs="?",
        default=Path("configs/crossing.yaml"),
        help="Path to a UTM scenario YAML file.",
    )
    parser.add_argument("--model-dir", type=Path, default=Path("models/crossing"))
    parser.add_argument("--init-model-dir", type=Path, default=None)
    parser.add_argument("--timesteps", type=int, default=200_000)
    parser.add_argument("--rollout-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--epochs", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--entropy-coef", type=float, default=0.005)
    parser.add_argument("--bc-anchor-coef", type=float, default=0.05)
    parser.add_argument("--eval-interval", type=int, default=10)
    parser.add_argument("--goal-reward", type=float, default=25.0)
    parser.add_argument("--progress-reward-scale", type=float, default=1.0)
    parser.add_argument("--step-penalty", type=float, default=-0.02)
    parser.add_argument("--wait-penalty", type=float, default=-0.02)
    parser.add_argument("--oscillation-penalty", type=float, default=-0.2)
    parser.add_argument("--unsafe-hold-penalty", type=float, default=-3.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scenario = UTMScenario.from_yaml(args.scenario)
    env = UTMMAPFEnv(
        scenario,
        goal_reward=args.goal_reward,
        progress_reward_scale=args.progress_reward_scale,
        step_penalty=args.step_penalty,
        wait_penalty=args.wait_penalty,
        oscillation_penalty=args.oscillation_penalty,
        unsafe_hold_penalty=args.unsafe_hold_penalty,
    )
    model = DiscreteMAPPO(
        env,
        learning_rate=args.learning_rate,
        rollout_steps=args.rollout_steps,
        batch_size=args.batch_size,
        update_epochs=args.epochs,
        entropy_coef=args.entropy_coef,
        bc_anchor_coef=args.bc_anchor_coef,
    )
    if args.init_model_dir is not None:
        model.load_weights(args.init_model_dir)
    model.learn(total_timesteps=args.timesteps, eval_interval=args.eval_interval)
    model.save(args.model_dir)
    print(f"saved model to {args.model_dir}")


if __name__ == "__main__":
    main()
