from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from tqdm import trange

from utm_mappo import UTMMAPFEnv, UTMScenario
from utm_mappo.expert import (
    DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS,
    prioritized_shortest_path_actions,
)
from utm_mappo.mappo import DiscreteMAPPO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Behavior-clone a shortest-path expert.")
    parser.add_argument(
        "scenario",
        type=Path,
        nargs="?",
        default=Path("configs/crossing.yaml"),
    )
    parser.add_argument("--model-dir", type=Path, default=Path("models/crossing_bc"))
    parser.add_argument("--updates", type=int, default=3000)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument(
        "--pibt-distance-mode",
        choices=("static", "astar"),
        default="static",
        help="Distance heuristic used by the online PIBT teacher.",
    )
    parser.add_argument(
        "--pibt-astar-max-expansions",
        type=int,
        default=DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS,
        help="Per-query expansion budget when --pibt-distance-mode astar.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scenario = UTMScenario.from_yaml(args.scenario)
    env = UTMMAPFEnv(scenario)
    model = DiscreteMAPPO(env, learning_rate=args.learning_rate)
    optimizer = model.optimizer

    observations, _ = env.reset()
    progress = trange(args.updates, desc="bc")
    for _ in progress:
        if not observations:
            observations, _ = env.reset()

        agents = sorted(observations)
        masks = [env.action_mask(agent) for agent in agents]
        expert_actions = prioritized_shortest_path_actions(
            env,
            agents,
            distance_mode=args.pibt_distance_mode,
            astar_max_expansions=args.pibt_astar_max_expansions,
        )
        target_list = []
        for agent, mask in zip(agents, masks):
            action = expert_actions[agent]
            if not mask[action]:
                action = 0
            target_list.append(action)

        obs_tensor = torch.as_tensor(
            np.stack([observations[agent] for agent in agents]),
            dtype=torch.float32,
            device=model.device,
        )
        mask_tensor = torch.as_tensor(
            np.stack(masks),
            dtype=torch.bool,
            device=model.device,
        )
        target_actions = torch.as_tensor(
            target_list,
            dtype=torch.long,
            device=model.device,
        )

        dist = model.model.action_distribution(obs_tensor, mask_tensor)
        logits = dist.logits
        loss = F.cross_entropy(logits, target_actions)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        actions = {
            agent: int(action)
            for agent, action in zip(agents, target_actions.detach().cpu().numpy())
        }
        observations, _, _, _, _ = env.step(actions)
        progress.set_postfix(loss=f"{loss.item():.3f}")

    model.save(args.model_dir)
    print(f"saved behavior-cloned model to {args.model_dir}")


if __name__ == "__main__":
    main()
