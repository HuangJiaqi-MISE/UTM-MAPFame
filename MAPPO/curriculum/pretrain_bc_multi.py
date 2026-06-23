from __future__ import annotations

import argparse
import random
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from tqdm import trange

from common import load_env, scenario_paths, validate_env_compatibility

from utm_mappo.expert import prioritized_shortest_path_actions  # noqa: E402
from utm_mappo.mappo import DiscreteMAPPO  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Behavior-clone the priority-aware expert across a scenario directory."
    )
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--init-model-dir", type=Path, default=None)
    parser.add_argument("--updates", type=int, default=20_000)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--freeze-actor-encoder", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rng = random.Random(args.seed)
    np.random.seed(args.seed)

    paths = scenario_paths(args.scenario_dir)
    envs = [load_env(path) for path in paths]
    validate_env_compatibility(envs)

    model = DiscreteMAPPO(
        envs[0],
        learning_rate=args.learning_rate,
        freeze_actor_encoder=args.freeze_actor_encoder,
    )
    if args.init_model_dir is not None:
        model.load_weights(args.init_model_dir)

    optimizer = model.optimizer
    observations_by_env = [None for _ in envs]
    progress = trange(args.updates, desc="multi-scenario bc")
    for _ in progress:
        index = rng.randrange(len(envs))
        env = envs[index]
        model.env = env

        observations = observations_by_env[index]
        if not observations:
            observations, _ = env.reset()

        agents = sorted(observations)
        masks = [env.action_mask(agent) for agent in agents]
        expert_actions = prioritized_shortest_path_actions(env, agents)
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
        loss = F.cross_entropy(dist.logits, target_actions)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        actions = {
            agent: int(action)
            for agent, action in zip(agents, target_actions.detach().cpu().numpy())
        }
        next_observations, _, _, _, _ = env.step(actions)
        observations_by_env[index] = next_observations if next_observations else None
        progress.set_postfix(loss=f"{loss.item():.3f}", scenario=paths[index].name)

    model.save(args.model_dir)
    print(f"saved behavior-cloned model to {args.model_dir}")


if __name__ == "__main__":
    main()
