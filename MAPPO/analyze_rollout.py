from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

import numpy as np
import torch

from utm_mappo import UTMMAPFEnv, UTMScenario
from utm_mappo.env import UTMAction
from utm_mappo.expert import prioritized_shortest_path_actions
from utm_mappo.mappo import DiscreteMAPPO


ACTION_NAMES = {
    int(UTMAction.WAIT): "WAIT",
    int(UTMAction.POS_X): "+X",
    int(UTMAction.NEG_X): "-X",
    int(UTMAction.POS_Y): "+Y",
    int(UTMAction.NEG_Y): "-Y",
    int(UTMAction.POS_Z): "+Z",
    int(UTMAction.NEG_Z): "-Z",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one deterministic rollout and print per-agent diagnostics."
    )
    parser.add_argument(
        "scenario",
        type=Path,
        nargs="?",
        default=Path("configs/crossing.yaml"),
        help="Path to a UTM scenario YAML file.",
    )
    parser.add_argument("--model-dir", type=Path, default=Path("models/crossing"))
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Sample actions instead of taking deterministic argmax actions.",
    )
    parser.add_argument(
        "--expert-only",
        action="store_true",
        help="Roll out the priority-aware expert instead of a trained model.",
    )
    return parser.parse_args()


def policy_debug(
    env: UTMMAPFEnv,
    model: DiscreteMAPPO,
    observations: dict[str, np.ndarray],
) -> dict[str, dict[str, object]]:
    if not observations:
        return {}

    agents = sorted(observations)
    obs_tensor = torch.as_tensor(
        np.stack([observations[agent] for agent in agents]),
        dtype=torch.float32,
        device=model.device,
    )
    mask_tensor = torch.as_tensor(
        np.stack([env.action_mask(agent) for agent in agents]),
        dtype=torch.bool,
        device=model.device,
    )
    with torch.no_grad():
        dist = model.model.action_distribution(obs_tensor, mask_tensor)
        probabilities = dist.probs.detach().cpu().numpy()

    debug: dict[str, dict[str, object]] = {}
    expert_actions = prioritized_shortest_path_actions(env, agents)
    for agent, probs in zip(agents, probabilities):
        policy_action = int(probs.argmax())
        expert_action = expert_actions[agent]
        debug[agent] = {
            "policy_action": ACTION_NAMES[policy_action],
            "expert_action": ACTION_NAMES[expert_action],
            "probabilities": {
                ACTION_NAMES[index]: round(float(probability), 3)
                for index, probability in enumerate(probs)
            },
        }
    return debug


def main() -> None:
    args = parse_args()
    scenario = UTMScenario.from_yaml(args.scenario)
    env = UTMMAPFEnv(scenario)
    model = None if args.expert_only else DiscreteMAPPO.load(env, args.model_dir)

    observations, _ = env.reset()
    first_step_expert_actions = prioritized_shortest_path_actions(
        env, sorted(observations)
    )
    first_step_debug = (
        {} if model is None else policy_debug(env, model, observations)
    )
    diagnostics = {
        agent: Counter(
            {
                "moves": 0,
                "waits": 0,
                "oscillations": 0,
                "invalid_actions": 0,
                "no_fly_holds": 0,
                "unsafe_holds": 0,
                "action_wait": 0,
                "action_pos_x": 0,
                "action_neg_x": 0,
                "action_pos_y": 0,
                "action_neg_y": 0,
                "action_pos_z": 0,
                "action_neg_z": 0,
            }
        )
        for agent in env.possible_agents
    }

    while observations:
        before = {
            agent: env.render()["agents"][agent]["cell"] for agent in observations
        }
        if model is None:
            actions = prioritized_shortest_path_actions(env, sorted(observations))
        else:
            actions = model.predict(observations, deterministic=not args.stochastic)
        for agent, action in actions.items():
            diagnostics[agent][f"action_{ACTION_NAMES[action].lower().replace('+', 'pos_').replace('-', 'neg_')}"] += 1
        observations, _, _, _, infos = env.step(actions)

        for agent, info in infos.items():
            if info["cell"] == before[agent]:
                diagnostics[agent]["waits"] += 1
            else:
                diagnostics[agent]["moves"] += 1
            if info["oscillated"]:
                diagnostics[agent]["oscillations"] += 1
            if info["invalid_action"]:
                diagnostics[agent]["invalid_actions"] += 1
            if info["no_fly_hold"]:
                diagnostics[agent]["no_fly_holds"] += 1
            if info["unsafe_hold"]:
                diagnostics[agent]["unsafe_holds"] += 1

    render_state = env.render()
    print(f"time_steps={render_state['time_step']}")
    print("first_step_policy:")
    for agent in env.possible_agents:
        if model is None:
            expert_action = first_step_expert_actions.get(agent, int(UTMAction.WAIT))
            print(f"  {agent}: expert={ACTION_NAMES[expert_action]}")
            continue

        debug = first_step_debug.get(agent)
        if debug is None:
            continue
        print(
            f"  {agent}: policy={debug['policy_action']} "
            f"expert={debug['expert_action']} probs={debug['probabilities']}"
        )
    for agent in env.possible_agents:
        state = render_state["agents"][agent]
        counts = diagnostics[agent]
        print(
            " ".join(
                [
                    f"{agent}:",
                    f"cell={state['cell']}",
                    f"goal={state['goal']}",
                    f"reached={state['reached_goal']}",
                    f"moves={counts['moves']}",
                    f"waits={counts['waits']}",
                    f"oscillations={counts['oscillations']}",
                    f"invalid={counts['invalid_actions']}",
                    f"no_fly={counts['no_fly_holds']}",
                    f"unsafe={counts['unsafe_holds']}",
                    "actions={"
                    f"WAIT:{counts['action_wait']},"
                    f"+X:{counts['action_pos_x']},"
                    f"-X:{counts['action_neg_x']},"
                    f"+Y:{counts['action_pos_y']},"
                    f"-Y:{counts['action_neg_y']},"
                    f"+Z:{counts['action_pos_z']},"
                    f"-Z:{counts['action_neg_z']}"
                    "}",
                ]
            )
        )


if __name__ == "__main__":
    main()
