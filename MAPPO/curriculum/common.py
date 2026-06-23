from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Callable

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from utm_mappo import UTMMAPFEnv, UTMScenario  # noqa: E402
from utm_mappo.mappo import DiscreteMAPPO  # noqa: E402

PolicyFn = Callable[[dict[str, np.ndarray]], dict[str, int]]


def scenario_paths(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    paths = sorted(path.rglob("*.yaml"))
    if not paths:
        raise FileNotFoundError(f"no YAML scenarios found under {path}")
    return paths


def load_env(path: Path, **env_kwargs: Any) -> UTMMAPFEnv:
    return UTMMAPFEnv(UTMScenario.from_yaml(path), **env_kwargs)


def validate_env_compatibility(envs: list[UTMMAPFEnv]) -> None:
    if not envs:
        raise ValueError("at least one environment is required")

    first_agents = tuple(envs[0].possible_agents)
    first_obs_shape = envs[0].observation_space(first_agents[0]).shape
    for env in envs[1:]:
        agents = tuple(env.possible_agents)
        if agents != first_agents:
            raise ValueError(
                "multi-scenario training currently requires identical mission ids "
                f"per stage; got {agents}, expected {first_agents}"
            )
        obs_shape = env.observation_space(agents[0]).shape
        if obs_shape != first_obs_shape:
            raise ValueError(
                "multi-scenario training currently requires identical observation "
                f"shape per stage; got {obs_shape}, expected {first_obs_shape}"
            )


def rollout_metrics(
    env: UTMMAPFEnv,
    model: DiscreteMAPPO,
    stochastic: bool = False,
) -> dict[str, Any]:
    return rollout_metrics_with_policy(
        env,
        lambda observations: model.predict(
            observations, deterministic=not stochastic
        ),
    )


def rollout_metrics_with_policy(
    env: UTMMAPFEnv,
    policy: PolicyFn,
) -> dict[str, Any]:
    observations, _ = env.reset()
    totals = {
        "moves": 0,
        "waits": 0,
        "oscillations": 0,
        "invalid": 0,
        "no_fly": 0,
        "unsafe": 0,
    }

    while observations:
        before = {
            agent: env.render()["agents"][agent]["cell"] for agent in observations
        }
        actions = policy(observations)
        observations, _, _, _, infos = env.step(actions)

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

    render_state = env.render()
    reached = [
        bool(agent_state["reached_goal"])
        for agent_state in render_state["agents"].values()
    ]
    result = {
        "time_steps": int(render_state["time_step"]),
        "agents": len(reached),
        "all_reached": all(reached),
        "reached_count": int(np.sum(reached)),
    }
    result.update(totals)
    return result
