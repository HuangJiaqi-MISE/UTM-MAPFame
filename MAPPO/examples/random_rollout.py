from __future__ import annotations

from utm_mappo import UTMMAPFEnv
from utm_mappo.scenarios import crossing_scenario


def main() -> None:
    env = UTMMAPFEnv(crossing_scenario())
    observations, _ = env.reset(seed=7)
    total_rewards = {agent: 0.0 for agent in env.possible_agents}

    while env.agents:
        actions = {
            agent: env.action_space(agent).sample()
            for agent in env.agents
            if agent in observations
        }
        observations, rewards, terminations, truncations, infos = env.step(actions)
        for agent, reward in rewards.items():
            total_rewards[agent] += reward
        if all(terminations.get(agent, False) or truncations.get(agent, False) for agent in rewards):
            break

    print(env.render())
    print(total_rewards)


if __name__ == "__main__":
    main()
