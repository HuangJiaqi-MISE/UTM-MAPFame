from __future__ import annotations

from utm_mappo import UTMMAPFEnv
from utm_mappo.mappo import DiscreteMAPPO
from utm_mappo.scenarios import crossing_scenario


def main() -> None:
    env = UTMMAPFEnv(crossing_scenario())
    model = DiscreteMAPPO(env, rollout_steps=32, batch_size=32, update_epochs=1)
    reward = model.evaluate(num_episodes=1)
    print(f"untrained deterministic reward: {reward:.3f}")


if __name__ == "__main__":
    main()
