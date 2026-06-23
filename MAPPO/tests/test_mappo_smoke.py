from __future__ import annotations

import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("tqdm")

from utm_mappo import UTMMAPFEnv
from utm_mappo.mappo import DiscreteMAPPO
from utm_mappo.scenarios import crossing_scenario


def test_discrete_mappo_collect_and_update_smoke() -> None:
    env = UTMMAPFEnv(crossing_scenario())
    model = DiscreteMAPPO(
        env,
        rollout_steps=8,
        batch_size=8,
        update_epochs=1,
        hidden_dim=32,
        features_dim=32,
        device=torch.device("cpu"),
    )

    rollout_metrics = model.collect_rollout()
    train_metrics = model.update()

    assert "mean_episode_reward" in rollout_metrics
    assert train_metrics["loss"] == pytest.approx(train_metrics["loss"])


def test_discrete_mappo_save_and_load(tmp_path) -> None:
    env = UTMMAPFEnv(crossing_scenario())
    model = DiscreteMAPPO(
        env,
        rollout_steps=8,
        batch_size=8,
        update_epochs=1,
        hidden_dim=32,
        features_dim=32,
        device=torch.device("cpu"),
    )
    model.save(tmp_path)

    loaded = DiscreteMAPPO.load(env, tmp_path, device=torch.device("cpu"))
    observations, _ = env.reset()
    actions = loaded.predict(observations)

    assert set(actions).issubset(set(env.possible_agents))
