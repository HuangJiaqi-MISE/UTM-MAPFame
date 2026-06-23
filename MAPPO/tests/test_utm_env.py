from __future__ import annotations

from utm_mappo import UTMMAPFEnv, UTMAction
from utm_mappo.config import GridConfig, MissionConfig, UTMScenario


def test_env_reaches_goal_with_direct_actions() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(5, 3, 2)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(2, 1, 0),
                protection_xy_radius=0,
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )
    env = UTMMAPFEnv(scenario)
    observations, _ = env.reset()
    assert "agent_1" in observations

    env.step({"agent_1": UTMAction.POS_X})
    _, rewards, terminations, _, _ = env.step({"agent_1": UTMAction.POS_X})

    assert terminations["agent_1"]
    assert rewards["agent_1"] > 0
    assert env.render()["agents"]["agent_1"]["cell"] == (2, 1, 0)


def test_protection_conflict_yields_higher_mission_id() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(5, 3, 2)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(4, 1, 0),
                protection_xy_radius=0,
            ),
            MissionConfig(
                mission_id=2,
                start=(2, 1, 0),
                goal=(4, 2, 0),
                protection_xy_radius=0,
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )
    env = UTMMAPFEnv(scenario)
    env.reset()
    _, _, _, _, infos = env.step(
        {"agent_1": UTMAction.POS_X, "agent_2": UTMAction.NEG_X}
    )

    assert not infos["agent_1"]["unsafe_hold"]
    assert infos["agent_2"]["unsafe_hold"]
    assert env.render()["agents"]["agent_1"]["cell"] == (1, 1, 0)
    assert env.render()["agents"]["agent_2"]["cell"] == (2, 1, 0)


def test_action_mask_rejects_static_invalid_moves() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(3, 3, 2), blocked_cells=((1, 1, 0),)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(2, 1, 0),
                protection_xy_radius=0,
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )
    env = UTMMAPFEnv(scenario)
    env.reset()
    mask = env.action_mask("agent_1")

    assert mask[UTMAction.WAIT]
    assert not mask[UTMAction.POS_X]
    assert not mask[UTMAction.NEG_X]
