from __future__ import annotations

from utm_mappo import UTMMAPFEnv, UTMAction
from utm_mappo.config import GridConfig, MissionConfig, UTMScenario
from utm_mappo.geometry import body_box, transition_conflict
from utm_mappo.scenarios import crossing_scenario


def test_env_reaches_goal_with_direct_actions() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(5, 3, 2)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(2, 1, 0),
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


def test_vertex_conflict_yields_higher_mission_id() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(5, 3, 2)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(4, 1, 0),
            ),
            MissionConfig(
                mission_id=2,
                start=(2, 1, 0),
                goal=(4, 2, 0),
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
    assert infos["agent_2"]["conflict_reason"] == "vertex"
    assert env.render()["agents"]["agent_1"]["cell"] == (1, 1, 0)
    assert env.render()["agents"]["agent_2"]["cell"] == (2, 1, 0)


def test_adjacent_robot_cells_are_not_standalone_conflicts() -> None:
    mission_a = MissionConfig(
        mission_id=1,
        start=(1, 1, 0),
        goal=(3, 1, 0),
    )
    mission_b = MissionConfig(
        mission_id=2,
        start=(2, 1, 0),
        goal=(1, 1, 0),
    )

    assert (
        transition_conflict(
            mission_a.start,
            mission_a.start,
            mission_a,
            mission_b.start,
            mission_b.start,
            mission_b,
        )
        is None
    )


def test_robot_body_is_single_grid_cell() -> None:
    mission = MissionConfig(
        mission_id=1,
        start=(5, 5, 5),
        goal=(6, 5, 5),
    )

    box = body_box(mission.start, mission)

    assert box.min_cell == (5, 5, 5)
    assert box.max_cell == (5, 5, 5)


def test_downwash_conflict_uses_fixed_single_cell_body() -> None:
    upper = MissionConfig(
        mission_id=1,
        start=(2, 2, 2),
        goal=(3, 2, 2),
    )
    lower = MissionConfig(
        mission_id=2,
        start=(2, 2, 1),
        goal=(3, 2, 1),
    )
    lateral = MissionConfig(
        mission_id=3,
        start=(3, 2, 1),
        goal=(4, 2, 1),
    )

    assert (
        transition_conflict(
            upper.start,
            upper.start,
            upper,
            lower.start,
            lower.start,
            lower,
        )
        == "downwash"
    )
    assert (
        transition_conflict(
            upper.start,
            upper.start,
            upper,
            lateral.start,
            lateral.start,
            lateral,
        )
        is None
    )


def test_action_mask_rejects_static_invalid_moves() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(3, 3, 2), blocked_cells=((1, 1, 0),)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(2, 1, 0),
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


def test_wait_streak_penalty_increases_with_consecutive_waits() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(3, 3, 1)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(2, 1, 0),
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )
    env = UTMMAPFEnv(
        scenario,
        step_penalty=0.0,
        wait_penalty=0.0,
        progress_reward_scale=0.0,
        wait_streak_penalty_scale=-0.1,
    )
    env.reset()

    _, rewards, _, _, infos = env.step({"agent_1": UTMAction.WAIT})
    assert infos["agent_1"]["wait_streak"] == 1
    assert rewards["agent_1"] == -0.1

    _, rewards, _, _, infos = env.step({"agent_1": UTMAction.WAIT})
    assert infos["agent_1"]["wait_streak"] == 2
    assert rewards["agent_1"] == -0.2


def test_stationary_agent_blocking_downwash_conflict_is_penalized() -> None:
    scenario = UTMScenario(
        grid=GridConfig(dimensions=(3, 3, 3)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(1, 1, 1),
                goal=(0, 1, 1),
            ),
            MissionConfig(
                mission_id=2,
                start=(2, 1, 2),
                goal=(0, 1, 2),
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )
    env = UTMMAPFEnv(
        scenario,
        step_penalty=0.0,
        wait_penalty=0.0,
        progress_reward_scale=0.0,
        unsafe_hold_penalty=-2.0,
        blocking_penalty=-1.25,
    )
    env.reset()

    _, rewards, _, _, infos = env.step(
        {"agent_1": UTMAction.WAIT, "agent_2": UTMAction.NEG_X}
    )

    assert infos["agent_1"]["blocking_hold"]
    assert infos["agent_1"]["blocking_reason"] == "downwash"
    assert not infos["agent_1"]["unsafe_hold"]
    assert rewards["agent_1"] == -1.25
    assert infos["agent_2"]["unsafe_hold"]
    assert infos["agent_2"]["conflict_reason"] == "downwash"


def test_crossing_observation_includes_priority_history_features() -> None:
    env = UTMMAPFEnv(crossing_scenario())
    observations, _ = env.reset()

    assert env.observation_space("agent_1").shape == (645,)
    assert observations["agent_1"].shape == (645,)
