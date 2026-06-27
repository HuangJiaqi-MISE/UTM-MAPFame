from __future__ import annotations

from utm_mappo import UTMMAPFEnv
from utm_mappo.config import (
    GridConfig,
    MissionConfig,
    TemporalNoFlyZoneConfig,
    UTMScenario,
)
from utm_mappo.cbs_expert import cbs_plan
from utm_mappo.expert import _shortest_path_length, prioritized_pibt_action_plan
from utm_mappo.scenarios import crossing_scenario
from utm_mappo.space_time_expert import (
    action_from_transition,
    planned_cell,
    prioritized_space_time_plan,
)


def _small_cbs_scenario() -> UTMScenario:
    return UTMScenario(
        grid=GridConfig(dimensions=(3, 3, 1)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(2, 1, 0),
            ),
            MissionConfig(
                mission_id=2,
                start=(1, 0, 0),
                goal=(1, 2, 0),
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )


def _terminal_downwash_conflict_scenario() -> UTMScenario:
    return UTMScenario(
        grid=GridConfig(dimensions=(3, 3, 3)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 0, 2),
                goal=(1, 1, 2),
            ),
            MissionConfig(
                mission_id=2,
                start=(2, 2, 0),
                goal=(1, 1, 1),
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )


def _small_independent_scenario() -> UTMScenario:
    return UTMScenario(
        grid=GridConfig(dimensions=(5, 5, 2)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 0, 0),
                goal=(2, 0, 0),
            ),
            MissionConfig(
                mission_id=2,
                start=(4, 4, 0),
                goal=(4, 2, 0),
            ),
        ),
        max_time_steps=8,
        observation_radius=1,
    )


def _static_obstacle_detour_scenario() -> UTMScenario:
    return UTMScenario(
        grid=GridConfig(
            dimensions=(5, 3, 1),
            blocked_cells=((2, 1, 0),),
        ),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(4, 1, 0),
            ),
        ),
        max_time_steps=12,
        observation_radius=1,
    )


def _static_obstacle_unreachable_scenario() -> UTMScenario:
    return UTMScenario(
        grid=GridConfig(
            dimensions=(5, 3, 1),
            blocked_cells=((2, 0, 0), (2, 1, 0), (2, 2, 0)),
        ),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 1, 0),
                goal=(4, 1, 0),
            ),
        ),
        max_time_steps=12,
        observation_radius=1,
    )


def _temporal_no_fly_wait_scenario() -> UTMScenario:
    return UTMScenario(
        grid=GridConfig(dimensions=(3, 1, 1)),
        missions=(
            MissionConfig(
                mission_id=1,
                start=(0, 0, 0),
                goal=(2, 0, 0),
            ),
        ),
        no_fly_zones=(
            TemporalNoFlyZoneConfig(
                zone_id=1,
                min_cell=(1, 0, 0),
                max_cell=(1, 0, 0),
                start_time_step=1,
                end_time_step=1,
            ),
        ),
        max_time_steps=6,
        observation_radius=1,
    )


def test_space_time_plan_solves_crossing_without_unsafe_holds() -> None:
    env = UTMMAPFEnv(crossing_scenario())
    env.reset()
    plan = prioritized_space_time_plan(env)

    assert plan is not None

    observations, _ = env.reset()
    unsafe_holds = 0
    while observations:
        actions = {}
        for agent in observations:
            state = env._state_by_agent[agent]
            next_cell = planned_cell(plan[agent], env.time_step + 1)
            actions[agent] = action_from_transition(state.cell, next_cell)

        observations, _, _, _, infos = env.step(actions)
        unsafe_holds += sum(1 for info in infos.values() if info["unsafe_hold"])

    render_state = env.render()
    assert all(
        agent_state["reached_goal"]
        for agent_state in render_state["agents"].values()
    )
    assert unsafe_holds == 0


def test_cbs_plan_rejects_terminal_downwash_conflict_without_searching() -> None:
    env = UTMMAPFEnv(_terminal_downwash_conflict_scenario())
    env.reset()
    result = cbs_plan(env)

    assert result.plan is None
    assert result.reason == "terminal_downwash_conflict"
    assert result.expanded_nodes == 0
    assert result.low_level_searches == 0


def test_pibt_action_plan_replays_after_precompute() -> None:
    env = UTMMAPFEnv(_small_independent_scenario())
    action_plan = prioritized_pibt_action_plan(env)

    observations, _ = env.reset()
    while observations:
        actions = {
            agent: action_plan[agent][env.time_step]
            for agent in observations
        }
        observations, _, _, _, _ = env.step(actions)

    render_state = env.render()
    assert all(
        agent_state["reached_goal"]
        for agent_state in render_state["agents"].values()
    )


def test_pibt_action_plan_uses_static_obstacle_detour() -> None:
    env = UTMMAPFEnv(_static_obstacle_detour_scenario())
    action_plan = prioritized_pibt_action_plan(env)

    observations, _ = env.reset()
    while observations:
        actions = {
            agent: action_plan[agent][env.time_step]
            for agent in observations
        }
        observations, _, _, _, infos = env.step(actions)
        assert not any(info["invalid_action"] for info in infos.values())

    render_state = env.render()
    assert render_state["agents"]["agent_1"]["reached_goal"]


def test_pibt_astar_distance_accounts_for_temporal_no_fly_wait() -> None:
    env = UTMMAPFEnv(_temporal_no_fly_wait_scenario())
    env.reset()

    assert (
        _shortest_path_length(
            env,
            start=(0, 0, 0),
            goal=(2, 0, 0),
            start_time=0,
            distance_mode="static",
        )
        == 2
    )
    assert (
        _shortest_path_length(
            env,
            start=(0, 0, 0),
            goal=(2, 0, 0),
            start_time=0,
            distance_mode="astar",
            astar_max_expansions=1_000,
        )
        == 3
    )


def test_cbs_plan_rejects_static_unreachable_goal() -> None:
    env = UTMMAPFEnv(_static_obstacle_unreachable_scenario())
    env.reset()
    result = cbs_plan(env)

    assert result.plan is None
    assert result.reason == "initial_low_level_failed"


def test_cbs_plan_solves_small_vertex_conflict_without_unsafe_holds() -> None:
    env = UTMMAPFEnv(_small_cbs_scenario())
    env.reset()
    result = cbs_plan(env, max_high_level_nodes=2_000, max_seconds=5.0)

    assert result.plan is not None

    observations, _ = env.reset()
    unsafe_holds = 0
    while observations:
        actions = {}
        for agent in observations:
            state = env._state_by_agent[agent]
            next_cell = planned_cell(result.plan[agent], env.time_step + 1)
            actions[agent] = action_from_transition(state.cell, next_cell)

        observations, _, _, _, infos = env.step(actions)
        unsafe_holds += sum(1 for info in infos.values() if info["unsafe_hold"])

    render_state = env.render()
    assert all(
        agent_state["reached_goal"]
        for agent_state in render_state["agents"].values()
    )
    assert unsafe_holds == 0
