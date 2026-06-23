from __future__ import annotations

from utm_mappo import UTMMAPFEnv
from utm_mappo.cbs_expert import cbs_plan
from utm_mappo.scenarios import crossing_scenario
from utm_mappo.space_time_expert import (
    action_from_transition,
    planned_cell,
    prioritized_space_time_plan,
)


def test_cbs_plan_solves_crossing_without_unsafe_holds() -> None:
    env = UTMMAPFEnv(crossing_scenario())
    env.reset()
    result = cbs_plan(env)

    assert result.plan is not None
    assert_plan_executes_without_unsafe_holds(result.plan)


def test_space_time_plan_solves_crossing_without_unsafe_holds() -> None:
    env = UTMMAPFEnv(crossing_scenario())
    env.reset()
    plan = prioritized_space_time_plan(env)

    assert plan is not None
    assert_plan_executes_without_unsafe_holds(plan)


def assert_plan_executes_without_unsafe_holds(
    plan: dict[str, list[tuple[int, int, int]]]
) -> None:
    env = UTMMAPFEnv(crossing_scenario())
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
