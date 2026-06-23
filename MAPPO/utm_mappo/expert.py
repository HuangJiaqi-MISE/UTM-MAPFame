from __future__ import annotations

from collections import deque

from .config import Cell
from .env import ACTION_DELTAS, UTMAction, UTMMAPFEnv
from .geometry import add_cell, transition_conflict


MOVING_ACTIONS = (
    UTMAction.POS_X,
    UTMAction.NEG_X,
    UTMAction.POS_Y,
    UTMAction.NEG_Y,
    UTMAction.POS_Z,
    UTMAction.NEG_Z,
)


def shortest_path_action(env: UTMMAPFEnv, agent: str) -> int:
    actions = ranked_shortest_path_actions(env, agent)
    return actions[0] if actions else int(UTMAction.WAIT)


def prioritized_shortest_path_actions(
    env: UTMMAPFEnv, agents: list[str] | tuple[str, ...] | None = None
) -> dict[str, int]:
    if agents is None:
        agents = tuple(env.agents)

    active_agents = sorted(
        agents,
        key=lambda name: env._state_by_agent[name].mission.mission_id,
    )
    proposals = {
        agent: state.cell
        for agent, state in env._state_by_agent.items()
        if state.reached_goal
    }
    actions: dict[str, int] = {}

    for agent in active_agents:
        state = env._state_by_agent[agent]
        chosen_action = int(UTMAction.WAIT)
        chosen_cell = state.cell

        for action in ranked_shortest_path_actions(env, agent):
            candidate = add_cell(state.cell, ACTION_DELTAS[action])
            if _conflicts_with_committed(env, agent, candidate, proposals):
                continue

            chosen_action = action
            chosen_cell = candidate
            break

        actions[agent] = chosen_action
        proposals[agent] = chosen_cell

    return actions


def ranked_shortest_path_actions(env: UTMMAPFEnv, agent: str) -> list[int]:
    state = env._state_by_agent[agent]
    if state.cell == state.mission.goal:
        return [int(UTMAction.WAIT)]

    action_mask = env.action_mask(agent)
    scored_actions: list[tuple[int, int, int, int, int]] = []
    action_order = {int(action): index for index, action in enumerate(MOVING_ACTIONS)}

    for action in MOVING_ACTIONS:
        action_int = int(action)
        if not action_mask[action_int]:
            continue

        candidate = add_cell(state.cell, ACTION_DELTAS[action])
        remaining = _shortest_path_length(
            env,
            start=candidate,
            goal=state.mission.goal,
            start_time=env.time_step + 1,
        )
        if remaining is None:
            continue

        oscillation_cost = (
            1 if candidate == state.previous_cell and candidate != state.cell else 0
        )
        scored_actions.append(
            (
                remaining,
                oscillation_cost,
                0,
                action_order[action_int],
                action_int,
            )
        )

    scored_actions.sort()
    actions = [item[-1] for item in scored_actions]
    if action_mask[int(UTMAction.WAIT)]:
        actions.append(int(UTMAction.WAIT))
    return actions or [int(UTMAction.WAIT)]


def _shortest_path_length(
    env: UTMMAPFEnv, start: Cell, goal: Cell, start_time: int
) -> int | None:
    if start == goal:
        return 0

    horizon = env.scenario.max_time_steps
    queue: deque[tuple[Cell, int, int]] = deque([(start, start_time, 0)])
    visited: set[tuple[Cell, int]] = {(start, start_time)}

    while queue:
        cell, time_step, distance = queue.popleft()
        if cell == goal:
            return distance

        if time_step >= horizon:
            continue

        for delta in ACTION_DELTAS.values():
            candidate = add_cell(cell, delta)
            next_time = time_step + 1
            visit_key = (candidate, next_time)
            if visit_key in visited:
                continue
            if not env.is_free_static(candidate):
                continue
            if env.is_no_fly(candidate, next_time):
                continue

            visited.add(visit_key)
            queue.append((candidate, next_time, distance + 1))

    return None


def _conflicts_with_committed(
    env: UTMMAPFEnv, agent: str, candidate: Cell, proposals: dict[str, Cell]
) -> bool:
    state = env._state_by_agent[agent]

    for other_agent, other_candidate in proposals.items():
        other_state = env._state_by_agent[other_agent]
        reason = transition_conflict(
            state.cell,
            candidate,
            state.mission,
            other_state.cell,
            other_candidate,
            other_state.mission,
        )
        if reason is not None:
            return True

    return False
