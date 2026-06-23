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

    active_set = set(agents)
    active_agents = sorted(
        agents,
        key=lambda name: _dynamic_priority_key(env, name, len(active_set)),
    )
    proposals = {
        agent: state.cell
        for agent, state in env._state_by_agent.items()
        if state.reached_goal
    }
    actions: dict[str, int] = {}
    current_occupants = {
        state.cell: agent for agent, state in env._state_by_agent.items()
    }
    planning_stack: set[str] = set()

    def plan_agent(agent: str) -> bool:
        if agent in actions:
            return True
        if agent in planning_stack:
            return False

        state = env._state_by_agent[agent]
        planning_stack.add(agent)

        for action in ranked_shortest_path_actions(env, agent):
            candidate = add_cell(state.cell, ACTION_DELTAS[action])
            blocker = current_occupants.get(candidate)
            if blocker == agent:
                blocker = None
            if blocker is not None and blocker not in active_set:
                continue

            if _conflicts_with_committed(env, agent, candidate, proposals):
                continue

            action_snapshot = dict(actions)
            proposal_snapshot = dict(proposals)
            actions[agent] = action
            proposals[agent] = candidate

            if blocker is not None and blocker not in actions:
                if blocker in planning_stack or not plan_agent(blocker):
                    actions.clear()
                    actions.update(action_snapshot)
                    proposals.clear()
                    proposals.update(proposal_snapshot)
                    continue

            planning_stack.remove(agent)
            return True

        planning_stack.remove(agent)
        return False

    for agent in active_agents:
        if plan_agent(agent):
            continue
        state = env._state_by_agent[agent]
        actions[agent] = int(UTMAction.WAIT)
        proposals[agent] = state.cell

    return actions


def _dynamic_priority_key(
    env: UTMMAPFEnv, agent: str, active_count: int
) -> tuple[int, int, int, int]:
    state = env._state_by_agent[agent]
    mission_id = state.mission.mission_id
    active_count = max(1, active_count)
    round_robin_rank = (mission_id - 1 - env.time_step) % active_count
    distance = _shortest_path_length(
        env,
        start=state.cell,
        goal=state.mission.goal,
        start_time=env.time_step,
    )
    if distance is None:
        distance = 10_000

    return (
        -_wait_streak(state.path),
        round_robin_rank,
        -distance,
        mission_id,
    )


def _wait_streak(path: list[Cell]) -> int:
    if len(path) < 2:
        return 0

    streak = 0
    cursor = len(path) - 1
    while cursor > 0 and path[cursor] == path[cursor - 1]:
        streak += 1
        cursor -= 1
    return streak


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
    cache = _distance_cache(env)
    cache_key = (start, goal, start_time)
    if cache_key in cache:
        return cache[cache_key]

    if start == goal:
        cache[cache_key] = 0
        return 0

    horizon = env.scenario.max_time_steps
    queue: deque[tuple[Cell, int, int]] = deque([(start, start_time, 0)])
    visited: set[tuple[Cell, int]] = {(start, start_time)}

    while queue:
        cell, time_step, distance = queue.popleft()
        if cell == goal:
            cache[cache_key] = distance
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

    cache[cache_key] = None
    return None


def _distance_cache(env: UTMMAPFEnv) -> dict[tuple[Cell, Cell, int], int | None]:
    cache = getattr(env, "_expert_distance_cache", None)
    if cache is None:
        cache = {}
        setattr(env, "_expert_distance_cache", cache)
    return cache


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
