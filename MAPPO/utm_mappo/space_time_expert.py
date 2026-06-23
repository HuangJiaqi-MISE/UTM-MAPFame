from __future__ import annotations

import heapq
import random
from collections.abc import Iterable, Sequence

from .config import Cell
from .env import ACTION_DELTAS, UTMAction, UTMMAPFEnv
from .geometry import add_cell, manhattan_distance, transition_conflict


SpaceTimePlan = dict[str, list[Cell]]


def prioritized_space_time_plan(
    env: UTMMAPFEnv,
    max_retries: int = 32,
    seed: int = 1,
) -> SpaceTimePlan | None:
    """Build offline demonstrations with prioritized space-time A*.

    This planner is intended as a stronger offline teacher for behavior cloning.
    It is still prioritized planning, not a complete MAPF solver, but it searches
    full time-expanded paths and reserves earlier agents' transitions.
    """

    if not env._state_by_agent:
        env.reset()

    orders = _priority_orders(env, max_retries=max_retries, seed=seed)
    for order in orders:
        plan = _plan_for_order(env, order)
        if plan is not None:
            return plan
    return None


def action_from_transition(current: Cell, next_cell: Cell) -> int:
    delta = (
        next_cell[0] - current[0],
        next_cell[1] - current[1],
        next_cell[2] - current[2],
    )
    for action, action_delta in ACTION_DELTAS.items():
        if action_delta == delta:
            return int(action)
    return int(UTMAction.WAIT)


def planned_cell(path: Sequence[Cell], time_step: int) -> Cell:
    if time_step < len(path):
        return path[time_step]
    return path[-1]


def _priority_orders(
    env: UTMMAPFEnv,
    max_retries: int,
    seed: int,
) -> list[tuple[str, ...]]:
    agents = tuple(env.possible_agents)
    rng = random.Random(seed)
    orders: list[tuple[str, ...]] = []

    def add(order: Iterable[str]) -> None:
        item = tuple(order)
        if item not in orders:
            orders.append(item)

    add(agents)
    add(reversed(agents))
    add(
        sorted(
            agents,
            key=lambda agent: _mission_distance(env, agent),
            reverse=True,
        )
    )
    add(sorted(agents, key=lambda agent: _mission_distance(env, agent)))
    add(
        sorted(
            agents,
            key=lambda agent: (
                env._state_by_agent[agent].mission.start[0],
                env._state_by_agent[agent].mission.start[1],
                env._state_by_agent[agent].mission.start[2],
            ),
        )
    )
    add(
        sorted(
            agents,
            key=lambda agent: (
                env._state_by_agent[agent].mission.start[1],
                env._state_by_agent[agent].mission.start[0],
                env._state_by_agent[agent].mission.start[2],
            ),
        )
    )

    attempts = max(0, max_retries - len(orders))
    for _ in range(attempts):
        shuffled = list(agents)
        rng.shuffle(shuffled)
        add(shuffled)

    return orders[: max(1, max_retries)]


def _mission_distance(env: UTMMAPFEnv, agent: str) -> int:
    state = env._state_by_agent[agent]
    return manhattan_distance(state.mission.start, state.mission.goal)


def _plan_for_order(env: UTMMAPFEnv, order: Sequence[str]) -> SpaceTimePlan | None:
    planned: SpaceTimePlan = {}
    for agent in order:
        path = _plan_single_agent(env, agent, planned)
        if path is None:
            return None
        planned[agent] = path
    return {agent: planned[agent] for agent in env.possible_agents}


def _plan_single_agent(
    env: UTMMAPFEnv,
    agent: str,
    planned: SpaceTimePlan,
) -> list[Cell] | None:
    state = env._state_by_agent[agent]
    mission = state.mission
    start = mission.start
    goal = mission.goal
    horizon = env.scenario.max_time_steps

    if start == goal:
        return [start]

    queue: list[tuple[int, int, int, Cell, int]] = []
    counter = 0
    start_key = (start, 0)
    parents: dict[tuple[Cell, int], tuple[Cell, int] | None] = {start_key: None}
    heapq.heappush(
        queue,
        (manhattan_distance(start, goal), 0, counter, start, 0),
    )

    while queue:
        _, time_step, _, cell, _ = heapq.heappop(queue)
        if cell == goal and _can_hold_goal(
            env=env,
            agent=agent,
            goal=goal,
            arrival_time=time_step,
            planned=planned,
        ):
            return _reconstruct_path(parents, (cell, time_step))
        if time_step >= horizon:
            continue

        for action in _ordered_actions(cell, goal):
            candidate = add_cell(cell, ACTION_DELTAS[action])
            next_time = time_step + 1
            key = (candidate, next_time)
            if key in parents:
                continue
            if not env.is_free_static(candidate):
                continue
            if env.is_no_fly(candidate, next_time):
                continue
            if _conflicts_with_reserved(
                env=env,
                agent=agent,
                from_cell=cell,
                to_cell=candidate,
                time_step=time_step,
                planned=planned,
            ):
                continue
            if candidate == goal and not _can_hold_goal(
                env=env,
                agent=agent,
                goal=goal,
                arrival_time=next_time,
                planned=planned,
            ):
                continue

            parents[key] = (cell, time_step)
            counter += 1
            wait_cost = 1 if action == int(UTMAction.WAIT) else 0
            priority = next_time + manhattan_distance(candidate, goal) + wait_cost
            heapq.heappush(
                queue,
                (priority, next_time, counter, candidate, action),
            )

    return None


def _ordered_actions(cell: Cell, goal: Cell) -> tuple[int, ...]:
    moving_actions = tuple(action for action in ACTION_DELTAS if action != UTMAction.WAIT)
    return tuple(
        sorted(
            moving_actions,
            key=lambda action: manhattan_distance(
                add_cell(cell, ACTION_DELTAS[action]), goal
            ),
        )
    ) + (int(UTMAction.WAIT),)


def _conflicts_with_reserved(
    env: UTMMAPFEnv,
    agent: str,
    from_cell: Cell,
    to_cell: Cell,
    time_step: int,
    planned: SpaceTimePlan,
) -> bool:
    mission = env._state_by_agent[agent].mission
    for other_agent, other_path in planned.items():
        other_state = env._state_by_agent[other_agent]
        other_from = planned_cell(other_path, time_step)
        other_to = planned_cell(other_path, time_step + 1)
        reason = transition_conflict(
            from_cell,
            to_cell,
            mission,
            other_from,
            other_to,
            other_state.mission,
        )
        if reason is not None:
            return True
    return False


def _can_hold_goal(
    env: UTMMAPFEnv,
    agent: str,
    goal: Cell,
    arrival_time: int,
    planned: SpaceTimePlan,
) -> bool:
    mission = env._state_by_agent[agent].mission
    for time_step in range(arrival_time, env.scenario.max_time_steps):
        for other_agent, other_path in planned.items():
            other_state = env._state_by_agent[other_agent]
            other_from = planned_cell(other_path, time_step)
            other_to = planned_cell(other_path, time_step + 1)
            reason = transition_conflict(
                goal,
                goal,
                mission,
                other_from,
                other_to,
                other_state.mission,
            )
            if reason is not None:
                return False
    return True


def _reconstruct_path(
    parents: dict[tuple[Cell, int], tuple[Cell, int] | None],
    key: tuple[Cell, int],
) -> list[Cell]:
    path = []
    while key is not None:
        cell, _ = key
        path.append(cell)
        key = parents[key]
    path.reverse()
    return path
