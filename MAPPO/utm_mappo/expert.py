from __future__ import annotations

import heapq
import time
from collections import deque
from typing import Literal

from .config import Cell
from .env import ACTION_DELTAS, UTMAction, UTMMAPFEnv
from .geometry import add_cell, manhattan_distance, transition_conflict


MOVING_ACTIONS = (
    UTMAction.POS_X,
    UTMAction.NEG_X,
    UTMAction.POS_Y,
    UTMAction.NEG_Y,
    UTMAction.POS_Z,
    UTMAction.NEG_Z,
)

PIBTActionPlan = dict[str, list[int]]
PIBTProfile = dict[str, float | int | str]
PIBTDistanceMode = Literal["static", "astar"]
DEFAULT_PIBT_DISTANCE_MODE: PIBTDistanceMode = "static"
DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS = 200_000


def prioritized_pibt_action_plan(
    env: UTMMAPFEnv,
    profile: bool = False,
    profile_log_interval: float = 0.0,
    distance_mode: PIBTDistanceMode = DEFAULT_PIBT_DISTANCE_MODE,
    astar_max_expansions: int = DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS,
) -> PIBTActionPlan:
    """Precompute a full PIBT rollout as per-agent action sequences."""

    distance_mode = _validate_distance_mode(distance_mode)
    observations, _ = env.reset()
    if profile:
        reset_pibt_profile(
            env,
            log_interval_seconds=profile_log_interval,
            distance_mode=distance_mode,
            astar_max_expansions=astar_max_expansions,
        )
    plan: PIBTActionPlan = {agent: [] for agent in env.possible_agents}

    while observations:
        active_profile = get_pibt_profile(env)
        if active_profile is not None:
            active_profile["planning_steps"] += 1
            active_profile["active_agent_steps"] += len(observations)
        agents = sorted(observations)
        masks = {agent: env.action_mask(agent).copy() for agent in agents}
        expert_actions = prioritized_shortest_path_actions(
            env,
            agents,
            distance_mode=distance_mode,
            astar_max_expansions=astar_max_expansions,
        )
        actions: dict[str, int] = {}

        for agent in agents:
            action = int(expert_actions.get(agent, int(UTMAction.WAIT)))
            if action >= masks[agent].shape[0] or not masks[agent][action]:
                action = int(UTMAction.WAIT)
            actions[agent] = action

        for agent in env.possible_agents:
            plan[agent].append(int(actions.get(agent, int(UTMAction.WAIT))))

        observations, _, _, _, _ = env.step(actions)

    if profile:
        _log_pibt_profile(env, prefix="pibt profile final", force=True)
    return plan


def reset_pibt_profile(
    env: UTMMAPFEnv,
    log_interval_seconds: float = 0.0,
    distance_mode: PIBTDistanceMode = DEFAULT_PIBT_DISTANCE_MODE,
    astar_max_expansions: int = DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS,
) -> None:
    now = time.perf_counter()
    setattr(
        env,
        "_pibt_profile",
        {
            "started_at": now,
            "last_log_at": now,
            "log_interval_seconds": max(0.0, float(log_interval_seconds)),
            "distance_mode": distance_mode,
            "astar_max_expansions": max(0, int(astar_max_expansions)),
            "planning_steps": 0,
            "active_agent_steps": 0,
            "shortest_path_calls": 0,
            "manhattan_calls": 0,
            "static_distance_calls": 0,
            "static_fallbacks": 0,
            "time_bfs_cache_hits": 0,
            "time_bfs_calls": 0,
            "time_bfs_successes": 0,
            "time_bfs_failures": 0,
            "time_bfs_expansions": 0,
            "time_bfs_seconds": 0.0,
            "time_bfs_max_expansions": 0,
            "time_bfs_max_seconds": 0.0,
            "time_astar_cache_hits": 0,
            "time_astar_calls": 0,
            "time_astar_successes": 0,
            "time_astar_failures": 0,
            "time_astar_budget_exhaustions": 0,
            "time_astar_expansions": 0,
            "time_astar_seconds": 0.0,
            "time_astar_max_expansions": 0,
            "time_astar_max_seconds": 0.0,
        },
    )


def get_pibt_profile(env: UTMMAPFEnv) -> PIBTProfile | None:
    profile = getattr(env, "_pibt_profile", None)
    if profile is None:
        return None
    return profile


def shortest_path_action(env: UTMMAPFEnv, agent: str) -> int:
    actions = ranked_shortest_path_actions(env, agent)
    return actions[0] if actions else int(UTMAction.WAIT)


def prioritized_shortest_path_actions(
    env: UTMMAPFEnv,
    agents: list[str] | tuple[str, ...] | None = None,
    *,
    distance_mode: PIBTDistanceMode = DEFAULT_PIBT_DISTANCE_MODE,
    astar_max_expansions: int = DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS,
) -> dict[str, int]:
    distance_mode = _validate_distance_mode(distance_mode)
    if agents is None:
        agents = tuple(env.agents)

    active_set = set(agents)
    active_agents = sorted(
        agents,
        key=lambda name: _dynamic_priority_key(
            env,
            name,
            len(active_set),
            distance_mode=distance_mode,
            astar_max_expansions=astar_max_expansions,
        ),
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

        for action in ranked_shortest_path_actions(
            env,
            agent,
            distance_mode=distance_mode,
            astar_max_expansions=astar_max_expansions,
        ):
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
    env: UTMMAPFEnv,
    agent: str,
    active_count: int,
    *,
    distance_mode: PIBTDistanceMode,
    astar_max_expansions: int,
) -> tuple[int, int, int, int, int]:
    state = env._state_by_agent[agent]
    mission_id = state.mission.mission_id
    active_count = max(1, active_count)
    round_robin_rank = (mission_id - 1 - env.time_step) % active_count
    distance = _shortest_path_length(
        env,
        start=state.cell,
        goal=state.mission.goal,
        start_time=env.time_step,
        distance_mode=distance_mode,
        astar_max_expansions=astar_max_expansions,
    )
    if distance is None:
        distance = 10_000

    return (
        -_wait_streak(state.path),
        -_oscillation_streak(state.path),
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


def _oscillation_streak(path: list[Cell]) -> int:
    if len(path) < 4:
        return 0

    streak = 0
    cursor = len(path) - 1
    while cursor >= 3:
        if path[cursor] != path[cursor - 2]:
            break
        if path[cursor - 1] != path[cursor - 3]:
            break
        if path[cursor] == path[cursor - 1]:
            break
        streak += 1
        cursor -= 2
    return streak


def _recent_visit_count(path: list[Cell], candidate: Cell, window: int = 8) -> int:
    if not path:
        return 0
    return sum(1 for cell in path[-window:] if cell == candidate)


def ranked_shortest_path_actions(
    env: UTMMAPFEnv,
    agent: str,
    *,
    distance_mode: PIBTDistanceMode = DEFAULT_PIBT_DISTANCE_MODE,
    astar_max_expansions: int = DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS,
) -> list[int]:
    distance_mode = _validate_distance_mode(distance_mode)
    state = env._state_by_agent[agent]
    if state.cell == state.mission.goal:
        return [int(UTMAction.WAIT)]

    action_mask = env.action_mask(agent)
    scored_actions: list[tuple[int, int, int, int, int, int]] = []
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
            distance_mode=distance_mode,
            astar_max_expansions=astar_max_expansions,
        )
        if remaining is None:
            continue

        oscillation_cost = (
            1 if candidate == state.previous_cell and candidate != state.cell else 0
        )
        recent_visit_cost = _recent_visit_count(state.path, candidate)
        combined_cost = remaining + recent_visit_cost * 3 + oscillation_cost * 5
        scored_actions.append(
            (
                combined_cost,
                recent_visit_cost,
                remaining,
                oscillation_cost,
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
    env: UTMMAPFEnv,
    start: Cell,
    goal: Cell,
    start_time: int,
    *,
    distance_mode: PIBTDistanceMode = DEFAULT_PIBT_DISTANCE_MODE,
    astar_max_expansions: int = DEFAULT_PIBT_ASTAR_MAX_EXPANSIONS,
) -> int | None:
    distance_mode = _validate_distance_mode(distance_mode)
    profile = get_pibt_profile(env)
    if profile is not None:
        profile["shortest_path_calls"] += 1

    has_no_fly = _has_enabled_no_fly(env)
    if not env.scenario.grid.blocked_cells and not has_no_fly:
        if profile is not None:
            profile["manhattan_calls"] += 1
        return manhattan_distance(start, goal)

    if distance_mode == "static" or not has_no_fly:
        if profile is not None:
            profile["static_distance_calls"] += 1
        distance_map = _static_distance_map(env, goal)
        return distance_map.get(start)

    cache = _distance_cache(env)
    search_budget = max(0, int(astar_max_expansions))
    cache_key = ("astar", start, goal, start_time, search_budget)
    if cache_key in cache:
        if profile is not None:
            profile["time_astar_cache_hits"] += 1
        return cache[cache_key]

    if start == goal:
        cache[cache_key] = 0
        return 0

    result, budget_exhausted = _time_expanded_astar_length(
        env,
        start=start,
        goal=goal,
        start_time=start_time,
        max_expansions=search_budget,
    )
    if result is None and budget_exhausted:
        if profile is not None:
            profile["static_fallbacks"] += 1
        result = _static_distance_map(env, goal).get(start)
    cache[cache_key] = result
    return result


def _time_expanded_astar_length(
    env: UTMMAPFEnv,
    start: Cell,
    goal: Cell,
    start_time: int,
    max_expansions: int,
) -> tuple[int | None, bool]:
    profile = get_pibt_profile(env)
    if profile is not None:
        profile["time_astar_calls"] += 1
    search_started = time.perf_counter()
    expansions = 0
    result: int | None = None
    budget_exhausted = False
    horizon = env.scenario.max_time_steps
    static_distances = _static_distance_map(env, goal)
    initial_h = static_distances.get(start)
    if initial_h is None:
        _record_time_astar_profile(
            env,
            result=None,
            expansions=0,
            elapsed=time.perf_counter() - search_started,
            budget_exhausted=False,
        )
        return None, False

    queue: list[tuple[int, int, int, int, Cell, int]] = []
    sequence = 0
    heapq.heappush(queue, (initial_h, initial_h, 0, sequence, start, start_time))
    best_distance: dict[tuple[Cell, int], int] = {(start, start_time): 0}

    while queue:
        _, _, distance, _, cell, time_step = heapq.heappop(queue)
        if best_distance.get((cell, time_step)) != distance:
            continue
        expansions += 1
        if max_expansions > 0 and expansions > max_expansions:
            budget_exhausted = True
            break
        if cell == goal:
            result = distance
            break

        if time_step >= horizon:
            continue

        for delta in ACTION_DELTAS.values():
            candidate = add_cell(cell, delta)
            next_time = time_step + 1
            if not env.is_free_static(candidate):
                continue
            if env.is_no_fly(candidate, next_time):
                continue
            heuristic = static_distances.get(candidate)
            if heuristic is None:
                continue

            next_distance = distance + 1
            visit_key = (candidate, next_time)
            if next_distance >= best_distance.get(visit_key, 1_000_000_000):
                continue
            best_distance[visit_key] = next_distance
            sequence += 1
            heapq.heappush(
                queue,
                (
                    next_distance + heuristic,
                    heuristic,
                    next_distance,
                    sequence,
                    candidate,
                    next_time,
                ),
            )

        _maybe_log_pibt_time_search_progress(env, expansions)

    _record_time_astar_profile(
        env,
        result=result,
        expansions=expansions,
        elapsed=time.perf_counter() - search_started,
        budget_exhausted=budget_exhausted,
    )
    return result, budget_exhausted


def _record_time_astar_profile(
    env: UTMMAPFEnv,
    *,
    result: int | None,
    expansions: int,
    elapsed: float,
    budget_exhausted: bool,
) -> None:
    profile = get_pibt_profile(env)
    if profile is None:
        return
    profile["time_astar_expansions"] += expansions
    profile["time_astar_seconds"] += elapsed
    profile["time_astar_max_expansions"] = max(
        int(profile["time_astar_max_expansions"]),
        expansions,
    )
    profile["time_astar_max_seconds"] = max(
        float(profile["time_astar_max_seconds"]),
        elapsed,
    )
    if budget_exhausted:
        profile["time_astar_budget_exhaustions"] += 1
    if result is None:
        profile["time_astar_failures"] += 1
    else:
        profile["time_astar_successes"] += 1


def _maybe_log_pibt_time_search_progress(
    env: UTMMAPFEnv, current_expansions: int
) -> None:
    profile = get_pibt_profile(env)
    if profile is None:
        return
    interval = float(profile["log_interval_seconds"])
    if interval <= 0.0:
        return
    now = time.perf_counter()
    if now - float(profile["last_log_at"]) < interval:
        return
    profile["last_log_at"] = now
    _log_pibt_profile(
        env,
        prefix=f"pibt profile in-search current_expansions={current_expansions}",
    )


def _log_pibt_profile(
    env: UTMMAPFEnv,
    prefix: str = "pibt profile",
    force: bool = False,
) -> None:
    profile = get_pibt_profile(env)
    if profile is None:
        return
    interval = float(profile["log_interval_seconds"])
    if not force and interval <= 0.0:
        return
    elapsed = time.perf_counter() - float(profile["started_at"])
    astar_calls = max(1, int(profile["time_astar_calls"]))
    mean_astar_seconds = float(profile["time_astar_seconds"]) / astar_calls
    mean_astar_expansions = float(profile["time_astar_expansions"]) / astar_calls
    print(
        f"{prefix}: elapsed={elapsed:.1f}s time_step={env.time_step} "
        f"mode={profile.get('distance_mode', DEFAULT_PIBT_DISTANCE_MODE)} "
        f"planning_steps={int(profile['planning_steps'])} "
        f"shortest_calls={int(profile['shortest_path_calls'])} "
        f"static_calls={int(profile['static_distance_calls'])} "
        f"static_fallbacks={int(profile['static_fallbacks'])} "
        f"time_astar_calls={int(profile['time_astar_calls'])} "
        f"cache_hits={int(profile['time_astar_cache_hits'])} "
        f"budget_exhaustions={int(profile['time_astar_budget_exhaustions'])} "
        f"astar_seconds={float(profile['time_astar_seconds']):.2f} "
        f"mean_astar_seconds={mean_astar_seconds:.4f} "
        f"astar_expansions={int(profile['time_astar_expansions'])} "
        f"mean_astar_expansions={mean_astar_expansions:.1f} "
        f"max_astar_seconds={float(profile['time_astar_max_seconds']):.4f} "
        f"max_astar_expansions={int(profile['time_astar_max_expansions'])}",
        flush=True,
    )


def _distance_cache(
    env: UTMMAPFEnv,
) -> dict[tuple[str, Cell, Cell, int, int], int | None]:
    cache = getattr(env, "_expert_distance_cache", None)
    if cache is None:
        cache = {}
        setattr(env, "_expert_distance_cache", cache)
    return cache


def _has_enabled_no_fly(env: UTMMAPFEnv) -> bool:
    return any(zone.enabled for zone in env.scenario.no_fly_zones)


def _validate_distance_mode(distance_mode: str) -> PIBTDistanceMode:
    if distance_mode not in ("static", "astar"):
        raise ValueError(
            f"unsupported PIBT distance mode {distance_mode!r}; "
            "expected 'static' or 'astar'"
        )
    return distance_mode  # type: ignore[return-value]


def _static_distance_map(env: UTMMAPFEnv, goal: Cell) -> dict[Cell, int]:
    """Shortest static-grid distance to one goal, cached per environment.

    PIBT uses these distances many times per rollout to rank local actions. For
    obstacle-only scenarios there is no need to rerun a time-expanded BFS from
    every candidate cell; one reverse BFS from the goal is enough.
    """

    cache = getattr(env, "_expert_static_distance_maps", None)
    if cache is None:
        cache = {}
        setattr(env, "_expert_static_distance_maps", cache)

    if goal in cache:
        return cache[goal]

    distances: dict[Cell, int] = {}
    if not env.is_free_static(goal):
        cache[goal] = distances
        return distances

    queue: deque[Cell] = deque([goal])
    distances[goal] = 0

    moving_deltas = tuple(
        delta for action, delta in ACTION_DELTAS.items() if action != UTMAction.WAIT
    )
    while queue:
        cell = queue.popleft()
        distance = distances[cell]
        for delta in moving_deltas:
            candidate = add_cell(cell, delta)
            if candidate in distances:
                continue
            if not env.is_free_static(candidate):
                continue
            distances[candidate] = distance + 1
            queue.append(candidate)

    cache[goal] = distances
    return distances


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
