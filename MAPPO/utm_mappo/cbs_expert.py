from __future__ import annotations

import heapq
import time
from dataclasses import dataclass
from typing import Any

from .config import Cell
from .env import ACTION_DELTAS, UTMAction, UTMMAPFEnv
from .geometry import add_cell, manhattan_distance, transition_conflict
from .space_time_expert import action_from_transition, planned_cell


CBSPlan = dict[str, list[Cell]]


@dataclass(frozen=True)
class Constraint:
    agent: str
    time_step: int
    to_cell: Cell
    from_cell: Cell | None = None

    @property
    def is_edge(self) -> bool:
        return self.from_cell is not None


@dataclass(frozen=True)
class Conflict:
    agent_a: str
    agent_b: str
    time_step: int
    from_a: Cell
    to_a: Cell
    from_b: Cell
    to_b: Cell
    reason: str


@dataclass
class CBSResult:
    plan: CBSPlan | None
    expanded_nodes: int
    generated_nodes: int
    low_level_searches: int
    low_level_expansions: int
    elapsed_seconds: float
    reason: str | None = None


@dataclass
class _SearchBudget:
    deadline: float | None
    max_low_level_expansions: int
    low_level_searches: int = 0
    low_level_expansions: int = 0
    timed_out: bool = False
    low_level_exhausted: bool = False

    def expired(self) -> bool:
        if self.deadline is None:
            return False
        self.timed_out = time.monotonic() >= self.deadline
        return self.timed_out

    def record_low_level_expansion(self) -> bool:
        self.low_level_expansions += 1
        if self.max_low_level_expansions <= 0:
            return True
        if self.low_level_expansions <= self.max_low_level_expansions:
            return True
        self.low_level_exhausted = True
        return False

    @property
    def stop_reason(self) -> str | None:
        if self.timed_out:
            return "max_seconds"
        if self.low_level_exhausted:
            return "max_low_level_expansions"
        return None


def cbs_plan(
    env: UTMMAPFEnv,
    max_high_level_nodes: int = 0,
    max_low_level_expansions: int = 0,
    max_seconds: float = 0.0,
) -> CBSResult:
    """Generate complete offline demonstrations with Conflict-Based Search.

    This is a CPU baseline for teacher comparison experiments. It keeps CBS
    bounded so a difficult scene is skipped instead of blocking dataset
    generation. The conflict predicate is the same UTM transition predicate used
    by the environment: vertex, edge, and downwash conflicts.
    """

    started = time.monotonic()
    deadline = None if max_seconds <= 0 else started + max_seconds
    budget = _SearchBudget(
        deadline=deadline,
        max_low_level_expansions=max_low_level_expansions,
    )

    if not env._state_by_agent:
        env.reset()

    constraints: dict[str, frozenset[Constraint]] = {
        agent: frozenset() for agent in env.possible_agents
    }
    paths = _initial_paths(env, constraints, budget)
    if paths is None:
        return _result(
            plan=None,
            expanded_nodes=0,
            generated_nodes=0,
            budget=budget,
            started=started,
            reason=budget.stop_reason or "initial_low_level_failed",
        )

    open_nodes: list[tuple[int, int, int, dict[str, frozenset[Constraint]], CBSPlan]] = []
    counter = 0
    heapq.heappush(
        open_nodes,
        (_plan_cost(paths), _plan_makespan(paths), counter, constraints, paths),
    )
    seen = {_constraint_signature(constraints)}
    expanded_nodes = 0
    generated_nodes = 1

    while open_nodes:
        if budget.expired():
            return _result(
                plan=None,
                expanded_nodes=expanded_nodes,
                generated_nodes=generated_nodes,
                budget=budget,
                started=started,
                reason="max_seconds",
            )
        if max_high_level_nodes > 0 and expanded_nodes >= max_high_level_nodes:
            return _result(
                plan=None,
                expanded_nodes=expanded_nodes,
                generated_nodes=generated_nodes,
                budget=budget,
                started=started,
                reason="max_high_level_nodes",
            )

        _, _, _, node_constraints, node_paths = heapq.heappop(open_nodes)
        expanded_nodes += 1
        conflict = find_first_conflict(env, node_paths)
        if conflict is None:
            return _result(
                plan=node_paths,
                expanded_nodes=expanded_nodes,
                generated_nodes=generated_nodes,
                budget=budget,
                started=started,
                reason=None,
            )

        for constraint in constraints_for_conflict(conflict):
            next_constraints = {
                agent: values for agent, values in node_constraints.items()
            }
            agent_constraints = set(next_constraints[constraint.agent])
            if constraint in agent_constraints:
                continue
            agent_constraints.add(constraint)
            next_constraints[constraint.agent] = frozenset(agent_constraints)

            signature = _constraint_signature(next_constraints)
            if signature in seen:
                continue
            seen.add(signature)

            new_path = _low_level_plan(
                env,
                constraint.agent,
                next_constraints[constraint.agent],
                budget,
            )
            if new_path is None:
                if budget.stop_reason is not None:
                    return _result(
                        plan=None,
                        expanded_nodes=expanded_nodes,
                        generated_nodes=generated_nodes,
                        budget=budget,
                        started=started,
                        reason=budget.stop_reason,
                    )
                continue

            next_paths = dict(node_paths)
            next_paths[constraint.agent] = new_path
            counter += 1
            generated_nodes += 1
            heapq.heappush(
                open_nodes,
                (
                    _plan_cost(next_paths),
                    _plan_makespan(next_paths),
                    counter,
                    next_constraints,
                    next_paths,
                ),
            )

    return _result(
        plan=None,
        expanded_nodes=expanded_nodes,
        generated_nodes=generated_nodes,
        budget=budget,
        started=started,
        reason="open_set_exhausted",
    )


def actions_from_plan(env: UTMMAPFEnv, plan: CBSPlan) -> dict[str, int]:
    actions = {}
    for agent in env.agents:
        state = env._state_by_agent[agent]
        next_cell = planned_cell(plan[agent], env.time_step + 1)
        actions[agent] = action_from_transition(state.cell, next_cell)
    return actions


def find_first_conflict(env: UTMMAPFEnv, paths: CBSPlan) -> Conflict | None:
    horizon = env.scenario.max_time_steps
    agents = tuple(env.possible_agents)
    for time_step in range(horizon):
        for index, agent_a in enumerate(agents):
            state_a = env._state_by_agent[agent_a]
            from_a = planned_cell(paths[agent_a], time_step)
            to_a = planned_cell(paths[agent_a], time_step + 1)
            for agent_b in agents[index + 1 :]:
                state_b = env._state_by_agent[agent_b]
                from_b = planned_cell(paths[agent_b], time_step)
                to_b = planned_cell(paths[agent_b], time_step + 1)
                reason = transition_conflict(
                    from_a,
                    to_a,
                    state_a.mission,
                    from_b,
                    to_b,
                    state_b.mission,
                )
                if reason is not None:
                    return Conflict(
                        agent_a=agent_a,
                        agent_b=agent_b,
                        time_step=time_step,
                        from_a=from_a,
                        to_a=to_a,
                        from_b=from_b,
                        to_b=to_b,
                        reason=reason,
                    )
    return None


def constraints_for_conflict(conflict: Conflict) -> tuple[Constraint, Constraint]:
    if conflict.reason == "vertex":
        return (
            Constraint(
                agent=conflict.agent_a,
                time_step=conflict.time_step + 1,
                to_cell=conflict.to_a,
            ),
            Constraint(
                agent=conflict.agent_b,
                time_step=conflict.time_step + 1,
                to_cell=conflict.to_b,
            ),
        )

    return (
        Constraint(
            agent=conflict.agent_a,
            time_step=conflict.time_step,
            from_cell=conflict.from_a,
            to_cell=conflict.to_a,
        ),
        Constraint(
            agent=conflict.agent_b,
            time_step=conflict.time_step,
            from_cell=conflict.from_b,
            to_cell=conflict.to_b,
        ),
    )


def _initial_paths(
    env: UTMMAPFEnv,
    constraints: dict[str, frozenset[Constraint]],
    budget: _SearchBudget,
) -> CBSPlan | None:
    paths: CBSPlan = {}
    for agent in env.possible_agents:
        path = _low_level_plan(env, agent, constraints[agent], budget)
        if path is None:
            return None
        paths[agent] = path
    return paths


def _low_level_plan(
    env: UTMMAPFEnv,
    agent: str,
    constraints: frozenset[Constraint],
    budget: _SearchBudget,
) -> list[Cell] | None:
    budget.low_level_searches += 1
    state = env._state_by_agent[agent]
    mission = state.mission
    start = mission.start
    goal = mission.goal
    horizon = env.scenario.max_time_steps

    if _violates_vertex_constraints(start, 0, constraints):
        return None

    queue: list[tuple[int, int, int, Cell]] = []
    parents: dict[tuple[Cell, int], tuple[Cell, int] | None] = {(start, 0): None}
    counter = 0
    heapq.heappush(queue, (manhattan_distance(start, goal), 0, counter, start))

    while queue:
        if budget.expired() or not budget.record_low_level_expansion():
            return None

        _, time_step, _, cell = heapq.heappop(queue)
        if cell == goal and _can_wait_at_goal(
            goal,
            time_step,
            horizon,
            constraints,
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
            if _violates_constraints(
                from_cell=cell,
                to_cell=candidate,
                time_step=time_step,
                next_time=next_time,
                constraints=constraints,
            ):
                continue

            parents[key] = (cell, time_step)
            counter += 1
            wait_cost = 1 if action == int(UTMAction.WAIT) else 0
            priority = next_time + manhattan_distance(candidate, goal) + wait_cost
            heapq.heappush(queue, (priority, next_time, counter, candidate))

    return None


def _ordered_actions(cell: Cell, goal: Cell) -> tuple[int, ...]:
    moving_actions = tuple(
        int(action) for action in ACTION_DELTAS if action != UTMAction.WAIT
    )
    return tuple(
        sorted(
            moving_actions,
            key=lambda action: manhattan_distance(
                add_cell(cell, ACTION_DELTAS[action]), goal
            ),
        )
    ) + (int(UTMAction.WAIT),)


def _violates_constraints(
    from_cell: Cell,
    to_cell: Cell,
    time_step: int,
    next_time: int,
    constraints: frozenset[Constraint],
) -> bool:
    for constraint in constraints:
        if constraint.is_edge:
            if (
                constraint.time_step == time_step
                and constraint.from_cell == from_cell
                and constraint.to_cell == to_cell
            ):
                return True
        elif constraint.time_step == next_time and constraint.to_cell == to_cell:
            return True
    return False


def _violates_vertex_constraints(
    cell: Cell,
    time_step: int,
    constraints: frozenset[Constraint],
) -> bool:
    return any(
        not constraint.is_edge
        and constraint.time_step == time_step
        and constraint.to_cell == cell
        for constraint in constraints
    )


def _can_wait_at_goal(
    goal: Cell,
    arrival_time: int,
    horizon: int,
    constraints: frozenset[Constraint],
) -> bool:
    for wait_time in range(arrival_time, horizon + 1):
        if _violates_vertex_constraints(goal, wait_time, constraints):
            return False
    for wait_time in range(arrival_time, horizon):
        if _violates_constraints(
            from_cell=goal,
            to_cell=goal,
            time_step=wait_time,
            next_time=wait_time + 1,
            constraints=constraints,
        ):
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


def _plan_cost(paths: CBSPlan) -> int:
    return sum(max(0, len(path) - 1) for path in paths.values())


def _plan_makespan(paths: CBSPlan) -> int:
    return max((len(path) - 1 for path in paths.values()), default=0)


def _constraint_signature(
    constraints: dict[str, frozenset[Constraint]]
) -> tuple[tuple[str, tuple[tuple[Any, ...], ...]], ...]:
    return tuple(
        sorted(
            (
                agent,
                tuple(
                    sorted(
                        (
                            constraint.time_step,
                            constraint.from_cell
                            if constraint.from_cell is not None
                            else (-1, -1, -1),
                            constraint.to_cell,
                        )
                        for constraint in values
                    )
                ),
            )
            for agent, values in constraints.items()
        )
    )


def _result(
    plan: CBSPlan | None,
    expanded_nodes: int,
    generated_nodes: int,
    budget: _SearchBudget,
    started: float,
    reason: str | None,
) -> CBSResult:
    return CBSResult(
        plan=plan,
        expanded_nodes=expanded_nodes,
        generated_nodes=generated_nodes,
        low_level_searches=budget.low_level_searches,
        low_level_expansions=budget.low_level_expansions,
        elapsed_seconds=time.monotonic() - started,
        reason=reason,
    )
