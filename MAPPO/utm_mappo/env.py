from __future__ import annotations

from enum import IntEnum
from typing import Any

import numpy as np
from gymnasium import spaces
from pettingzoo import ParallelEnv

from .config import AgentRuntimeState, Cell, UTMScenario
from .geometry import (
    add_cell,
    boxes_overlap,
    manhattan_distance,
    protection_box,
    downwash_box,
    transition_conflict,
    zone_contains,
)


class UTMAction(IntEnum):
    WAIT = 0
    POS_X = 1
    NEG_X = 2
    POS_Y = 3
    NEG_Y = 4
    POS_Z = 5
    NEG_Z = 6


ACTION_DELTAS: dict[int, Cell] = {
    UTMAction.WAIT: (0, 0, 0),
    UTMAction.POS_X: (1, 0, 0),
    UTMAction.NEG_X: (-1, 0, 0),
    UTMAction.POS_Y: (0, 1, 0),
    UTMAction.NEG_Y: (0, -1, 0),
    UTMAction.POS_Z: (0, 0, 1),
    UTMAction.NEG_Z: (0, 0, -1),
}


class UTMMAPFEnv(ParallelEnv):
    metadata = {"name": "utm_mapf_v0", "render_modes": ["none"]}

    def __init__(
        self,
        scenario: UTMScenario | dict[str, Any],
        render_mode: str | None = None,
        invalid_action_penalty: float = -1.0,
        unsafe_hold_penalty: float = -2.0,
        step_penalty: float = -0.05,
        wait_penalty: float = -0.02,
        oscillation_penalty: float = -0.2,
        goal_reward: float = 10.0,
        progress_reward_scale: float = 0.25,
    ):
        if isinstance(scenario, dict):
            scenario = UTMScenario.from_dict(scenario)
        self.scenario = scenario
        self.render_mode = render_mode
        self.invalid_action_penalty = invalid_action_penalty
        self.unsafe_hold_penalty = unsafe_hold_penalty
        self.step_penalty = step_penalty
        self.wait_penalty = wait_penalty
        self.oscillation_penalty = oscillation_penalty
        self.goal_reward = goal_reward
        self.progress_reward_scale = progress_reward_scale

        self.dimensions = scenario.grid.dimensions
        self.occupancy = np.zeros(self.dimensions, dtype=np.bool_)
        for cell in scenario.grid.blocked_cells:
            if self.is_inside(cell):
                self.occupancy[cell] = True

        self._validate_scenario()

        self.possible_agents = [
            f"agent_{mission.mission_id}" for mission in scenario.missions
        ]
        self._state_by_agent: dict[str, AgentRuntimeState] = {}
        self.agents: list[str] = []
        self.time_step = 0
        self.np_random = np.random.default_rng()

        self._priority_history_dim = 4
        self._state_dim = (
            3 + 3 + 1 + 1 + 1 + self._priority_history_dim + len(UTMAction)
        )
        self._local_channels = 5
        self._local_side = scenario.observation_radius * 2 + 1
        self._obs_dim = self._state_dim + (
            self._local_channels * self._local_side**3
        )

    def observation_space(self, agent: str) -> spaces.Box:
        del agent
        return spaces.Box(low=-1.0, high=1.0, shape=(self._obs_dim,), dtype=np.float32)

    def action_space(self, agent: str) -> spaces.Discrete:
        del agent
        return spaces.Discrete(len(UTMAction))

    def action_mask(self, agent: str) -> np.ndarray:
        mask = np.zeros(len(UTMAction), dtype=np.bool_)
        state = self._state_by_agent.get(agent)
        if state is None or state.reached_goal:
            mask[int(UTMAction.WAIT)] = True
            return mask

        next_time = self.time_step + 1
        for action, delta in ACTION_DELTAS.items():
            candidate = add_cell(state.cell, delta)
            if self.is_free_static(candidate) and not self.is_no_fly(candidate, next_time):
                mask[int(action)] = True

        if not mask.any():
            mask[int(UTMAction.WAIT)] = True
        return mask

    @property
    def observation_spaces(self) -> dict[str, spaces.Box]:
        return {agent: self.observation_space(agent) for agent in self.possible_agents}

    @property
    def action_spaces(self) -> dict[str, spaces.Discrete]:
        return {agent: self.action_space(agent) for agent in self.possible_agents}

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, np.ndarray], dict[str, dict[str, Any]]]:
        del options
        if seed is not None:
            self.np_random = np.random.default_rng(seed)

        self.time_step = 0
        self._state_by_agent = {}
        for mission in self.scenario.missions:
            agent = f"agent_{mission.mission_id}"
            self._state_by_agent[agent] = AgentRuntimeState(
                mission=mission,
                cell=mission.start,
                previous_cell=mission.start,
                reached_goal=mission.start == mission.goal,
                path=[mission.start],
            )

        self.agents = [
            agent
            for agent in self.possible_agents
            if not self._state_by_agent[agent].reached_goal
        ]
        observations = {agent: self._observe(agent) for agent in self.agents}
        infos = {agent: {} for agent in self.agents}
        return observations, infos

    def step(
        self, actions: dict[str, int]
    ) -> tuple[
        dict[str, np.ndarray],
        dict[str, float],
        dict[str, bool],
        dict[str, bool],
        dict[str, dict[str, Any]],
    ]:
        if not self.agents:
            return {}, {}, {}, {}, {}

        previous_distances = {
            agent: manhattan_distance(state.cell, state.mission.goal)
            for agent, state in self._state_by_agent.items()
        }
        next_time = self.time_step + 1
        proposals: dict[str, Cell] = {}
        invalid_agents: set[str] = set()
        no_fly_agents: set[str] = set()
        unsafe_agents: set[str] = set()
        conflict_reasons: dict[str, str] = {}

        for agent in self.possible_agents:
            state = self._state_by_agent[agent]
            if state.reached_goal:
                proposals[agent] = state.cell
                continue

            action = int(actions.get(agent, UTMAction.WAIT))
            action_mask = self.action_mask(agent)
            if action not in ACTION_DELTAS or not action_mask[action]:
                action = UTMAction.WAIT
                invalid_agents.add(agent)

            candidate = add_cell(state.cell, ACTION_DELTAS[action])
            state.last_action = action
            if not self.is_free_static(candidate):
                candidate = state.cell
                invalid_agents.add(agent)

            if self.is_no_fly(candidate, next_time):
                candidate = state.cell
                no_fly_agents.add(agent)

            proposals[agent] = candidate

        self._resolve_dynamic_conflicts(
            proposals=proposals,
            unsafe_agents=unsafe_agents,
            conflict_reasons=conflict_reasons,
        )

        rewards: dict[str, float] = {}
        terminations: dict[str, bool] = {}
        truncations: dict[str, bool] = {}
        infos: dict[str, dict[str, Any]] = {}

        self.time_step = next_time
        for agent in self.possible_agents:
            state = self._state_by_agent[agent]
            if state.reached_goal:
                continue

            previous_cell = state.previous_cell
            old_cell = state.cell
            new_cell = proposals[agent]
            reached_now = new_cell == state.mission.goal
            waited = new_cell == old_cell and not reached_now
            oscillated = new_cell == previous_cell and new_cell != old_cell

            state.previous_cell = old_cell
            state.cell = new_cell
            state.path.append(new_cell)
            state.reached_goal = reached_now

            previous_distance = previous_distances[agent]
            current_distance = manhattan_distance(new_cell, state.mission.goal)
            reward = self.step_penalty
            reward += (previous_distance - current_distance) * self.progress_reward_scale
            if agent in invalid_agents:
                reward += self.invalid_action_penalty
            if agent in no_fly_agents:
                reward += self.invalid_action_penalty
            if agent in unsafe_agents:
                reward += self.unsafe_hold_penalty
            if waited:
                reward += self.wait_penalty
            if oscillated:
                reward += self.oscillation_penalty
            if reached_now:
                reward += self.goal_reward

            done = reached_now
            truncated = self.time_step >= self.scenario.max_time_steps
            rewards[agent] = float(reward)
            terminations[agent] = done
            truncations[agent] = truncated
            infos[agent] = {
                "cell": new_cell,
                "goal": state.mission.goal,
                "invalid_action": agent in invalid_agents,
                "no_fly_hold": agent in no_fly_agents,
                "unsafe_hold": agent in unsafe_agents,
                "waited": waited,
                "oscillated": oscillated,
                "conflict_reason": conflict_reasons.get(agent),
            }

        self.agents = [
            agent
            for agent in self.possible_agents
            if not self._state_by_agent[agent].reached_goal
            and self.time_step < self.scenario.max_time_steps
        ]
        observations = {agent: self._observe(agent) for agent in self.agents}
        return observations, rewards, terminations, truncations, infos

    def state(self) -> np.ndarray:
        return np.concatenate(
            [self._observe(agent) for agent in self.possible_agents], dtype=np.float32
        )

    def render(self) -> dict[str, Any]:
        return {
            "time_step": self.time_step,
            "agents": {
                agent: {
                    "mission_id": state.mission.mission_id,
                    "cell": state.cell,
                    "goal": state.mission.goal,
                    "reached_goal": state.reached_goal,
                }
                for agent, state in self._state_by_agent.items()
            },
        }

    def close(self) -> None:
        return None

    def _resolve_dynamic_conflicts(
        self,
        proposals: dict[str, Cell],
        unsafe_agents: set[str],
        conflict_reasons: dict[str, str],
    ) -> None:
        names = self.possible_agents
        max_passes = max(1, len(names) * 2)

        for _ in range(max_passes):
            changed = False
            unresolved: list[tuple[str, str, str]] = []

            for i, agent_a in enumerate(names):
                state_a = self._state_by_agent[agent_a]
                for agent_b in names[i + 1 :]:
                    state_b = self._state_by_agent[agent_b]
                    reason = transition_conflict(
                        state_a.cell,
                        proposals[agent_a],
                        state_a.mission,
                        state_b.cell,
                        proposals[agent_b],
                        state_b.mission,
                    )
                    if reason is None:
                        continue

                    yielding_agent = self._choose_yielding_agent(
                        agent_a, agent_b, proposals
                    )
                    if yielding_agent not in self.agents:
                        other_agent = agent_b if yielding_agent == agent_a else agent_a
                        yielding_agent = other_agent

                    if yielding_agent in self.agents:
                        unsafe_agents.add(yielding_agent)
                        conflict_reasons[yielding_agent] = reason
                        current_cell = self._state_by_agent[yielding_agent].cell
                        if proposals[yielding_agent] != current_cell:
                            proposals[yielding_agent] = current_cell
                            changed = True
                        else:
                            unresolved.append((agent_a, agent_b, reason))

            if changed:
                continue

            for agent_a, agent_b, reason in unresolved:
                for agent in (agent_a, agent_b):
                    if agent not in self.agents:
                        continue
                    unsafe_agents.add(agent)
                    conflict_reasons[agent] = reason
                    current_cell = self._state_by_agent[agent].cell
                    if proposals[agent] != current_cell:
                        proposals[agent] = current_cell
                        changed = True

            if not changed:
                break

    def _choose_yielding_agent(
        self, agent_a: str, agent_b: str, proposals: dict[str, Cell]
    ) -> str:
        state_a = self._state_by_agent[agent_a]
        state_b = self._state_by_agent[agent_b]

        active_a = agent_a in self.agents
        active_b = agent_b in self.agents
        if active_a != active_b:
            return agent_a if active_a else agent_b

        moving_a = proposals[agent_a] != state_a.cell
        moving_b = proposals[agent_b] != state_b.cell
        if moving_a != moving_b:
            return agent_a if moving_a else agent_b

        at_goal_a = state_a.cell == state_a.mission.goal
        at_goal_b = state_b.cell == state_b.mission.goal
        if at_goal_a != at_goal_b:
            return agent_b if at_goal_a else agent_a

        return (
            agent_a
            if state_a.mission.mission_id > state_b.mission.mission_id
            else agent_b
        )

    def is_inside(self, cell: Cell) -> bool:
        return (
            0 <= cell[0] < self.dimensions[0]
            and 0 <= cell[1] < self.dimensions[1]
            and 0 <= cell[2] < self.dimensions[2]
        )

    def is_free_static(self, cell: Cell) -> bool:
        return self.is_inside(cell) and not bool(self.occupancy[cell])

    def is_no_fly(self, cell: Cell, time_step: int) -> bool:
        return any(
            zone_contains(zone, cell, time_step)
            for zone in self.scenario.no_fly_zones
        )

    def _validate_scenario(self) -> None:
        for mission in self.scenario.missions:
            for label, cell in (("start", mission.start), ("goal", mission.goal)):
                if not self.is_inside(cell):
                    raise ValueError(
                        f"mission {mission.mission_id} {label} cell is outside grid: {cell}"
                    )
                if self.occupancy[cell]:
                    raise ValueError(
                        f"mission {mission.mission_id} {label} cell is blocked: {cell}"
                    )
            if self.is_no_fly(mission.start, 0):
                raise ValueError(
                    f"mission {mission.mission_id} start cell is in a no-fly zone at t=0"
                )

        for index_a, mission_a in enumerate(self.scenario.missions):
            for mission_b in self.scenario.missions[index_a + 1 :]:
                if boxes_overlap(
                    protection_box(mission_a.start, mission_a),
                    protection_box(mission_b.start, mission_b),
                ):
                    raise ValueError(
                        "initial protection conflict between "
                        f"mission {mission_a.mission_id} and mission {mission_b.mission_id}"
                    )
                if boxes_overlap(
                    downwash_box(mission_a.start, mission_a),
                    protection_box(mission_b.start, mission_b),
                ) or boxes_overlap(
                    downwash_box(mission_b.start, mission_b),
                    protection_box(mission_a.start, mission_a),
                ):
                    raise ValueError(
                        "initial downwash conflict between "
                        f"mission {mission_a.mission_id} and mission {mission_b.mission_id}"
                    )

    def _observe(self, agent: str) -> np.ndarray:
        state = self._state_by_agent[agent]
        dims = np.asarray(self.dimensions, dtype=np.float32)
        cell = np.asarray(state.cell, dtype=np.float32)
        goal = np.asarray(state.mission.goal, dtype=np.float32)
        delta = goal - cell
        max_distance = max(1.0, float(sum(self.dimensions)))

        action_one_hot = np.zeros(len(UTMAction), dtype=np.float32)
        action_one_hot[int(state.last_action)] = 1.0

        priority_history = self._priority_history_features(agent)
        state_vector = np.concatenate(
            [
                (cell / np.maximum(dims - 1, 1.0)) * 2.0 - 1.0,
                delta / np.maximum(dims, 1.0),
                np.asarray(
                    [
                        manhattan_distance(state.cell, state.mission.goal)
                        / max_distance,
                        1.0 if state.reached_goal else 0.0,
                        self.time_step / max(1.0, self.scenario.max_time_steps),
                    ],
                    dtype=np.float32,
                ),
                priority_history,
                action_one_hot,
            ],
            dtype=np.float32,
        )

        local = self._local_observation(agent)
        return np.concatenate([state_vector, local.flatten()], dtype=np.float32)

    def _priority_history_features(self, agent: str) -> np.ndarray:
        state = self._state_by_agent[agent]
        mission_ids = [mission.mission_id for mission in self.scenario.missions]
        min_mission_id = min(mission_ids)
        max_mission_id = max(mission_ids)
        mission_span = max(1, max_mission_id - min_mission_id)
        mission_id_norm = (state.mission.mission_id - min_mission_id) / mission_span

        active_count = max(1, len(self.agents))
        active_denominator = max(1, active_count - 1)
        round_robin_rank = (
            state.mission.mission_id - 1 - self.time_step
        ) % active_count
        round_robin_rank_norm = round_robin_rank / active_denominator

        wait_streak_norm = min(self._wait_streak(state.path), 10) / 10.0
        oscillation_streak_norm = min(self._oscillation_streak(state.path), 5) / 5.0

        return np.asarray(
            [
                mission_id_norm,
                round_robin_rank_norm,
                wait_streak_norm,
                oscillation_streak_norm,
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _wait_streak(path: list[Cell]) -> int:
        if len(path) < 2:
            return 0

        streak = 0
        cursor = len(path) - 1
        while cursor > 0 and path[cursor] == path[cursor - 1]:
            streak += 1
            cursor -= 1
        return streak

    @staticmethod
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

    def _local_observation(self, agent: str) -> np.ndarray:
        radius = self.scenario.observation_radius
        side = self._local_side
        center = self._state_by_agent[agent].cell
        grid = np.zeros((self._local_channels, side, side, side), dtype=np.float32)

        other_boxes = [
            protection_box(state.cell, state.mission)
            for other_agent, state in self._state_by_agent.items()
            if other_agent != agent
        ]
        goal = self._state_by_agent[agent].mission.goal

        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    cell = (center[0] + dx, center[1] + dy, center[2] + dz)
                    ix, iy, iz = dx + radius, dy + radius, dz + radius
                    if not self.is_inside(cell):
                        grid[0, ix, iy, iz] = 1.0
                        continue
                    if self.occupancy[cell]:
                        grid[0, ix, iy, iz] = 1.0
                    if self.is_no_fly(cell, self.time_step + 1):
                        grid[1, ix, iy, iz] = 1.0
                    if any(
                        box.min_cell[0] <= cell[0] <= box.max_cell[0]
                        and box.min_cell[1] <= cell[1] <= box.max_cell[1]
                        and box.min_cell[2] <= cell[2] <= box.max_cell[2]
                        for box in other_boxes
                    ):
                        grid[2, ix, iy, iz] = 1.0
                    if cell == goal:
                        grid[3, ix, iy, iz] = 1.0

        grid[4, radius, radius, radius] = 1.0
        return grid
