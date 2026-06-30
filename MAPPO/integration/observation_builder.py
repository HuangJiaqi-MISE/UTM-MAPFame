from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from utm_mappo.config import GridConfig, MissionConfig, TemporalNoFlyZoneConfig, UTMScenario
from utm_mappo.env import ACTION_DELTAS, UTMAction, UTMMAPFEnv
from utm_mappo.geometry import add_cell, manhattan_distance

from .schemas import EmergencyStepRequest, FailedAgentSnapshot, NoFlyZoneSnapshot


LOCAL_CHANNELS = 5
PRIORITY_HISTORY_DIM = 4
STATE_DIM = 3 + 3 + 1 + 1 + 1 + PRIORITY_HISTORY_DIM + len(UTMAction)


@dataclass(frozen=True)
class BuiltObservations:
    agents: tuple[FailedAgentSnapshot, ...]
    observations: np.ndarray
    action_masks: np.ndarray


def build_observations(request: EmergencyStepRequest) -> BuiltObservations:
    agents = request.failed_agents
    observations = np.stack([_observe(request, agent) for agent in agents]).astype(
        np.float32
    )
    masks = np.stack([action_mask(request, agent) for agent in agents]).astype(np.bool_)
    return BuiltObservations(agents=agents, observations=observations, action_masks=masks)


def observation_dim(observation_radius: int) -> int:
    side = observation_radius * 2 + 1
    return STATE_DIM + LOCAL_CHANNELS * side**3


def action_mask(request: EmergencyStepRequest, agent: FailedAgentSnapshot) -> np.ndarray:
    mask = np.zeros(len(UTMAction), dtype=np.bool_)
    next_time = request.time_step + 1
    for action, delta in ACTION_DELTAS.items():
        candidate = add_cell(agent.current_cell, delta)
        if is_free_static(request, candidate) and not is_no_fly(
            request, candidate, next_time
        ):
            mask[int(action)] = True
    if not mask.any():
        mask[int(UTMAction.WAIT)] = True
    return mask


def build_env_for_model(request: EmergencyStepRequest) -> UTMMAPFEnv:
    """Builds a signature-compatible env for DiscreteMAPPO.load."""

    scenario = UTMScenario(
        grid=GridConfig(
            dimensions=request.grid_dimensions,
            blocked_cells=request.blocked_cells,
        ),
        missions=tuple(
            MissionConfig(
                mission_id=agent.mission_id,
                start=agent.current_cell,
                goal=agent.goal_cell,
            )
            for agent in request.failed_agents
        ),
        no_fly_zones=tuple(_to_config(zone) for zone in request.no_fly_zones),
        max_time_steps=request.max_time_steps,
        observation_radius=request.observation_radius,
    )
    return UTMMAPFEnv(scenario)


def is_inside(request: EmergencyStepRequest, cell: tuple[int, int, int]) -> bool:
    dims = request.grid_dimensions
    return 0 <= cell[0] < dims[0] and 0 <= cell[1] < dims[1] and 0 <= cell[2] < dims[2]


def is_free_static(request: EmergencyStepRequest, cell: tuple[int, int, int]) -> bool:
    return is_inside(request, cell) and cell not in request.blocked_cell_set


def is_no_fly(
    request: EmergencyStepRequest,
    cell: tuple[int, int, int],
    time_step: int,
) -> bool:
    for zone in request.no_fly_zones:
        if not zone.enabled:
            continue
        if time_step < zone.start_time_step or time_step > zone.end_time_step:
            continue
        if (
            zone.min_cell[0] <= cell[0] <= zone.max_cell[0]
            and zone.min_cell[1] <= cell[1] <= zone.max_cell[1]
            and zone.min_cell[2] <= cell[2] <= zone.max_cell[2]
        ):
            return True
    return False


def _observe(
    request: EmergencyStepRequest,
    agent: FailedAgentSnapshot,
) -> np.ndarray:
    dims = np.asarray(request.grid_dimensions, dtype=np.float32)
    cell = np.asarray(agent.current_cell, dtype=np.float32)
    goal = np.asarray(agent.goal_cell, dtype=np.float32)
    delta = goal - cell
    max_distance = max(1.0, float(sum(request.grid_dimensions)))

    action_one_hot = np.zeros(len(UTMAction), dtype=np.float32)
    action_one_hot[int(agent.last_action)] = 1.0

    state_vector = np.concatenate(
        [
            (cell / np.maximum(dims - 1, 1.0)) * 2.0 - 1.0,
            delta / np.maximum(dims, 1.0),
            np.asarray(
                [
                    manhattan_distance(agent.current_cell, agent.goal_cell)
                    / max_distance,
                    1.0 if agent.current_cell == agent.goal_cell else 0.0,
                    request.time_step / max(1.0, request.max_time_steps),
                ],
                dtype=np.float32,
            ),
            _priority_history_features(request, agent),
            action_one_hot,
        ],
        dtype=np.float32,
    )

    local = _local_observation(request, agent)
    return np.concatenate([state_vector, local.flatten()], dtype=np.float32)


def _priority_history_features(
    request: EmergencyStepRequest,
    agent: FailedAgentSnapshot,
) -> np.ndarray:
    mission_ids = [item.mission_id for item in request.failed_agents]
    min_mission_id = min(mission_ids)
    max_mission_id = max(mission_ids)
    mission_span = max(1, max_mission_id - min_mission_id)
    mission_id_norm = (agent.mission_id - min_mission_id) / mission_span

    active_count = max(1, len(request.failed_agents))
    active_denominator = max(1, active_count - 1)
    round_robin_rank = (agent.mission_id - 1 - request.time_step) % active_count
    round_robin_rank_norm = round_robin_rank / active_denominator

    wait_streak_norm = min(_wait_streak(agent.recent_path), 10) / 10.0
    oscillation_streak_norm = min(_oscillation_streak(agent.recent_path), 5) / 5.0

    return np.asarray(
        [
            mission_id_norm,
            round_robin_rank_norm,
            wait_streak_norm,
            oscillation_streak_norm,
        ],
        dtype=np.float32,
    )


def _local_observation(
    request: EmergencyStepRequest,
    agent: FailedAgentSnapshot,
) -> np.ndarray:
    radius = request.observation_radius
    side = radius * 2 + 1
    center = agent.current_cell
    grid = np.zeros((LOCAL_CHANNELS, side, side, side), dtype=np.float32)

    occupied_cells = {
        other.current_cell
        for other in request.failed_agents
        if other.agent_id != agent.agent_id
    }
    failed_agent_ids = {item.agent_id for item in request.failed_agents}
    occupied_cells.update(
        neighbor.cell
        for neighbor in request.rid_neighbors
        if neighbor.agent_id not in failed_agent_ids
    )

    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            for dz in range(-radius, radius + 1):
                cell = (center[0] + dx, center[1] + dy, center[2] + dz)
                ix, iy, iz = dx + radius, dy + radius, dz + radius
                if not is_inside(request, cell):
                    grid[0, ix, iy, iz] = 1.0
                    continue
                if cell in request.blocked_cell_set:
                    grid[0, ix, iy, iz] = 1.0
                if is_no_fly(request, cell, request.time_step + 1):
                    grid[1, ix, iy, iz] = 1.0
                if cell in occupied_cells:
                    grid[2, ix, iy, iz] = 1.0
                if cell == agent.goal_cell:
                    grid[3, ix, iy, iz] = 1.0

    grid[4, radius, radius, radius] = 1.0
    return grid


def _wait_streak(path: tuple[tuple[int, int, int], ...]) -> int:
    if len(path) < 2:
        return 0
    streak = 0
    cursor = len(path) - 1
    while cursor > 0 and path[cursor] == path[cursor - 1]:
        streak += 1
        cursor -= 1
    return streak


def _oscillation_streak(path: tuple[tuple[int, int, int], ...]) -> int:
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


def _to_config(zone: NoFlyZoneSnapshot) -> TemporalNoFlyZoneConfig:
    return TemporalNoFlyZoneConfig(
        zone_id=zone.zone_id,
        min_cell=zone.min_cell,
        max_cell=zone.max_cell,
        start_time_step=zone.start_time_step,
        end_time_step=zone.end_time_step,
        enabled=zone.enabled,
    )
