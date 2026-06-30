from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from typing import Any

from utm_mappo.config import Cell
from utm_mappo.env import UTMAction


ACTION_NAMES = ("WAIT", "+X", "-X", "+Y", "-Y", "+Z", "-Z")
ACTION_TO_NAME = {int(action): ACTION_NAMES[int(action)] for action in UTMAction}
ACTION_NAME_TO_INDEX = {name: index for index, name in enumerate(ACTION_NAMES)}


@dataclass(frozen=True)
class FailedAgentSnapshot:
    agent_id: str
    mission_id: int
    current_cell: Cell
    previous_cell: Cell
    goal_cell: Cell
    last_action: int
    recent_path: tuple[Cell, ...]
    consecutive_wait_count: int = 0
    consecutive_filter_reject_count: int = 0

    @property
    def internal_name(self) -> str:
        return f"agent_{self.mission_id}"


@dataclass(frozen=True)
class RIDNeighborSnapshot:
    agent_id: str
    cell: Cell
    is_failed_agent: bool = False


@dataclass(frozen=True)
class NoFlyZoneSnapshot:
    zone_id: int
    min_cell: Cell
    max_cell: Cell
    start_time_step: int
    end_time_step: int
    enabled: bool = True


@dataclass(frozen=True)
class EmergencyStepRequest:
    episode_id: str
    time_step: int
    grid_dimensions: Cell
    max_time_steps: int
    observation_radius: int
    failed_agents: tuple[FailedAgentSnapshot, ...]
    rid_neighbors: tuple[RIDNeighborSnapshot, ...]
    blocked_cells: tuple[Cell, ...]
    no_fly_zones: tuple[NoFlyZoneSnapshot, ...]

    @cached_property
    def blocked_cell_set(self) -> frozenset[Cell]:
        return frozenset(self.blocked_cells)


def parse_request(data: dict[str, Any]) -> EmergencyStepRequest:
    failed_agents = tuple(
        _parse_failed_agent(item) for item in data.get("failed_agents", [])
    )
    if len(failed_agents) != 8:
        raise ValueError(
            f"expected exactly 8 failed agents, got {len(failed_agents)}"
        )

    mission_ids = [agent.mission_id for agent in failed_agents]
    if sorted(mission_ids) != list(range(1, len(failed_agents) + 1)):
        raise ValueError(
            "failed agent mission_id values must be the contiguous range 1..8 "
            "for the current trained MAPPO checkpoints"
        )

    blocked_cells = data.get("blocked_cells")
    if blocked_cells is None:
        blocked_cells = data.get("blocked_cells_local_or_global", [])

    return EmergencyStepRequest(
        episode_id=str(data.get("episode_id", "")),
        time_step=int(data.get("time_step", 0)),
        grid_dimensions=_cell(data.get("grid_dimensions", (100, 100, 10))),
        max_time_steps=max(1, int(data.get("max_time_steps", 240))),
        observation_radius=max(1, int(data.get("observation_radius", 2))),
        failed_agents=tuple(sorted(failed_agents, key=lambda agent: agent.mission_id)),
        rid_neighbors=tuple(
            _parse_neighbor(item) for item in data.get("rid_neighbors", [])
        ),
        blocked_cells=tuple(_cell(item) for item in blocked_cells),
        no_fly_zones=tuple(
            _parse_no_fly_zone(item) for item in data.get("no_fly_zones", [])
        ),
    )


def action_name(action: int) -> str:
    if 0 <= int(action) < len(ACTION_NAMES):
        return ACTION_NAMES[int(action)]
    return "WAIT"


def action_index(value: str | int) -> int:
    if isinstance(value, int):
        if 0 <= value < len(ACTION_NAMES):
            return value
        return int(UTMAction.WAIT)
    return ACTION_NAME_TO_INDEX.get(value, int(UTMAction.WAIT))


def _parse_failed_agent(data: dict[str, Any]) -> FailedAgentSnapshot:
    current = _cell(data["current_cell"])
    previous = _cell(data.get("previous_cell", current))
    recent_path = tuple(_cell(item) for item in data.get("recent_path", [current]))
    if not recent_path:
        recent_path = (current,)
    if recent_path[-1] != current:
        recent_path = (*recent_path, current)

    return FailedAgentSnapshot(
        agent_id=str(data["agent_id"]),
        mission_id=int(data["mission_id"]),
        current_cell=current,
        previous_cell=previous,
        goal_cell=_cell(data["goal_cell"]),
        last_action=action_index(data.get("last_action", "WAIT")),
        recent_path=recent_path,
        consecutive_wait_count=int(data.get("consecutive_wait_count", 0)),
        consecutive_filter_reject_count=int(
            data.get("consecutive_filter_reject_count", 0)
        ),
    )


def _parse_neighbor(data: dict[str, Any]) -> RIDNeighborSnapshot:
    return RIDNeighborSnapshot(
        agent_id=str(data["agent_id"]),
        cell=_cell(data["cell"]),
        is_failed_agent=bool(data.get("is_failed_agent", False)),
    )


def _parse_no_fly_zone(data: dict[str, Any]) -> NoFlyZoneSnapshot:
    min_cell = _cell(data["min_cell"])
    max_cell = _cell(data["max_cell"])
    return NoFlyZoneSnapshot(
        zone_id=int(data.get("zone_id", 1)),
        enabled=bool(data.get("enabled", True)),
        min_cell=(
            min(min_cell[0], max_cell[0]),
            min(min_cell[1], max_cell[1]),
            min(min_cell[2], max_cell[2]),
        ),
        max_cell=(
            max(min_cell[0], max_cell[0]),
            max(min_cell[1], max_cell[1]),
            max(min_cell[2], max_cell[2]),
        ),
        start_time_step=max(0, int(data.get("start_time_step", 0))),
        end_time_step=max(0, int(data.get("end_time_step", 0))),
    )


def _cell(value: Any) -> Cell:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"cell must be a 3-item coordinate, got {value!r}")
    return int(value[0]), int(value[1]), int(value[2])
