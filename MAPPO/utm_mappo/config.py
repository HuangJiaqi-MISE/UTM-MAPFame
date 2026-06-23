from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

Cell = tuple[int, int, int]


def _cell(value: Any, name: str) -> Cell:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must be a 3-item cell coordinate")
    return int(value[0]), int(value[1]), int(value[2])


@dataclass(frozen=True)
class GridConfig:
    dimensions: Cell = (20, 20, 6)
    blocked_cells: tuple[Cell, ...] = ()

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "GridConfig":
        return cls(
            dimensions=_cell(data.get("dimensions", (20, 20, 6)), "grid.dimensions"),
            blocked_cells=tuple(
                _cell(cell, "grid.blocked_cells[]")
                for cell in data.get("blocked_cells", [])
            ),
        )


@dataclass(frozen=True)
class MissionConfig:
    mission_id: int
    start: Cell
    goal: Cell
    protection_xy_radius: int = 1
    protection_z_up: int = 0
    protection_z_down: int = 0
    downwash_xy_radius: int = 0
    downwash_z_below: int = 0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MissionConfig":
        return cls(
            mission_id=int(data["mission_id"]),
            start=_cell(data["start"], "mission.start"),
            goal=_cell(data["goal"], "mission.goal"),
            protection_xy_radius=max(0, int(data.get("protection_xy_radius", 1))),
            protection_z_up=max(0, int(data.get("protection_z_up", 0))),
            protection_z_down=max(0, int(data.get("protection_z_down", 0))),
            downwash_xy_radius=max(0, int(data.get("downwash_xy_radius", 0))),
            downwash_z_below=max(0, int(data.get("downwash_z_below", 0))),
        )


@dataclass(frozen=True)
class TemporalNoFlyZoneConfig:
    zone_id: int
    min_cell: Cell
    max_cell: Cell
    start_time_step: int = 0
    end_time_step: int = 0
    enabled: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TemporalNoFlyZoneConfig":
        min_cell = _cell(data["min_cell"], "no_fly_zone.min_cell")
        max_cell = _cell(data["max_cell"], "no_fly_zone.max_cell")
        start = max(0, int(data.get("start_time_step", 0)))
        end = max(start, int(data.get("end_time_step", start)))
        return cls(
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
            start_time_step=start,
            end_time_step=end,
        )


@dataclass(frozen=True)
class UTMScenario:
    grid: GridConfig
    missions: tuple[MissionConfig, ...]
    no_fly_zones: tuple[TemporalNoFlyZoneConfig, ...] = ()
    max_time_steps: int = 256
    observation_radius: int = 2

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "UTMScenario":
        missions = tuple(MissionConfig.from_dict(item) for item in data["missions"])
        if not missions:
            raise ValueError("scenario must include at least one mission")

        mission_ids = [mission.mission_id for mission in missions]
        if len(mission_ids) != len(set(mission_ids)):
            raise ValueError("mission ids must be unique")

        return cls(
            grid=GridConfig.from_dict(data.get("grid", {})),
            missions=missions,
            no_fly_zones=tuple(
                TemporalNoFlyZoneConfig.from_dict(item)
                for item in data.get("no_fly_zones", [])
            ),
            max_time_steps=max(1, int(data.get("max_time_steps", 256))),
            observation_radius=max(1, int(data.get("observation_radius", 2))),
        )

    @classmethod
    def from_yaml(cls, path: str | Path) -> "UTMScenario":
        with Path(path).open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        return cls.from_dict(data)

    def to_dict(self) -> dict[str, Any]:
        return {
            "grid": {
                "dimensions": list(self.grid.dimensions),
                "blocked_cells": [list(cell) for cell in self.grid.blocked_cells],
            },
            "missions": [
                {
                    "mission_id": mission.mission_id,
                    "start": list(mission.start),
                    "goal": list(mission.goal),
                    "protection_xy_radius": mission.protection_xy_radius,
                    "protection_z_up": mission.protection_z_up,
                    "protection_z_down": mission.protection_z_down,
                    "downwash_xy_radius": mission.downwash_xy_radius,
                    "downwash_z_below": mission.downwash_z_below,
                }
                for mission in self.missions
            ],
            "no_fly_zones": [
                {
                    "zone_id": zone.zone_id,
                    "enabled": zone.enabled,
                    "min_cell": list(zone.min_cell),
                    "max_cell": list(zone.max_cell),
                    "start_time_step": zone.start_time_step,
                    "end_time_step": zone.end_time_step,
                }
                for zone in self.no_fly_zones
            ],
            "max_time_steps": self.max_time_steps,
            "observation_radius": self.observation_radius,
        }


@dataclass
class AgentRuntimeState:
    mission: MissionConfig
    cell: Cell
    previous_cell: Cell
    reached_goal: bool = False
    last_action: int = 0
    path: list[Cell] = field(default_factory=list)

    @property
    def agent_name(self) -> str:
        return f"agent_{self.mission.mission_id}"
