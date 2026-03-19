# UTM-MAPFame

## Overview

`UTM-MAPFame` is a 3D drone path planning and airspace traffic management simulation system built with Unreal Engine and C++.

It is intended for research and experimentation in the following areas:

- 3D grid-based environment modeling and obstacle avoidance
- Single-drone path planning
- Multi-drone cooperative path planning
- Time-dependent no-fly zone modeling
- Execution delay, conflict prediction, and replanning studies

## Key Features

- Generates a 3D occupancy grid from an Unreal Engine scene
- Supports single-agent planning with `A*`, `D* Lite`, and `SIPP`
- Supports multi-agent planning with `CBS`, `ECBS`, `PBS`, `LaCAM`, and `LaCAM-UTM`
- Supports temporal no-fly zone modeling
- Models drone protected volume and downwash risk
- Simulates centralized discrete-time execution
- Supports delay injection, plan-execution synchronization, conflict prediction, and replanning
- Exports experiment statistics and structured JSON logs

## Repository Structure

```text
UTM-MAPFame/
├─ Public/
│  ├─ Actors/        # Public Actor headers
│  └─ Planning/      # Public planners, data structures, and interfaces
├─ Private/
│  ├─ Actors/        # Actor implementations
│  └─ Planning/      # Planning algorithm implementations
├─ UTM.Build.cs      # Unreal module build configuration
├─ UTM.cpp           # Module entry point
└─ UTMGameModeBase.* # Base GameMode
```

The main entry point is `APathPlanningDemoActor`, which coordinates mapping, planning, execution, visualization, and experiment statistics.

## Requirements

- Unreal Engine project with C++ module support
- A host project that can load the `UTM` module
- A scene suitable for grid generation and mission setup

## Installation

### 1. Add the Module to a Host Unreal Project

Copy this directory into the host project's `Source/UTM/` folder and ensure the project recognizes it as a valid Unreal module.

### 2. Generate Project Files and Build

A typical workflow looks like this:

```powershell
GenerateProjectFiles.bat <Project>.uproject
Build.bat <Project>Editor Win64 Development <Project>.uproject
```

### 3. Open the Editor and Place the Main Controller Actor

Place `APathPlanningDemoActor` in the level. This actor serves as the top-level controller for the full experiment pipeline.

## Quick Start

### 1. Configure the Grid

Set the following parameters before running experiments:

- `GridOrigin`
- `GridDim`
- `CellSize`
- `PlannerType`

### 2. Configure Missions

Mission definitions can be provided in either of these ways:

- Use scene Actors tagged as `Start_i` and `Goal_i`
- Set start and goal positions directly in `MissionConfigs`

### 3. Run Planning

You can start planning in either of these ways:

- Enable `bAutoRunPlanningOnBeginPlay`
- Call `RunPlanning()` from the editor or through Blueprint

After execution, you can inspect:

- Debug path visualization
- Drone spawning and execution behavior
- Planning and execution statistics in the log output

## Typical Workflow

1. Create or load an urban obstacle environment
2. Build the 3D grid map
3. Configure mission start and goal states
4. Select a planning algorithm
5. Run path planning
6. Enable centralized execution simulation
7. Observe delays, conflicts, synchronization, and replanning behavior
8. Review logs and exported JSON statistics

## Supported Planners

| Category | Planner | Description |
|---|---|---|
| Single-agent | A* | Baseline 3D static path planner |
| Single-agent | D* Lite | Heuristic-based dynamic replanning algorithm |
| Single-agent | SIPP | Safe Interval Path Planning with support for temporal no-fly zones |
| Multi-agent | CBS | Classic Conflict-Based Search |
| Multi-agent | ECBS | Bounded-suboptimal variant of CBS with improved efficiency |
| Multi-agent | PBS | Priority-Based Search for multi-agent planning |
| Multi-agent | LaCAM | Scalable MAPF planner for larger problem instances |
| Multi-agent | LaCAM-UTM (ours) | Extended LaCAM with protected volume, downwash, and no-fly-zone constraints |

## Core Modules

### Map Representation

`FGridMap3D` is responsible for:

- Converting between world coordinates and grid coordinates
- Detecting scene obstacles
- Generating the occupancy grid

### Missions and Constraints

`FDroneMissionConfig` defines mission parameters, including:

- Start and goal locations
- Protected volume radius
- Vertical safety expansion
- Downwash risk zone

`FTemporalNoFlyZoneConfig` defines temporal no-fly zones, including:

- Spatial extent
- Active time windows

### Execution and Synchronization

The system supports:

- Discrete-time execution
- Randomized or scripted delays
- Plan-execution synchronization
- Hold actions after predicted conflicts
- Local or global replanning

This makes the framework suitable not only for path generation, but also for evaluating whether planned trajectories remain safe and feasible during execution.

## Outputs

The system produces both visual and logged outputs.

### Visualization

- Grid debug rendering
- Path lines and waypoints
- Mission markers and no-fly-zone markers
- Drone execution positions and debug text

### Logs and Statistics

- Total planning time
- Per-mission success status
- Path length
- Execution delay statistics
- Vertex and edge conflict statistics
- Replanning counts
- Structured experiment JSON output

## Notes

- `JPS` currently exists only as an enum placeholder and falls back to `A*`
- Multi-agent execution experiments primarily rely on the centralized execution framework implemented in `APathPlanningDemoActor`

## Suggested Use Cases

This project is suitable for:

- Research prototypes for UAV traffic management
- Experiments on multi-agent path finding in 3D environments
- Studies of execution-time uncertainty and safety constraints
- Benchmarking centralized planning and replanning strategies

## Future Work

Potential extensions include:

- Additional decentralized execution models
- More realistic vehicle dynamics and control constraints
- Richer conflict resolution strategies
- Expanded experiment configuration and benchmark tooling

## Citation

If you use this project in academic work, cite the corresponding paper, thesis, or project documentation when available.


## License

This project is licensed under the GNU General Public License v3.0. See the `LICENSE` file for details.