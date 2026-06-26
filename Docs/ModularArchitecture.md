# UTM-MAPFame Modular Architecture

This document defines the target module boundaries for turning UTM-MAPFame into
a reusable MAPF/UTM research platform. The immediate goal is to let researchers
replace algorithms without editing the Unreal scene controller.

## Current Problem

`APathPlanningDemoActor` currently coordinates scene input, grid construction,
planner selection, mission preparation, execution simulation, replanning,
visualization, editor tools, and experiment logging. This is convenient for a
demo, but it makes algorithm research harder because new methods must touch a
large Unreal Actor instead of a small algorithm-facing interface.

## Target Boundaries

### Planner Registry

Owns planner selection and construction. Adding a new planner should require:

1. Implementing `IPathPlannerBase` or `IMultiAgentPlannerBase`.
2. Adding one registry entry in `FPlannerRegistry`.
3. Exposing only the required runtime config fields.

Implemented first in:

- `Source/UTM/Public/Planning/PlannerTypes.h`
- `Source/UTM/Public/Planning/PlannerRegistry.h`
- `Source/UTM/Private/Planning/PlannerRegistry.cpp`

### Task Assignment

Target role: convert available tasks, agent states, and policy constraints into
mission assignments. This should become a standalone module similar to the
LORR start kit scheduler.

Candidate interface:

- Input: current agent states, open task pool, grid, no-fly constraints.
- Output: assigned `FDroneMissionConfig` list or per-agent task ids.
- Default implementation: static mission list, preserving current behavior.

Initial static scheduler extraction is implemented in:

- `Source/UTM/Public/Planning/MissionSchedulerTypes.h`
- `Source/UTM/Public/Planning/MissionSchedulerRegistry.h`
- `Source/UTM/Private/Planning/MissionSchedulerRegistry.cpp`

The editor-facing `MissionConfigs` array remains the default mission source.
`APathPlanningDemoActor` now adapts raw missions through
`BuildScheduledMissionConfigs()` before invoking multi-agent planners.

Available task assignment strategies:

- `Static Scheduler`: forwards `MissionConfigs` unchanged. This preserves the
  legacy EUW workflow.
- `Nearest-First Scheduler`: treats mission starts as current agent positions
  and mission goals as open tasks. It visits agents by `MissionId` and greedily
  assigns the nearest remaining goal to each agent.

### Execution Coordinator

Target role: own staged execution, delay handling, conflict-aware hold decisions,
and replan requests. This should separate algorithmic execution policy from
drone visualization.

Candidate interface:

- Input: planned cell paths, current observed cells, delay model, safety model.
- Output: per-agent commands such as advance, hold, recover, or replan.
- Default implementation: the current centralized execution/alignment logic.

### Experiment Logging

Target role: produce structured metrics independently of visualization and
editor tools. This should make batch experiments and paper figures easier to
generate without reading Unreal log text.

### Visualization And Editor Tools

Target role: stay in Actor/Blueprint-facing code. This layer should consume
planner, scheduler, executor, and logging outputs, not own their core policy.

## Migration Plan

1. Extract planner registry and runtime config. This is the first completed
   step and keeps current behavior unchanged.
2. Extract mission/task source and static scheduler while keeping
   `MissionConfigs` as the default source. The static scheduler layer is now in
   place; future work can add dynamic assignment policies behind the same
   registry.
3. Extract centralized execution state and alignment policy into an executor
   class.
4. Move structured JSON generation into an experiment reporter.
5. Keep `APathPlanningDemoActor` as orchestration plus editor/visualization
   facade only.
