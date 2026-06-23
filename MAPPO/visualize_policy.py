from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle

from utm_mappo import UTMMAPFEnv, UTMScenario
from utm_mappo.mappo import DiscreteMAPPO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize a trained UTM MAPPO policy.")
    parser.add_argument(
        "scenario",
        type=Path,
        nargs="?",
        default=Path("configs/crossing.yaml"),
        help="Path to a UTM scenario YAML file.",
    )
    parser.add_argument("--model-dir", type=Path, default=Path("models/crossing"))
    parser.add_argument("--output", type=Path, default=Path("runs/crossing.gif"))
    parser.add_argument("--fps", type=int, default=4)
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def collect_rollout(
    env: UTMMAPFEnv,
    model: DiscreteMAPPO,
    max_steps: int = 0,
) -> list[dict[str, Any]]:
    observations, _ = env.reset()
    frames = [env.render()]
    steps = 0
    limit = max_steps if max_steps > 0 else env.scenario.max_time_steps

    while observations and steps < limit:
        actions = model.predict(observations, deterministic=True)
        observations, _, _, _, _ = env.step(actions)
        frames.append(env.render())
        steps += 1

    return frames


def draw_grid(ax: Axes, env: UTMMAPFEnv, z: int, frame: dict[str, Any]) -> None:
    width, height, _ = env.dimensions
    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(-0.5, height - 0.5)
    ax.set_aspect("equal")
    ax.set_xticks(range(width))
    ax.set_yticks(range(height))
    ax.grid(True, color="#dddddd", linewidth=0.6)
    ax.set_title(f"z={z}")

    for x in range(width):
        for y in range(height):
            if env.occupancy[x, y, z]:
                ax.add_patch(
                    Rectangle(
                        (x - 0.5, y - 0.5),
                        1,
                        1,
                        facecolor="#444444",
                        alpha=0.45,
                    )
                )

    time_step = int(frame["time_step"])
    for zone in env.scenario.no_fly_zones:
        if not zone.enabled:
            continue
        if not (zone.start_time_step <= time_step <= zone.end_time_step):
            continue
        if not (zone.min_cell[2] <= z <= zone.max_cell[2]):
            continue
        x0, y0, _ = zone.min_cell
        x1, y1, _ = zone.max_cell
        ax.add_patch(
            Rectangle(
                (x0 - 0.5, y0 - 0.5),
                x1 - x0 + 1,
                y1 - y0 + 1,
                facecolor="#ff3333",
                alpha=0.25,
                edgecolor="#cc0000",
            )
        )


def draw_agents(ax: Axes, z: int, frame: dict[str, Any]) -> None:
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
    for index, (agent, state) in enumerate(sorted(frame["agents"].items())):
        color = colors[index % len(colors)]
        cell = tuple(state["cell"])
        goal = tuple(state["goal"])
        mission_id = state["mission_id"]

        if goal[2] == z:
            ax.scatter(
                goal[0],
                goal[1],
                marker="*",
                s=180,
                color=color,
                edgecolor="black",
                linewidth=0.5,
                alpha=0.6,
            )

        if cell[2] != z:
            continue

        ax.scatter(
            cell[0],
            cell[1],
            marker="o",
            s=170,
            color=color,
            edgecolor="black",
            linewidth=1.0,
        )
        ax.text(
            cell[0],
            cell[1],
            str(mission_id),
            ha="center",
            va="center",
            color="white",
            fontsize=9,
            fontweight="bold",
        )


def render_animation(
    env: UTMMAPFEnv,
    frames: list[dict[str, Any]],
    output: Path,
    fps: int,
    show: bool,
) -> None:
    z_count = env.dimensions[2]
    fig, axes = plt.subplots(1, z_count, figsize=(4.5 * z_count, 4.8), squeeze=False)
    flat_axes = list(axes[0])

    def update(frame_index: int) -> list[Any]:
        frame = frames[frame_index]
        for z, ax in enumerate(flat_axes):
            ax.clear()
            draw_grid(ax, env, z, frame)
            draw_agents(ax, z, frame)
        fig.suptitle(f"UTM MAPPO policy rollout, t={frame['time_step']}")
        return []

    animation = FuncAnimation(
        fig,
        update,
        frames=len(frames),
        interval=max(1, int(1000 / max(1, fps))),
        blit=False,
        repeat=False,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=fps))
    if show:
        plt.show()
    plt.close(fig)


def main() -> None:
    args = parse_args()
    scenario = UTMScenario.from_yaml(args.scenario)
    env = UTMMAPFEnv(scenario)
    model = DiscreteMAPPO.load(env, args.model_dir)
    frames = collect_rollout(env, model, max_steps=args.max_steps)
    render_animation(env, frames, args.output, fps=args.fps, show=args.show)
    print(f"saved visualization to {args.output}")


if __name__ == "__main__":
    main()
