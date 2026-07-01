from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


RUN_FIELDS = [
    "run_id",
    "timestamp",
    "actor_name",
    "map_type",
    "mission_seed",
    "city_seed",
    "random_seed",
    "execution_random_seed",
    "phase",
    "group_id",
    "group_name",
    "scenario_name",
    "recovery_mode",
    "service_mode",
    "response_modes",
    "trigger_time_step",
    "agent_count",
    "observation_radius",
    "max_time_steps",
    "action_horizon",
    "continuous_shadow_mode",
    "shadow_request_count",
    "request_interval_steps",
    "include_blocked_cells",
    "max_blocked_cells",
    "sent_requests",
    "responses",
    "failed_responses",
    "http_200_count",
    "http_error_count",
    "round_trip_avg_ms",
    "round_trip_min_ms",
    "round_trip_max_ms",
    "service_total_avg_ms",
    "service_policy_avg_ms",
    "service_model_forward_avg_ms",
    "service_obs_avg_ms",
    "service_safety_avg_ms",
    "action_count",
    "accepted_action_count",
    "substituted_action_count",
    "fallback_action_count",
    "fallback_wait_action_count",
    "not_applied_action_count",
    "ghost_reached",
    "ghost_total",
    "ghost_all_reached",
    "ghost_completed",
    "ghost_steps",
    "ghost_repaired_moves",
    "ghost_rejected_moves",
    "ghost_wait_moves",
    "first_error",
    "jsonl_path",
    "csv_path",
]


BENCHMARK_FIELDS = [
    "map_type",
    "recovery_mode",
    "num_runs",
    "success_rate",
    "mean_ghost_reached",
    "mean_ghost_steps",
    "mean_round_trip_ms",
    "mean_service_total_ms",
    "mean_repaired_moves",
    "mean_rejected_moves",
    "mean_wait_moves",
    "mean_fallback_actions",
    "failed_request_rate",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize UE MAPPO emergency JSONL logs into CSV tables."
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        required=True,
        help="Directory containing mappo_shadow_*.jsonl logs.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output CSV with one row per UE emergency run.",
    )
    parser.add_argument(
        "--benchmark-out",
        type=Path,
        default=None,
        help="Optional aggregate CSV grouped by map_type and recovery_mode.",
    )
    parser.add_argument(
        "--pattern",
        default="mappo_shadow_*.jsonl",
        help="Glob pattern for JSONL logs under --log-dir.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = [summarize_run(path) for path in sorted(args.log_dir.glob(args.pattern))]
    rows = [row for row in rows if row is not None]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_csv(args.out, RUN_FIELDS, rows)
    print(f"wrote {len(rows)} run summaries to {args.out}")

    if args.benchmark_out is not None:
        benchmark_rows = aggregate_benchmark(rows)
        args.benchmark_out.parent.mkdir(parents=True, exist_ok=True)
        write_csv(args.benchmark_out, BENCHMARK_FIELDS, benchmark_rows)
        print(f"wrote {len(benchmark_rows)} aggregate rows to {args.benchmark_out}")


def summarize_run(jsonl_path: Path) -> dict[str, Any] | None:
    events = read_jsonl(jsonl_path)
    if not events:
        return None

    response_events = [event for event in events if event.get("event") == "mappo_emergency_response"]
    ghost_steps = [event for event in events if event.get("event") == "mappo_emergency_ghost_step"]
    summaries = [event for event in events if event.get("event") == "mappo_emergency_summary"]
    finals = [event for event in events if event.get("event") == "mappo_emergency_ghost_final"]
    experiment_summaries = [
        event for event in events if event.get("event") == "structured_experiment_summary"
    ]

    summary = summaries[-1] if summaries else {}
    final = finals[-1] if finals else {}
    experiment = experiment_summaries[-1] if experiment_summaries else {}
    experiment_detail = experiment.get("summary") if isinstance(experiment.get("summary"), dict) else {}

    recovery_mode = first_non_empty(
        summary.get("recovery_mode"),
        final.get("recovery_mode"),
        *(event.get("recovery_mode") for event in events),
    )
    service_mode = first_non_empty(
        summary.get("service_mode"),
        final.get("service_mode"),
        *(event.get("service_mode") for event in events),
    )

    response_modes: set[str] = set()
    response_codes = Counter()
    service_totals: list[float] = []
    service_policy: list[float] = []
    service_forward: list[float] = []
    service_obs: list[float] = []
    service_safety: list[float] = []
    round_trips: list[float] = []
    action_status = Counter()
    first_error = ""
    first_time_step = ""
    first_action_horizon = ""
    agent_count = ""

    for event in response_events:
        response_codes[str(event.get("response_code", ""))] += 1
        round_trips.append(to_float(event.get("round_trip_ms")))
        if first_time_step == "":
            first_time_step = event.get("ue_time_step", "")

        response = event.get("response") or {}
        if isinstance(response, dict):
            mode = response.get("mode")
            if mode:
                response_modes.add(str(mode))
            if not first_error and response.get("error"):
                first_error = str(response.get("error"))

            first_action_horizon = first_action_horizon or response.get("action_horizon_length", "")
            actions = response.get("actions") or []
            if actions and agent_count == "":
                agent_count = len(actions)

            timing = response.get("timing_ms") or {}
            if timing:
                service_totals.append(to_float(timing.get("total")))
                service_policy.append(to_float(timing.get("policy_inference")))
                service_forward.append(to_float(timing.get("model_forward")))
                service_obs.append(to_float(timing.get("observation_build")))
                service_safety.append(to_float(timing.get("safety_filter")))

            for action in actions:
                status = str(action.get("safety_filter_status") or "")
                if status:
                    action_status[status] += 1
                if action.get("fallback_used"):
                    action_status["fallback_used"] += 1

    ghost_repaired = sum(int(step.get("repaired_ghost_moves", 0) or 0) for step in ghost_steps)
    ghost_rejected = sum(int(step.get("rejected_ghost_moves", 0) or 0) for step in ghost_steps)
    ghost_wait = sum(int(step.get("wait_moves", 0) or 0) for step in ghost_steps)

    filename_info = parse_filename(jsonl_path)
    csv_path = jsonl_path.with_suffix(".csv")
    run_id = first_non_empty(
        summary.get("run_id"),
        final.get("run_id"),
        *(event.get("run_id") for event in events),
        jsonl_path.stem,
    )

    row = {
        "run_id": run_id,
        "timestamp": first_non_empty(
            summary.get("timestamp"),
            final.get("timestamp"),
            filename_info.get("timestamp", ""),
        ),
        "actor_name": filename_info.get("actor_name", ""),
        "map_type": structured_value("map_type", experiment, experiment_detail, events),
        "mission_seed": first_non_empty(
            structured_value("mission_seed", experiment, experiment_detail, events),
            structured_value("execution_random_seed", experiment, experiment_detail, events),
            structured_value("random_seed", experiment, experiment_detail, events),
        ),
        "city_seed": structured_value("city_seed", experiment, experiment_detail, events),
        "random_seed": structured_value("random_seed", experiment, experiment_detail, events),
        "execution_random_seed": structured_value(
            "execution_random_seed", experiment, experiment_detail, events
        ),
        "phase": structured_value("phase", experiment, experiment_detail, events),
        "group_id": structured_value("group_id", experiment, experiment_detail, events),
        "group_name": structured_value("group_name", experiment, experiment_detail, events),
        "scenario_name": structured_value("scenario_name", experiment, experiment_detail, events),
        "recovery_mode": recovery_mode,
        "service_mode": service_mode,
        "response_modes": "|".join(sorted(response_modes)),
        "trigger_time_step": first_non_empty(summary.get("trigger_time_step"), first_time_step),
        "agent_count": first_non_empty(summary.get("agent_count"), agent_count),
        "observation_radius": first_non_empty(summary.get("observation_radius")),
        "max_time_steps": first_non_empty(summary.get("max_time_steps")),
        "action_horizon": first_non_empty(summary.get("action_horizon"), first_action_horizon),
        "continuous_shadow_mode": first_non_empty(summary.get("continuous_shadow_mode")),
        "shadow_request_count": first_non_empty(summary.get("shadow_request_count")),
        "request_interval_steps": first_non_empty(summary.get("request_interval_steps")),
        "include_blocked_cells": first_non_empty(summary.get("include_blocked_cells")),
        "max_blocked_cells": first_non_empty(summary.get("max_blocked_cells")),
        "sent_requests": first_non_empty(summary.get("sent_requests"), len(response_events)),
        "responses": first_non_empty(summary.get("responses"), len(response_events)),
        "failed_responses": first_non_empty(summary.get("failed_responses"), response_error_count(response_codes)),
        "http_200_count": response_codes.get("200", 0),
        "http_error_count": response_error_count(response_codes),
        "round_trip_avg_ms": fmt(first_non_empty(summary.get("round_trip_avg_ms"), mean(round_trips))),
        "round_trip_min_ms": fmt(first_non_empty(summary.get("round_trip_min_ms"), min_or_zero(round_trips))),
        "round_trip_max_ms": fmt(first_non_empty(summary.get("round_trip_max_ms"), max_or_zero(round_trips))),
        "service_total_avg_ms": fmt(first_non_empty(summary.get("service_total_avg_ms"), mean(service_totals))),
        "service_policy_avg_ms": fmt(mean(service_policy)),
        "service_model_forward_avg_ms": fmt(mean(service_forward)),
        "service_obs_avg_ms": fmt(mean(service_obs)),
        "service_safety_avg_ms": fmt(mean(service_safety)),
        "action_count": first_non_empty(summary.get("action_count"), sum_action_count(action_status)),
        "accepted_action_count": first_non_empty(summary.get("accepted_action_count"), action_status.get("accepted", 0)),
        "substituted_action_count": first_non_empty(summary.get("substituted_action_count"), action_status.get("substituted", 0)),
        "fallback_action_count": first_non_empty(summary.get("fallback_action_count"), action_status.get("fallback_used", 0)),
        "fallback_wait_action_count": action_status.get("fallback_wait", 0),
        "not_applied_action_count": action_status.get("not_applied", 0),
        "ghost_reached": first_non_empty(final.get("reached")),
        "ghost_total": first_non_empty(final.get("total")),
        "ghost_all_reached": first_non_empty(final.get("all_reached")),
        "ghost_completed": first_non_empty(final.get("completed")),
        "ghost_steps": first_non_empty(final.get("ghost_steps"), len(ghost_steps)),
        "ghost_repaired_moves": ghost_repaired,
        "ghost_rejected_moves": ghost_rejected,
        "ghost_wait_moves": ghost_wait,
        "first_error": first_error,
        "jsonl_path": str(jsonl_path),
        "csv_path": str(csv_path) if csv_path.exists() else "",
    }
    return row


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), start=1):
        clean = line.strip().lstrip("\ufeff")
        if not clean:
            continue
        try:
            event = json.loads(clean)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSONL line: {exc}") from exc
        if isinstance(event, dict):
            events.append(event)
    return events


def parse_filename(path: Path) -> dict[str, str]:
    match = re.match(r"mappo_shadow_(?P<actor>.+)_(?P<date>\d{8})_(?P<time>\d{6})$", path.stem)
    if not match:
        return {}
    return {
        "actor_name": match.group("actor"),
        "timestamp": f"{match.group('date')}_{match.group('time')}",
    }


def aggregate_benchmark(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[(str(row.get("map_type", "")), str(row.get("recovery_mode", "")))].append(row)

    aggregate_rows: list[dict[str, Any]] = []
    for (map_type, recovery_mode), group in sorted(groups.items()):
        num_runs = len(group)
        successes = sum(str(row.get("ghost_all_reached")).lower() == "true" for row in group)
        sent_requests = sum(to_float(row.get("sent_requests")) for row in group)
        failed_requests = sum(to_float(row.get("failed_responses")) for row in group)
        aggregate_rows.append(
            {
                "map_type": map_type,
                "recovery_mode": recovery_mode,
                "num_runs": num_runs,
                "success_rate": fmt(successes / num_runs if num_runs else 0.0),
                "mean_ghost_reached": fmt(mean_value(group, "ghost_reached")),
                "mean_ghost_steps": fmt(mean_value(group, "ghost_steps")),
                "mean_round_trip_ms": fmt(mean_value(group, "round_trip_avg_ms")),
                "mean_service_total_ms": fmt(mean_value(group, "service_total_avg_ms")),
                "mean_repaired_moves": fmt(mean_value(group, "ghost_repaired_moves")),
                "mean_rejected_moves": fmt(mean_value(group, "ghost_rejected_moves")),
                "mean_wait_moves": fmt(mean_value(group, "ghost_wait_moves")),
                "mean_fallback_actions": fmt(mean_value(group, "fallback_action_count")),
                "failed_request_rate": fmt(failed_requests / sent_requests if sent_requests else 0.0),
            }
        )
    return aggregate_rows


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def first_non_empty(*values: Any) -> Any:
    for value in values:
        if value is None:
            continue
        if value == "":
            continue
        return value
    return ""


def structured_value(
    key: str,
    experiment: dict[str, Any],
    experiment_detail: dict[str, Any],
    events: list[dict[str, Any]],
) -> Any:
    return first_non_empty(
        experiment.get(key),
        experiment_detail.get(key),
        *(event.get(key) for event in events),
    )


def to_float(value: Any) -> float:
    if value in (None, ""):
        return 0.0
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def mean_value(rows: list[dict[str, Any]], key: str) -> float:
    values = [to_float(row.get(key)) for row in rows if row.get(key) not in (None, "")]
    return mean(values)


def min_or_zero(values: list[float]) -> float:
    return min(values) if values else 0.0


def max_or_zero(values: list[float]) -> float:
    return max(values) if values else 0.0


def fmt(value: Any) -> str:
    return f"{to_float(value):.6f}"


def response_error_count(codes: Counter[str]) -> int:
    total = 0
    for code, count in codes.items():
        try:
            numeric = int(code)
        except ValueError:
            total += count
            continue
        if numeric < 200 or numeric >= 300:
            total += count
    return total


def sum_action_count(statuses: Counter[str]) -> int:
    return sum(
        count
        for status, count in statuses.items()
        if status not in {"fallback_used"}
    )


if __name__ == "__main__":
    main()
