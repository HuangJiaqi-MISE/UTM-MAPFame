from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

from .emergency_service import EmergencyRecoveryService


DEFAULT_MODEL_DIR = Path("models/utm8_100x100_obs_mappo_ft2_deadlock")
DEFAULT_REQUEST = Path("integration/demo_request.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run UE-MAPPO emergency integration smoke tests."
    )
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--request", type=Path, default=DEFAULT_REQUEST)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--write-requests",
        type=Path,
        default=None,
        help="Optional directory for writing generated conflict request JSON files.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base = json.loads(args.request.read_text(encoding="utf-8"))

    run_mode_smoke_tests(base, args.model_dir, args.device)
    run_conflict_smoke_tests(base, args.model_dir, args.device)

    if args.write_requests is not None:
        write_generated_requests(base, args.write_requests)


def run_mode_smoke_tests(base: dict, model_dir: Path, device: str) -> None:
    for mode in ("no_recovery", "rule", "mappo", "mappo_shield"):
        service = EmergencyRecoveryService(
            model_dir=model_dir if mode in ("mappo", "mappo_shield") else None,
            mode=mode,
            device=device,
        )
        response = service.step(copy.deepcopy(base))
        assert response["observation_shape"] == [8, 645]
        assert len(response["actions"]) == 8
        print(
            f"mode={mode} ok total_ms={response['timing_ms']['total']:.4f}"
        )


def run_conflict_smoke_tests(base: dict, model_dir: Path, device: str) -> None:
    service = EmergencyRecoveryService(
        model_dir=model_dir,
        mode="mappo_shield",
        device=device,
    )
    substitution_cases = {
        key: value
        for key, value in generated_conflict_requests(base).items()
        if key != "no_fly_block"
    }
    for name, payload in substitution_cases.items():
        response = service.step(payload)
        target = next(
            item for item in response["actions"] if item["agent_id"] == "uav_01"
        )
        assert target["raw_policy_action"] == "+X", (
            f"{name}: expected demo policy to prefer +X for uav_01, "
            f"got {target['raw_policy_action']}"
        )
        assert target["fallback_used"], f"{name}: expected safety fallback"
        assert target["selected_action"] != target["raw_policy_action"], (
            f"{name}: selected action should differ from rejected raw action"
        )
        assert target["safety_filter_status"] in ("substituted", "fallback_wait")
        print(
            f"case={name} ok raw={target['raw_policy_action']} "
            f"selected={target['selected_action']} "
            f"status={target['safety_filter_status']} "
            f"reason={target['reject_reason']}"
        )

    response = service.step(no_fly_block(base))
    target = next(item for item in response["actions"] if item["agent_id"] == "uav_01")
    assert target["selected_action"] != "+X", (
        "no_fly_block: +X enters a next-step no-fly cell and must not be selected"
    )
    assert target["safety_filter_status"] in (
        "accepted",
        "substituted",
        "fallback_wait",
    )
    print(
        "case=no_fly_block ok "
        f"raw={target['raw_policy_action']} "
        f"selected={target['selected_action']} "
        f"status={target['safety_filter_status']} "
        "note=blocked by action mask or safety filter"
    )


def generated_conflict_requests(base: dict) -> dict[str, dict]:
    cases = {
        "rid_occupied_block": rid_occupied_block(base),
        "no_fly_block": no_fly_block(base),
        "downwash_block": downwash_block(base),
        "failed_occupancy_block": failed_occupancy_block(base),
        "swap_risk_block": swap_risk_block(base),
    }
    return cases


def rid_occupied_block(base: dict) -> dict:
    payload = copy.deepcopy(base)
    payload["episode_id"] = "smoke_rid_occupied_block"
    payload["rid_neighbors"] = [
        {
            "agent_id": "normal_blocker",
            "cell": [11, 10, 3],
            "is_failed_agent": False,
        }
    ]
    return payload


def no_fly_block(base: dict) -> dict:
    payload = copy.deepcopy(base)
    payload["episode_id"] = "smoke_no_fly_block"
    payload["no_fly_zones"] = [
        {
            "zone_id": 99,
            "min_cell": [11, 10, 3],
            "max_cell": [11, 10, 3],
            "start_time_step": 33,
            "end_time_step": 33,
            "enabled": True,
        }
    ]
    return payload


def downwash_block(base: dict) -> dict:
    payload = copy.deepcopy(base)
    payload["episode_id"] = "smoke_downwash_block"
    payload["rid_neighbors"] = [
        {
            "agent_id": "normal_above_blocker",
            "cell": [11, 10, 4],
            "is_failed_agent": False,
        }
    ]
    return payload


def failed_occupancy_block(base: dict) -> dict:
    payload = copy.deepcopy(base)
    payload["episode_id"] = "smoke_failed_occupancy_block"
    agent = payload["failed_agents"][1]
    agent["current_cell"] = [11, 10, 3]
    agent["previous_cell"] = [12, 10, 3]
    agent["goal_cell"] = [0, 10, 3]
    agent["last_action"] = "-X"
    agent["recent_path"] = [[13, 10, 3], [12, 10, 3], [11, 10, 3]]
    payload["rid_neighbors"] = []
    return payload


def swap_risk_block(base: dict) -> dict:
    payload = failed_occupancy_block(base)
    payload["episode_id"] = "smoke_swap_risk_block"
    # The current greedy shield treats a proposed move into an uncommitted
    # failed agent's current cell as unsafe. This conservatively covers the
    # same local pattern that could become an edge swap in a joint executor.
    return payload


def write_generated_requests(base: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, payload in generated_conflict_requests(base).items():
        path = out_dir / f"{name}.json"
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
