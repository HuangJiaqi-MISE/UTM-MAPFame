from __future__ import annotations

import argparse
import json
import time
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any

import numpy as np

try:
    from .observation_builder import build_env_for_model, build_observations
    from .safety_filter import (
        decisions_to_response_items,
        policy_action_rankings,
        select_no_recovery_actions,
        select_raw_mappo_actions,
        select_rule_based_actions,
        select_shielded_actions,
    )
    from .schemas import ACTION_NAMES, EmergencyStepRequest, FailedAgentSnapshot, parse_request
    from utm_mappo.env import ACTION_DELTAS, UTMAction
    from utm_mappo.geometry import add_cell
except ImportError:  # Allows `python integration/emergency_service.py`.
    from integration.observation_builder import build_env_for_model, build_observations
    from integration.safety_filter import (
        decisions_to_response_items,
        policy_action_rankings,
        select_no_recovery_actions,
        select_raw_mappo_actions,
        select_rule_based_actions,
        select_shielded_actions,
    )
    from integration.schemas import ACTION_NAMES, EmergencyStepRequest, FailedAgentSnapshot, parse_request
    from utm_mappo.env import ACTION_DELTAS, UTMAction
    from utm_mappo.geometry import add_cell


MODES = ("no_recovery", "rule", "mappo", "mappo_shield")


class EmergencyRecoveryService:
    def __init__(
        self,
        model_dir: Path | None,
        mode: str,
        device: str = "auto",
    ) -> None:
        if mode not in MODES:
            raise ValueError(f"unsupported mode {mode!r}; expected one of {MODES}")
        if mode in ("mappo", "mappo_shield") and model_dir is None:
            raise ValueError(f"--model-dir is required for mode {mode}")

        self.model_dir = model_dir
        self.mode = mode
        self.device = device
        self._model_cache: dict[tuple[int, tuple[int, ...]], Any] = {}

    def step(self, payload: dict[str, Any]) -> dict[str, Any]:
        started = time.perf_counter()
        request = parse_request(payload)
        action_horizon = max(1, min(30, int(payload.get("action_horizon", 1))))

        horizon_steps = []
        total_observation_ms = 0.0
        total_inference_ms = 0.0
        total_model_load_ms = 0.0
        total_model_forward_ms = 0.0
        total_filter_ms = 0.0
        first_shape: list[int] | None = None
        first_actions: list[dict[str, object]] | None = None

        rollout_request = request
        for step_offset in range(action_horizon):
            (
                decisions,
                policy_probs,
                observation_shape,
                observation_ms,
                inference_ms,
                model_load_ms,
                model_forward_ms,
                filter_ms,
            ) = self._decide_once(rollout_request)

            actions = decisions_to_response_items(decisions, policy_probs)
            if first_actions is None:
                first_actions = actions
                first_shape = observation_shape

            horizon_steps.append(
                {
                    "step_offset": step_offset,
                    "time_step": rollout_request.time_step,
                    "actions": actions,
                }
            )

            total_observation_ms += observation_ms
            total_inference_ms += inference_ms
            total_model_load_ms += model_load_ms
            total_model_forward_ms += model_forward_ms
            total_filter_ms += filter_ms
            rollout_request = _advance_request(rollout_request, decisions)

        return {
            "episode_id": request.episode_id,
            "time_step": request.time_step,
            "mode": self.mode,
            "action_horizon_length": action_horizon,
            "observation_shape": first_shape or [0, 0],
            "actions": first_actions or [],
            "action_horizon": horizon_steps,
            "timing_ms": {
                "observation_build": round(total_observation_ms, 4),
                "model_load_or_cache": round(total_model_load_ms, 4),
                "model_forward": round(total_model_forward_ms, 4),
                "policy_inference": round(total_inference_ms, 4),
                "safety_filter": round(total_filter_ms, 4),
                "total": round(_elapsed_ms(started), 4),
            },
        }

    def _decide_once(
        self,
        request: EmergencyStepRequest,
    ) -> tuple[
        list[Any],
        dict[str, list[float]] | None,
        list[int],
        float,
        float,
        float,
        float,
        float,
    ]:
        obs_started = time.perf_counter()
        built = build_observations(request)
        observation_ms = _elapsed_ms(obs_started)

        policy_probs: dict[str, list[float]] | None = None
        inference_ms = 0.0
        model_load_ms = 0.0
        model_forward_ms = 0.0

        if self.mode in ("mappo", "mappo_shield"):
            inference_started = time.perf_counter()
            policy_probs, model_load_ms, model_forward_ms = self._predict(
                request,
                built.observations,
                built.action_masks,
            )
            inference_ms = _elapsed_ms(inference_started)

        filter_started = time.perf_counter()
        if self.mode == "no_recovery":
            decisions = select_no_recovery_actions(request)
        elif self.mode == "rule":
            decisions = select_rule_based_actions(request)
        elif self.mode == "mappo":
            assert policy_probs is not None
            decisions = select_raw_mappo_actions(request, policy_probs)
        else:
            assert policy_probs is not None
            rankings, raw_policy_actions = policy_action_rankings(request, policy_probs)
            decisions = select_shielded_actions(
                request,
                action_rankings=rankings,
                raw_policy_actions=raw_policy_actions,
            )
        filter_ms = _elapsed_ms(filter_started)

        return (
            decisions,
            policy_probs,
            list(built.observations.shape),
            observation_ms,
            inference_ms,
            model_load_ms,
            model_forward_ms,
            filter_ms,
        )

    def _predict(
        self,
        request: EmergencyStepRequest,
        observations: np.ndarray,
        action_masks: np.ndarray,
    ) -> tuple[dict[str, list[float]], float, float]:
        import torch

        load_started = time.perf_counter()
        model = self._load_model(request)
        model_load_ms = _elapsed_ms(load_started)
        obs_tensor = torch.as_tensor(
            observations,
            dtype=torch.float32,
            device=model.device,
        )
        mask_tensor = torch.as_tensor(
            action_masks,
            dtype=torch.bool,
            device=model.device,
        )
        forward_started = time.perf_counter()
        with torch.no_grad():
            dist = model.model.action_distribution(obs_tensor, mask_tensor)
            probs = dist.probs.detach().cpu().numpy()
        model_forward_ms = _elapsed_ms(forward_started)

        expected_shape = (len(request.failed_agents), len(ACTION_NAMES))
        if probs.shape != expected_shape:
            raise ValueError(
                f"policy output has shape {probs.shape}, expected {expected_shape}"
            )

        policy_probs = {
            agent.agent_id: [float(value) for value in probs[index]]
            for index, agent in enumerate(request.failed_agents)
        }
        for agent_id, values in policy_probs.items():
            if len(values) != len(ACTION_NAMES):
                raise ValueError(
                    f"policy probabilities for {agent_id} have length "
                    f"{len(values)}, expected {len(ACTION_NAMES)}"
                )
        return policy_probs, model_load_ms, model_forward_ms

    def _load_model(self, request: EmergencyStepRequest) -> Any:
        from utm_mappo.mappo import DiscreteMAPPO, default_device
        import torch

        env = build_env_for_model(request)
        signature = (
            len(env.possible_agents),
            env.observation_space(env.possible_agents[0]).shape,
        )
        model = self._model_cache.get(signature)
        if model is not None:
            model.env = env
            return model

        if self.device == "auto":
            target_device = default_device()
        else:
            target_device = torch.device(self.device)
        assert self.model_dir is not None
        model = DiscreteMAPPO.load(env, self.model_dir, device=target_device)
        self._model_cache[signature] = model
        return model


def _advance_request(
    request: EmergencyStepRequest,
    decisions: list[Any],
) -> EmergencyStepRequest:
    decisions_by_id = {decision.agent_id: decision for decision in decisions}
    next_agents: list[FailedAgentSnapshot] = []

    for agent in request.failed_agents:
        decision = decisions_by_id.get(agent.agent_id)
        action = (
            int(decision.selected_action)
            if decision is not None
            else int(UTMAction.WAIT)
        )
        next_cell = add_cell(agent.current_cell, ACTION_DELTAS[action])
        recent_path = (*agent.recent_path, next_cell)
        if len(recent_path) > 8:
            recent_path = recent_path[-8:]
        next_agents.append(
            replace(
                agent,
                previous_cell=agent.current_cell,
                current_cell=next_cell,
                last_action=action,
                recent_path=recent_path,
                consecutive_wait_count=(
                    agent.consecutive_wait_count + 1
                    if action == int(UTMAction.WAIT)
                    else 0
                ),
                consecutive_filter_reject_count=(
                    agent.consecutive_filter_reject_count + 1
                    if decision is not None and decision.fallback_used
                    else 0
                ),
            )
        )

    return replace(
        request,
        time_step=request.time_step + 1,
        failed_agents=tuple(sorted(next_agents, key=lambda agent: agent.mission_id)),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="UE-MAPPO emergency recovery inference prototype."
    )
    parser.add_argument("--model-dir", type=Path, default=None)
    parser.add_argument(
        "--mode",
        choices=MODES,
        default="mappo_shield",
        help="Emergency recovery mode to execute.",
    )
    parser.add_argument(
        "--request",
        type=Path,
        default=None,
        help="JSON request file for one-shot inference.",
    )
    parser.add_argument("--serve", action="store_true", help="Run an HTTP /step server.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--device",
        default="auto",
        help="Torch device for MAPPO inference: auto, cpu, cuda, or cuda:0.",
    )
    parser.add_argument(
        "--action-horizon",
        type=int,
        default=None,
        help="Override request action_horizon for one-shot inference.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    service = EmergencyRecoveryService(
        model_dir=args.model_dir,
        mode=args.mode,
        device=args.device,
    )

    if args.serve:
        run_http_server(service, host=args.host, port=args.port)
        return

    if args.request is None:
        raise ValueError("--request is required unless --serve is set")
    payload = json.loads(args.request.read_text(encoding="utf-8"))
    if args.action_horizon is not None:
        payload["action_horizon"] = args.action_horizon
    response = service.step(payload)
    print(json.dumps(response, indent=2))


def run_http_server(
    service: EmergencyRecoveryService,
    host: str,
    port: int,
) -> None:
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self) -> None:  # noqa: N802
            if self.path != "/step":
                self.send_error(404, "only POST /step is supported")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                response = service.step(payload)
                body = json.dumps(response).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except Exception as exc:  # pragma: no cover - HTTP boundary.
                body = json.dumps({"error": str(exc)}).encode("utf-8")
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        def log_message(self, format: str, *args: Any) -> None:
            del format, args

    server = HTTPServer((host, port), Handler)
    print(f"UE-MAPPO emergency service listening on http://{host}:{port}/step")
    server.serve_forever()


def _elapsed_ms(started: float) -> float:
    return (time.perf_counter() - started) * 1000.0


if __name__ == "__main__":
    main()
