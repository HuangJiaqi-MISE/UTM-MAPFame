from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from utm_mappo.env import ACTION_DELTAS, UTMAction
from utm_mappo.geometry import add_cell, manhattan_distance

from .observation_builder import is_free_static, is_no_fly
from .schemas import EmergencyStepRequest, FailedAgentSnapshot, action_name


@dataclass(frozen=True)
class ActionDecision:
    agent_id: str
    mission_id: int
    selected_action: int
    raw_policy_action: int
    safety_filter_status: str
    fallback_used: bool
    degradation_state: str
    reject_reason: str | None = None


def select_no_recovery_actions(request: EmergencyStepRequest) -> list[ActionDecision]:
    return [
        ActionDecision(
            agent_id=agent.agent_id,
            mission_id=agent.mission_id,
            selected_action=int(UTMAction.WAIT),
            raw_policy_action=int(UTMAction.WAIT),
            safety_filter_status="not_applied",
            fallback_used=False,
            degradation_state="hold",
        )
        for agent in request.failed_agents
    ]


def select_rule_based_actions(request: EmergencyStepRequest) -> list[ActionDecision]:
    ranks = {
        agent.agent_id: _rule_action_ranking(request, agent)
        for agent in request.failed_agents
    }
    return select_shielded_actions(request, ranks, raw_policy_actions=None)


def select_raw_mappo_actions(
    request: EmergencyStepRequest,
    policy_probs: dict[str, list[float]],
) -> list[ActionDecision]:
    decisions = []
    for agent in request.failed_agents:
        probs = np.asarray(policy_probs[agent.agent_id], dtype=np.float32)
        action = int(probs.argmax())
        decisions.append(
            ActionDecision(
                agent_id=agent.agent_id,
                mission_id=agent.mission_id,
                selected_action=action,
                raw_policy_action=action,
                safety_filter_status="not_applied",
                fallback_used=False,
                degradation_state="normal",
            )
        )
    return decisions


def select_shielded_actions(
    request: EmergencyStepRequest,
    action_rankings: dict[str, list[int]],
    raw_policy_actions: dict[str, int] | None,
) -> list[ActionDecision]:
    decisions: list[ActionDecision] = []
    committed: dict[str, tuple[int, int, int]] = {}
    current_by_id = {agent.agent_id: agent.current_cell for agent in request.failed_agents}
    occupied_by_rid = {neighbor.cell for neighbor in request.rid_neighbors}

    for agent in request.failed_agents:
        ranking = _ensure_wait_fallback(action_rankings[agent.agent_id])
        raw_action = (
            int(raw_policy_actions[agent.agent_id])
            if raw_policy_actions is not None
            else ranking[0]
        )
        selected_action = int(UTMAction.WAIT)
        status = "fallback_wait"
        fallback_used = True
        reject_reason = "no_safe_candidate"

        for action in ranking:
            candidate = add_cell(agent.current_cell, ACTION_DELTAS[action])
            reason = _candidate_reject_reason(
                request=request,
                agent=agent,
                action=action,
                candidate=candidate,
                committed=committed,
                current_by_id=current_by_id,
                occupied_by_rid=occupied_by_rid,
            )
            if reason is not None:
                reject_reason = reason
                continue
            selected_action = action
            status = "accepted" if action == raw_action else "substituted"
            fallback_used = action != raw_action
            reject_reason = None
            break

        committed[agent.agent_id] = add_cell(
            agent.current_cell, ACTION_DELTAS[selected_action]
        )
        decisions.append(
            ActionDecision(
                agent_id=agent.agent_id,
                mission_id=agent.mission_id,
                selected_action=selected_action,
                raw_policy_action=raw_action,
                safety_filter_status=status,
                fallback_used=fallback_used,
                degradation_state="normal" if not fallback_used else "filtered",
                reject_reason=reject_reason,
            )
        )

    return decisions


def policy_action_rankings(
    request: EmergencyStepRequest,
    policy_probs: dict[str, list[float]],
) -> tuple[dict[str, list[int]], dict[str, int]]:
    rankings: dict[str, list[int]] = {}
    raw_actions: dict[str, int] = {}
    for agent in request.failed_agents:
        probs = np.asarray(policy_probs[agent.agent_id], dtype=np.float32)
        ordered = [int(index) for index in np.argsort(-probs)]
        rankings[agent.agent_id] = ordered
        raw_actions[agent.agent_id] = ordered[0]
    return rankings, raw_actions


def decisions_to_response_items(
    decisions: list[ActionDecision],
    policy_probs: dict[str, list[float]] | None,
) -> list[dict[str, object]]:
    items = []
    for decision in decisions:
        probs = policy_probs.get(decision.agent_id) if policy_probs is not None else None
        items.append(
            {
                "agent_id": decision.agent_id,
                "mission_id": decision.mission_id,
                "selected_action": action_name(decision.selected_action),
                "raw_policy_action": action_name(decision.raw_policy_action),
                "raw_policy_probs": _format_probs(probs),
                "safety_filter_status": decision.safety_filter_status,
                "fallback_used": decision.fallback_used,
                "degradation_state": decision.degradation_state,
                "reject_reason": decision.reject_reason,
            }
        )
    return items


def _candidate_reject_reason(
    request: EmergencyStepRequest,
    agent: FailedAgentSnapshot,
    action: int,
    candidate: tuple[int, int, int],
    committed: dict[str, tuple[int, int, int]],
    current_by_id: dict[str, tuple[int, int, int]],
    occupied_by_rid: set[tuple[int, int, int]],
) -> str | None:
    del action
    if not is_free_static(request, candidate):
        return "static_or_boundary"
    if is_no_fly(request, candidate, request.time_step + 1):
        return "no_fly"
    if candidate in occupied_by_rid:
        return "rid_vertex"
    if any(_downwash_conflict(candidate, rid_cell) for rid_cell in occupied_by_rid):
        return "rid_downwash"

    for other_id, other_current in current_by_id.items():
        if other_id == agent.agent_id:
            continue
        other_candidate = committed.get(other_id, other_current)
        if candidate == other_candidate:
            return "vertex"
        if candidate == other_current and other_candidate == agent.current_cell:
            return "edge"
        if _downwash_conflict(candidate, other_candidate):
            return "downwash"
    return None


def _downwash_conflict(
    left: tuple[int, int, int],
    right: tuple[int, int, int],
) -> bool:
    return (
        left[0] == right[0]
        and left[1] == right[1]
        and abs(left[2] - right[2]) == 1
    )


def _rule_action_ranking(
    request: EmergencyStepRequest,
    agent: FailedAgentSnapshot,
) -> list[int]:
    scored = []
    for action, delta in ACTION_DELTAS.items():
        candidate = add_cell(agent.current_cell, delta)
        distance = manhattan_distance(candidate, agent.goal_cell)
        wait_cost = 1 if int(action) == int(UTMAction.WAIT) else 0
        scored.append((distance, wait_cost, int(action)))
    scored.sort()
    return [item[-1] for item in scored]


def _ensure_wait_fallback(actions: list[int]) -> list[int]:
    seen = set()
    result = []
    for action in actions:
        action = int(action)
        if 0 <= action < len(UTMAction) and action not in seen:
            seen.add(action)
            result.append(action)
    if int(UTMAction.WAIT) not in seen:
        result.append(int(UTMAction.WAIT))
    return result


def _format_probs(probs: list[float] | None) -> dict[str, float] | None:
    if probs is None:
        return None
    return {
        action_name(index): round(float(prob), 6)
        for index, prob in enumerate(probs)
    }
