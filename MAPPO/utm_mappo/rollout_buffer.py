from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

import numpy as np
import torch


@dataclass
class RolloutBatch:
    observations: torch.Tensor
    action_masks: torch.Tensor
    states: torch.Tensor
    critic_states: torch.Tensor
    actions: torch.Tensor
    expert_actions: torch.Tensor
    old_log_probs: torch.Tensor
    advantages: torch.Tensor
    returns: torch.Tensor
    critic_returns: torch.Tensor


class MAPPORolloutBuffer:
    def __init__(
        self,
        buffer_size: int,
        n_agents: int,
        obs_dim: int,
        state_dim: int,
        gamma: float,
        gae_lambda: float,
        device: torch.device,
    ):
        self.buffer_size = buffer_size
        self.n_agents = n_agents
        self.obs_dim = obs_dim
        self.state_dim = state_dim
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.device = device
        self.reset()

    def reset(self) -> None:
        shape = (self.buffer_size, self.n_agents)
        self.observations = np.zeros((*shape, self.obs_dim), dtype=np.float32)
        self.states = np.zeros((self.buffer_size, self.state_dim), dtype=np.float32)
        self.actions = np.zeros(shape, dtype=np.int64)
        self.expert_actions = np.zeros(shape, dtype=np.int64)
        self.action_masks = np.zeros((*shape, 0), dtype=np.bool_)
        self.log_probs = np.zeros(shape, dtype=np.float32)
        self.rewards = np.zeros(self.buffer_size, dtype=np.float32)
        self.values = np.zeros(self.buffer_size, dtype=np.float32)
        self.dones = np.zeros(self.buffer_size, dtype=np.float32)
        self.active_masks = np.zeros(shape, dtype=np.float32)
        self.advantages = np.zeros(self.buffer_size, dtype=np.float32)
        self.returns = np.zeros(self.buffer_size, dtype=np.float32)
        self.pos = 0

    @property
    def full(self) -> bool:
        return self.pos >= self.buffer_size

    def add(
        self,
        observations: np.ndarray,
        state: np.ndarray,
        actions: np.ndarray,
        expert_actions: np.ndarray,
        action_masks: np.ndarray,
        log_probs: np.ndarray,
        reward: float,
        value: float,
        done: bool,
        active_mask: np.ndarray,
    ) -> None:
        if self.full:
            raise RuntimeError("rollout buffer is full")
        self.observations[self.pos] = observations
        self.states[self.pos] = state
        self.actions[self.pos] = actions
        self.expert_actions[self.pos] = expert_actions
        if self.action_masks.shape[-1] == 0:
            self.action_masks = np.zeros(
                (*self.actions.shape, action_masks.shape[-1]), dtype=np.bool_
            )
        self.action_masks[self.pos] = action_masks
        self.log_probs[self.pos] = log_probs
        self.rewards[self.pos] = reward
        self.values[self.pos] = value
        self.dones[self.pos] = float(done)
        self.active_masks[self.pos] = active_mask
        self.pos += 1

    def compute_returns_and_advantages(self, last_value: float) -> None:
        last_gae = 0.0
        for step in reversed(range(self.buffer_size)):
            if step == self.buffer_size - 1:
                next_non_terminal = 1.0 - self.dones[step]
                next_value = last_value
            else:
                next_non_terminal = 1.0 - self.dones[step + 1]
                next_value = self.values[step + 1]

            delta = (
                self.rewards[step]
                + self.gamma * next_value * next_non_terminal
                - self.values[step]
            )
            last_gae = (
                delta + self.gamma * self.gae_lambda * next_non_terminal * last_gae
            )
            self.advantages[step] = last_gae
        self.returns = self.advantages + self.values

    def minibatches(self, batch_size: int) -> Iterator[RolloutBatch]:
        if not self.full:
            raise RuntimeError("rollout buffer must be full before sampling")

        observations = self.observations.reshape(-1, self.obs_dim)
        states = np.repeat(self.states, repeats=self.n_agents, axis=0)
        actions = self.actions.reshape(-1)
        expert_actions = self.expert_actions.reshape(-1)
        action_masks = self.action_masks.reshape(-1, self.action_masks.shape[-1])
        log_probs = self.log_probs.reshape(-1)
        masks = self.active_masks.reshape(-1) > 0.5
        advantages = np.repeat(self.advantages, repeats=self.n_agents, axis=0)
        returns = np.repeat(self.returns, repeats=self.n_agents, axis=0)

        valid_indices = np.flatnonzero(masks)
        np.random.shuffle(valid_indices)

        for start in range(0, len(valid_indices), batch_size):
            indices = valid_indices[start : start + batch_size]
            critic_indices = np.unique(indices // self.n_agents)
            yield RolloutBatch(
                observations=torch.as_tensor(
                    observations[indices], dtype=torch.float32, device=self.device
                ),
                action_masks=torch.as_tensor(
                    action_masks[indices], dtype=torch.bool, device=self.device
                ),
                states=torch.as_tensor(
                    states[indices], dtype=torch.float32, device=self.device
                ),
                critic_states=torch.as_tensor(
                    self.states[critic_indices], dtype=torch.float32, device=self.device
                ),
                actions=torch.as_tensor(
                    actions[indices], dtype=torch.long, device=self.device
                ),
                expert_actions=torch.as_tensor(
                    expert_actions[indices], dtype=torch.long, device=self.device
                ),
                old_log_probs=torch.as_tensor(
                    log_probs[indices], dtype=torch.float32, device=self.device
                ),
                advantages=torch.as_tensor(
                    advantages[indices], dtype=torch.float32, device=self.device
                ),
                returns=torch.as_tensor(
                    returns[indices], dtype=torch.float32, device=self.device
                ),
                critic_returns=torch.as_tensor(
                    self.returns[critic_indices], dtype=torch.float32, device=self.device
                ),
            )
