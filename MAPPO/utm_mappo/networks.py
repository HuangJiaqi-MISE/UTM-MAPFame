from __future__ import annotations

import torch
import torch.nn as nn
from gymnasium import spaces


class ObservationEncoder(nn.Module):
    def __init__(
        self,
        observation_space: spaces.Box,
        features_dim: int = 256,
        hidden_dim: int = 256,
    ):
        super().__init__()
        obs_dim = int(observation_space.shape[0])
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, features_dim),
            nn.ReLU(),
        )
        self.features_dim = features_dim

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.net(observations)


class DecentralizedActor(nn.Module):
    def __init__(self, features_dim: int, action_dim: int, hidden_dim: int = 128):
        super().__init__()
        self.policy = nn.Sequential(
            nn.Linear(features_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, action_dim),
        )

    def forward(
        self, features: torch.Tensor, action_masks: torch.Tensor | None = None
    ) -> torch.distributions.Categorical:
        logits = self.policy(features)
        if action_masks is not None:
            logits = logits.masked_fill(~action_masks.bool(), -1.0e9)
        return torch.distributions.Categorical(logits=logits)


class CentralizedCritic(nn.Module):
    def __init__(self, state_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.value = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, states: torch.Tensor) -> torch.Tensor:
        return self.value(states).squeeze(-1)


class ActorCritic(nn.Module):
    def __init__(
        self,
        observation_space: spaces.Box,
        action_space: spaces.Discrete,
        state_dim: int,
        features_dim: int = 256,
        hidden_dim: int = 256,
    ):
        super().__init__()
        self.encoder = ObservationEncoder(
            observation_space,
            features_dim=features_dim,
            hidden_dim=hidden_dim,
        )
        self.actor = DecentralizedActor(
            features_dim=features_dim,
            action_dim=int(action_space.n),
            hidden_dim=hidden_dim,
        )
        self.critic = CentralizedCritic(state_dim=state_dim, hidden_dim=hidden_dim)

    def action_distribution(
        self, observations: torch.Tensor, action_masks: torch.Tensor | None = None
    ) -> torch.distributions.Categorical:
        return self.actor(self.encoder(observations), action_masks=action_masks)

    def value(self, states: torch.Tensor) -> torch.Tensor:
        return self.critic(states)
