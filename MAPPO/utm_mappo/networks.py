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


class AttentionPoolingCritic(nn.Module):
    def __init__(
        self,
        features_dim: int,
        hidden_dim: int = 256,
        num_heads: int = 4,
    ):
        super().__init__()
        self.input_projection = nn.Linear(features_dim, hidden_dim)
        self.attention = nn.MultiheadAttention(
            embed_dim=hidden_dim,
            num_heads=num_heads,
            batch_first=True,
        )
        self.norm = nn.LayerNorm(hidden_dim)
        self.value = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(
        self,
        agent_features: torch.Tensor,
        active_masks: torch.Tensor | None = None,
    ) -> torch.Tensor:
        tokens = self.input_projection(agent_features)
        key_padding_mask = None

        if active_masks is not None:
            active = active_masks.bool()
            if active.ndim != 2:
                raise ValueError("active_masks must have shape [batch, n_agents]")
            empty_rows = ~active.any(dim=1)
            if empty_rows.any():
                active = active.clone()
                active[empty_rows, 0] = True
            key_padding_mask = ~active

        attended, _ = self.attention(
            tokens,
            tokens,
            tokens,
            key_padding_mask=key_padding_mask,
            need_weights=False,
        )
        tokens = self.norm(tokens + attended)

        if active_masks is None:
            pooled = tokens.mean(dim=1)
        else:
            mask = (~key_padding_mask).to(tokens.dtype).unsqueeze(-1)
            pooled = (tokens * mask).sum(dim=1) / mask.sum(dim=1).clamp_min(1.0)

        return self.value(pooled).squeeze(-1)


class ActorCritic(nn.Module):
    def __init__(
        self,
        observation_space: spaces.Box,
        action_space: spaces.Discrete,
        state_dim: int | None = None,
        n_agents: int = 1,
        features_dim: int = 256,
        hidden_dim: int = 256,
        critic_num_heads: int = 4,
    ):
        super().__init__()
        del state_dim
        self.obs_dim = int(observation_space.shape[0])
        self.n_agents = n_agents
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
        self.critic = AttentionPoolingCritic(
            features_dim=features_dim,
            hidden_dim=hidden_dim,
            num_heads=critic_num_heads,
        )

    def action_distribution(
        self, observations: torch.Tensor, action_masks: torch.Tensor | None = None
    ) -> torch.distributions.Categorical:
        return self.actor(self.encoder(observations), action_masks=action_masks)

    def value(
        self, states: torch.Tensor, active_masks: torch.Tensor | None = None
    ) -> torch.Tensor:
        if states.ndim == 2:
            expected_dim = self.obs_dim * self.n_agents
            if states.shape[-1] != expected_dim:
                raise ValueError(
                    f"flat critic state has dim {states.shape[-1]}, expected {expected_dim}"
                )
            observations = states.reshape(states.shape[0], self.n_agents, self.obs_dim)
        elif states.ndim == 3:
            observations = states
        else:
            raise ValueError("critic state must have shape [batch, state_dim] or [batch, n_agents, obs_dim]")

        return self.value_from_observations(observations, active_masks=active_masks)

    def value_from_observations(
        self,
        observations: torch.Tensor,
        active_masks: torch.Tensor | None = None,
    ) -> torch.Tensor:
        batch_size, n_agents, obs_dim = observations.shape
        features = self.encoder(observations.reshape(batch_size * n_agents, obs_dim))
        features = features.reshape(batch_size, n_agents, -1)
        return self.critic(features, active_masks=active_masks)
