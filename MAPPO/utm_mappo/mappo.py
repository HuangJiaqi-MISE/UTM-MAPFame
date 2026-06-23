from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from tqdm import trange

from .env import UTMMAPFEnv
from .expert import prioritized_shortest_path_actions
from .networks import ActorCritic
from .rollout_buffer import MAPPORolloutBuffer


def default_device() -> torch.device:
    if torch.cuda.is_available():
        return torch.device("cuda")
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


class DiscreteMAPPO:
    def __init__(
        self,
        env: UTMMAPFEnv,
        learning_rate: float = 3e-4,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        clip_ratio: float = 0.2,
        value_coef: float = 0.5,
        entropy_coef: float = 0.01,
        bc_anchor_coef: float = 0.0,
        max_grad_norm: float = 0.5,
        rollout_steps: int = 256,
        batch_size: int = 256,
        update_epochs: int = 4,
        features_dim: int = 256,
        hidden_dim: int = 256,
        device: torch.device | None = None,
    ):
        self.env = env
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_ratio = clip_ratio
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.bc_anchor_coef = bc_anchor_coef
        self.max_grad_norm = max_grad_norm
        self.rollout_steps = rollout_steps
        self.batch_size = batch_size
        self.update_epochs = update_epochs
        self.features_dim = features_dim
        self.hidden_dim = hidden_dim
        self.device = device or default_device()
        self.agent_order = list(env.possible_agents)
        self.n_agents = len(self.agent_order)

        observation_space = env.observation_space(self.agent_order[0])
        action_space = env.action_space(self.agent_order[0])
        obs_dim = int(observation_space.shape[0])
        self.state_dim = obs_dim * self.n_agents

        self.model = ActorCritic(
            observation_space=observation_space,
            action_space=action_space,
            state_dim=self.state_dim,
            n_agents=self.n_agents,
            features_dim=features_dim,
            hidden_dim=hidden_dim,
        ).to(self.device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        self.buffer = MAPPORolloutBuffer(
            buffer_size=rollout_steps,
            n_agents=self.n_agents,
            obs_dim=obs_dim,
            state_dim=self.state_dim,
            gamma=gamma,
            gae_lambda=gae_lambda,
            device=self.device,
        )

    def predict(
        self, observations_by_agent: dict[str, np.ndarray], deterministic: bool = True
    ) -> dict[str, int]:
        if not observations_by_agent:
            return {}
        agents = sorted(observations_by_agent)
        observations = np.stack([observations_by_agent[agent] for agent in agents])
        action_masks = np.stack([self.env.action_mask(agent) for agent in agents])
        obs_tensor = torch.as_tensor(
            observations, dtype=torch.float32, device=self.device
        )
        mask_tensor = torch.as_tensor(
            action_masks, dtype=torch.bool, device=self.device
        )
        with torch.no_grad():
            dist = self.model.action_distribution(obs_tensor, mask_tensor)
            actions = dist.probs.argmax(dim=-1) if deterministic else dist.sample()
        return {
            agent: int(action)
            for agent, action in zip(agents, actions.detach().cpu().numpy())
        }

    def learn(self, total_timesteps: int, eval_interval: int = 10) -> None:
        iterations = max(1, total_timesteps // self.rollout_steps)
        progress = trange(iterations, desc="training")
        for iteration in progress:
            rollout_metrics = self.collect_rollout()
            train_metrics = self.update()
            metrics = {**rollout_metrics, **train_metrics}
            progress.set_postfix(
                reward=f"{metrics['mean_episode_reward']:.3f}",
                loss=f"{metrics['loss']:.3f}",
                entropy=f"{metrics['entropy']:.3f}",
            )
            if eval_interval > 0 and (iteration + 1) % eval_interval == 0:
                eval_reward = self.evaluate(num_episodes=3)
                progress.write(f"eval_reward={eval_reward:.3f}")

    def collect_rollout(self) -> dict[str, float]:
        self.buffer.reset()
        episode_rewards: list[float] = []
        current_episode_reward = 0.0
        observations, _ = self.env.reset()

        while not self.buffer.full:
            dense_observations = self._dense_observations(observations)
            dense_action_masks = self._dense_action_masks(observations)
            if self.bc_anchor_coef > 0.0:
                dense_expert_actions = self._dense_expert_actions(
                    observations, dense_action_masks
                )
            else:
                dense_expert_actions = np.zeros(self.n_agents, dtype=np.int64)
            state = dense_observations.reshape(-1)
            active_mask = np.asarray(
                [1.0 if agent in observations else 0.0 for agent in self.agent_order],
                dtype=np.float32,
            )

            obs_tensor = torch.as_tensor(
                dense_observations, dtype=torch.float32, device=self.device
            )
            mask_tensor = torch.as_tensor(
                dense_action_masks, dtype=torch.bool, device=self.device
            )
            state_tensor = torch.as_tensor(
                state[None, :], dtype=torch.float32, device=self.device
            )

            with torch.no_grad():
                dist = self.model.action_distribution(obs_tensor, mask_tensor)
                sampled_actions = dist.sample()
                log_probs = dist.log_prob(sampled_actions)
                value = self.model.value(state_tensor).item()

            actions_by_agent = {
                agent: int(action)
                for agent, action in zip(
                    self.agent_order, sampled_actions.detach().cpu().numpy()
                )
                if agent in observations
            }
            next_observations, rewards, terminations, truncations, _ = self.env.step(
                actions_by_agent
            )
            team_reward = float(sum(rewards.values()) / max(1, len(rewards)))
            done = len(next_observations) == 0 or all(
                terminations.get(agent, False) or truncations.get(agent, False)
                for agent in rewards
            )
            current_episode_reward += team_reward

            self.buffer.add(
                observations=dense_observations,
                state=state,
                actions=sampled_actions.detach().cpu().numpy(),
                expert_actions=dense_expert_actions,
                action_masks=dense_action_masks,
                log_probs=log_probs.detach().cpu().numpy(),
                reward=team_reward,
                value=value,
                done=done,
                active_mask=active_mask,
            )

            observations = next_observations
            if done:
                episode_rewards.append(current_episode_reward)
                current_episode_reward = 0.0
                observations, _ = self.env.reset()

        last_state = self._dense_observations(observations).reshape(-1)
        with torch.no_grad():
            last_value = self.model.value(
                torch.as_tensor(
                    last_state[None, :], dtype=torch.float32, device=self.device
                )
            ).item()
        self.buffer.compute_returns_and_advantages(last_value)

        if current_episode_reward != 0.0:
            episode_rewards.append(current_episode_reward)
        return {
            "mean_episode_reward": float(np.mean(episode_rewards))
            if episode_rewards
            else 0.0
        }

    def update(self) -> dict[str, float]:
        losses: list[float] = []
        actor_losses: list[float] = []
        critic_losses: list[float] = []
        entropies: list[float] = []

        for _ in range(self.update_epochs):
            for batch in self.buffer.minibatches(self.batch_size):
                advantages = batch.advantages
                if advantages.numel() > 1:
                    advantages = (advantages - advantages.mean()) / (
                        advantages.std(unbiased=False) + 1e-8
                    )

                dist = self.model.action_distribution(
                    batch.observations, batch.action_masks
                )
                new_log_probs = dist.log_prob(batch.actions)
                entropy = dist.entropy().mean()
                ratio = torch.exp(new_log_probs - batch.old_log_probs)
                clipped_ratio = torch.clamp(
                    ratio, 1.0 - self.clip_ratio, 1.0 + self.clip_ratio
                )
                actor_loss = -torch.min(
                    ratio * advantages, clipped_ratio * advantages
                ).mean()

                values = self.model.value(batch.critic_states)
                critic_loss = F.mse_loss(values, batch.critic_returns)
                bc_anchor_loss = (
                    F.cross_entropy(dist.logits, batch.expert_actions)
                    if self.bc_anchor_coef > 0.0
                    else torch.zeros((), dtype=torch.float32, device=self.device)
                )
                loss = (
                    actor_loss
                    + self.value_coef * critic_loss
                    + self.bc_anchor_coef * bc_anchor_loss
                    - self.entropy_coef * entropy
                )

                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(
                    self.model.parameters(), self.max_grad_norm
                )
                self.optimizer.step()

                losses.append(float(loss.item()))
                actor_losses.append(float(actor_loss.item()))
                critic_losses.append(float(critic_loss.item()))
                entropies.append(float(entropy.item()))

        return {
            "loss": float(np.mean(losses)) if losses else 0.0,
            "actor_loss": float(np.mean(actor_losses)) if actor_losses else 0.0,
            "critic_loss": float(np.mean(critic_losses)) if critic_losses else 0.0,
            "entropy": float(np.mean(entropies)) if entropies else 0.0,
        }

    def evaluate(self, num_episodes: int = 5) -> float:
        rewards = []
        for _ in range(num_episodes):
            observations, _ = self.env.reset()
            episode_reward = 0.0
            while observations:
                actions = self.predict(observations, deterministic=True)
                observations, step_rewards, _, _, _ = self.env.step(actions)
                episode_reward += float(
                    sum(step_rewards.values()) / max(1, len(step_rewards))
                )
            rewards.append(episode_reward)
        return float(np.mean(rewards)) if rewards else 0.0

    def save(self, model_dir: str | Path) -> None:
        model_dir = Path(model_dir)
        model_dir.mkdir(parents=True, exist_ok=True)
        torch.save(
            {
                "model_state_dict": self.model.state_dict(),
                "agent_order": self.agent_order,
                "state_dim": self.state_dim,
                "n_agents": self.n_agents,
                "features_dim": self.features_dim,
                "hidden_dim": self.hidden_dim,
                "critic_type": "attention_pooling",
            },
            model_dir / "model.pt",
        )

    def load_weights(self, model_dir: str | Path) -> None:
        checkpoint = torch.load(Path(model_dir) / "model.pt", map_location=self.device)
        self._load_compatible_state_dict(checkpoint["model_state_dict"])

    @classmethod
    def load(
        cls,
        env: UTMMAPFEnv,
        model_dir: str | Path,
        device: torch.device | None = None,
    ) -> "DiscreteMAPPO":
        checkpoint_path = Path(model_dir) / "model.pt"
        target_device = device or default_device()
        checkpoint = torch.load(checkpoint_path, map_location=target_device)
        model = cls(
            env,
            features_dim=int(checkpoint.get("features_dim", 256)),
            hidden_dim=int(checkpoint.get("hidden_dim", 256)),
            device=target_device,
        )
        model._load_compatible_state_dict(checkpoint["model_state_dict"])
        model.model.eval()
        return model

    def _load_compatible_state_dict(
        self, checkpoint_state: dict[str, torch.Tensor]
    ) -> None:
        current_state = self.model.state_dict()
        compatible_state = {}
        skipped_keys = []

        for key, value in checkpoint_state.items():
            if key in current_state and current_state[key].shape == value.shape:
                compatible_state[key] = value
            else:
                skipped_keys.append(key)

        current_state.update(compatible_state)
        self.model.load_state_dict(current_state)

        if skipped_keys:
            print(
                "loaded compatible checkpoint tensors; skipped "
                f"{len(skipped_keys)} incompatible tensors"
            )

    def _dense_observations(self, observations: dict[str, np.ndarray]) -> np.ndarray:
        obs_dim = int(self.env.observation_space(self.agent_order[0]).shape[0])
        dense = np.zeros((self.n_agents, obs_dim), dtype=np.float32)
        for index, agent in enumerate(self.agent_order):
            if agent in observations:
                dense[index] = observations[agent]
            else:
                dense[index] = self.env._observe(agent)
        return dense

    def _dense_action_masks(self, observations: dict[str, np.ndarray]) -> np.ndarray:
        action_dim = int(self.env.action_space(self.agent_order[0]).n)
        dense = np.zeros((self.n_agents, action_dim), dtype=np.bool_)
        for index, agent in enumerate(self.agent_order):
            if agent in observations:
                dense[index] = self.env.action_mask(agent)
            else:
                dense[index, 0] = True
        return dense

    def _dense_expert_actions(
        self, observations: dict[str, np.ndarray], action_masks: np.ndarray
    ) -> np.ndarray:
        expert_actions = np.zeros(self.n_agents, dtype=np.int64)
        expert_actions_by_agent = prioritized_shortest_path_actions(
            self.env, tuple(observations)
        )
        for index, agent in enumerate(self.agent_order):
            if agent not in observations:
                continue
            action = expert_actions_by_agent[agent]
            if not action_masks[index, action]:
                action = 0
            expert_actions[index] = action
        return expert_actions
