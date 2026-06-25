from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
import torch.nn.functional as F
from tqdm import trange

from common import load_env, scenario_paths

from utm_mappo.mappo import DiscreteMAPPO  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train the UTM MAPPO actor from an offline expert dataset."
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        nargs="+",
        required=True,
        help="One or more offline dataset directories to concatenate.",
    )
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--scenario-dir", type=Path, default=None)
    parser.add_argument("--init-model-dir", type=Path, default=None)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument(
        "--updates",
        type=int,
        default=0,
        help="Use fixed random minibatch updates instead of epoch passes.",
    )
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--validation-fraction", type=float, default=0.05)
    parser.add_argument("--eval-interval", type=int, default=200)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--freeze-actor-encoder", action="store_true")
    parser.add_argument(
        "--device",
        default="auto",
        help="Torch device, for example auto, cuda, cuda:0, or cpu.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)

    metadata_items = [load_metadata(path) for path in args.dataset_dir]
    metadata = combine_metadata(args.dataset_dir, metadata_items)
    observations, action_masks, actions = load_datasets(
        args.dataset_dir,
        metadata_items=metadata_items,
        rng=rng,
        max_samples=args.max_samples,
    )
    scenario_dir = resolve_scenario_dir(args, metadata)
    env = load_env(scenario_paths(scenario_dir)[0])
    validate_dataset_shape(env, observations, action_masks, metadata)

    model = DiscreteMAPPO(
        env,
        learning_rate=args.learning_rate,
        freeze_actor_encoder=args.freeze_actor_encoder,
        device=parse_device(args.device),
    )
    if args.init_model_dir is not None:
        model.load_weights(args.init_model_dir)

    train_indices, validation_indices = split_indices(
        sample_count=actions.shape[0],
        validation_fraction=args.validation_fraction,
        rng=rng,
    )
    print(
        f"loaded {actions.shape[0]} samples from {metadata['stored_scenario_count']} "
        f"expert shards; train={len(train_indices)} val={len(validation_indices)} "
        f"device={model.device}"
    )

    if args.updates > 0:
        train_fixed_updates(
            model=model,
            observations=observations,
            action_masks=action_masks,
            actions=actions,
            train_indices=train_indices,
            validation_indices=validation_indices,
            batch_size=args.batch_size,
            updates=args.updates,
            eval_interval=args.eval_interval,
            rng=rng,
        )
    else:
        train_epochs(
            model=model,
            observations=observations,
            action_masks=action_masks,
            actions=actions,
            train_indices=train_indices,
            validation_indices=validation_indices,
            batch_size=args.batch_size,
            epochs=args.epochs,
            eval_interval=args.eval_interval,
            rng=rng,
        )

    if len(validation_indices) > 0:
        val_loss, val_accuracy = evaluate_bc(
            model, observations, action_masks, actions, validation_indices, args.batch_size
        )
        print(f"final_val_loss={val_loss:.4f} final_val_accuracy={val_accuracy:.3f}")

    model.save(args.model_dir)
    print(f"saved offline behavior-cloned model to {args.model_dir}")


def load_metadata(dataset_dir: Path) -> dict[str, Any]:
    metadata_path = dataset_dir / "metadata.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"missing dataset metadata: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def combine_metadata(
    dataset_dirs: list[Path],
    metadata_items: list[dict[str, Any]],
) -> dict[str, Any]:
    if not metadata_items:
        raise ValueError("at least one dataset directory is required")

    combined = dict(metadata_items[0])
    combined["dataset_dirs"] = [str(path) for path in dataset_dirs]
    combined["dataset_count"] = len(metadata_items)
    combined["stored_scenario_count"] = sum(
        int(item.get("stored_scenario_count", 0)) for item in metadata_items
    )
    combined["scenario_count"] = sum(
        int(item.get("scenario_count", 0)) for item in metadata_items
    )
    combined["samples"] = sum(int(item.get("samples", 0)) for item in metadata_items)

    expected_shape = tuple(metadata_items[0].get("obs_shape", ()))
    expected_action_dim = int(metadata_items[0].get("action_dim", 0))
    expected_agents = int(metadata_items[0].get("n_agents", 0))
    for dataset_dir, item in zip(dataset_dirs, metadata_items, strict=True):
        obs_shape = tuple(item.get("obs_shape", ()))
        action_dim = int(item.get("action_dim", 0))
        n_agents = int(item.get("n_agents", 0))
        if obs_shape != expected_shape:
            raise ValueError(
                f"{dataset_dir} obs_shape {obs_shape} does not match "
                f"{expected_shape}"
            )
        if action_dim != expected_action_dim:
            raise ValueError(
                f"{dataset_dir} action_dim {action_dim} does not match "
                f"{expected_action_dim}"
            )
        if n_agents != expected_agents:
            raise ValueError(
                f"{dataset_dir} n_agents {n_agents} does not match "
                f"{expected_agents}"
            )

    return combined


def load_datasets(
    dataset_dirs: list[Path],
    metadata_items: list[dict[str, Any]],
    rng: np.random.Generator,
    max_samples: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    observation_parts: list[np.ndarray] = []
    mask_parts: list[np.ndarray] = []
    action_parts: list[np.ndarray] = []

    for dataset_dir, metadata in zip(dataset_dirs, metadata_items, strict=True):
        observations, action_masks, actions = load_dataset(
            dataset_dir,
            metadata=metadata,
        )
        observation_parts.append(observations)
        mask_parts.append(action_masks)
        action_parts.append(actions)

    observations = np.concatenate(observation_parts, axis=0)
    action_masks = np.concatenate(mask_parts, axis=0)
    actions = np.concatenate(action_parts, axis=0)

    if max_samples > 0 and observations.shape[0] > max_samples:
        indices = rng.choice(observations.shape[0], size=max_samples, replace=False)
        observations = observations[indices]
        action_masks = action_masks[indices]
        actions = actions[indices]

    return observations, action_masks, actions


def load_dataset(
    dataset_dir: Path,
    metadata: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    observation_parts: list[np.ndarray] = []
    mask_parts: list[np.ndarray] = []
    action_parts: list[np.ndarray] = []

    for shard in metadata["shards"]:
        shard_path = dataset_dir / shard["file"]
        with np.load(shard_path) as data:
            observation_parts.append(data["observations"].astype(np.float32))
            mask_parts.append(data["action_masks"].astype(np.bool_))
            action_parts.append(data["actions"].astype(np.int64))

    if not observation_parts:
        raise RuntimeError(f"no shards found in {dataset_dir}")

    observations = np.concatenate(observation_parts, axis=0)
    action_masks = np.concatenate(mask_parts, axis=0)
    actions = np.concatenate(action_parts, axis=0)

    if actions.min(initial=0) < 0 or actions.max(initial=0) >= action_masks.shape[1]:
        raise ValueError("dataset contains target actions outside the action space")

    invalid_targets = ~action_masks[np.arange(actions.shape[0]), actions]
    if invalid_targets.any():
        raise ValueError(
            f"dataset contains {int(invalid_targets.sum())} target actions that "
            "are masked out"
        )

    return observations, action_masks, actions


def resolve_scenario_dir(args: argparse.Namespace, metadata: dict[str, Any]) -> Path:
    if args.scenario_dir is not None:
        return args.scenario_dir

    for key in ("scenario_dir", "scenario_dir_resolved"):
        value = metadata.get(key)
        if value is None:
            continue
        path = Path(value)
        if path.exists():
            return path

    raise ValueError(
        "--scenario-dir is required because the dataset metadata scenario path "
        "does not exist on this machine"
    )


def validate_dataset_shape(
    env: Any,
    observations: np.ndarray,
    action_masks: np.ndarray,
    metadata: dict[str, Any],
) -> None:
    agent = env.possible_agents[0]
    env_obs_shape = env.observation_space(agent).shape
    env_action_dim = int(env.action_space(agent).n)
    if observations.shape[1:] != env_obs_shape:
        raise ValueError(
            f"dataset observation shape {observations.shape[1:]} does not match "
            f"scenario observation shape {env_obs_shape}"
        )
    if action_masks.shape[1] != env_action_dim:
        raise ValueError(
            f"dataset action dim {action_masks.shape[1]} does not match "
            f"scenario action dim {env_action_dim}"
        )
    if int(metadata.get("n_agents", len(env.possible_agents))) != len(
        env.possible_agents
    ):
        raise ValueError(
            "dataset n_agents does not match the selected scenario directory"
        )


def split_indices(
    sample_count: int,
    validation_fraction: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    indices = np.arange(sample_count, dtype=np.int64)
    rng.shuffle(indices)
    validation_count = int(sample_count * max(0.0, min(validation_fraction, 0.5)))
    if validation_count <= 0 or validation_count >= sample_count:
        return indices, np.asarray([], dtype=np.int64)
    return indices[validation_count:], indices[:validation_count]


def train_fixed_updates(
    model: DiscreteMAPPO,
    observations: np.ndarray,
    action_masks: np.ndarray,
    actions: np.ndarray,
    train_indices: np.ndarray,
    validation_indices: np.ndarray,
    batch_size: int,
    updates: int,
    eval_interval: int,
    rng: np.random.Generator,
) -> None:
    progress = trange(updates, desc="offline bc")
    for step in progress:
        batch_indices = sample_batch(train_indices, batch_size, rng)
        loss, accuracy = train_batch(
            model, observations, action_masks, actions, batch_indices
        )
        progress.set_postfix(loss=f"{loss:.3f}", acc=f"{accuracy:.3f}")
        maybe_print_validation(
            model,
            observations,
            action_masks,
            actions,
            validation_indices,
            batch_size,
            eval_interval,
            step + 1,
            progress,
        )


def train_epochs(
    model: DiscreteMAPPO,
    observations: np.ndarray,
    action_masks: np.ndarray,
    actions: np.ndarray,
    train_indices: np.ndarray,
    validation_indices: np.ndarray,
    batch_size: int,
    epochs: int,
    eval_interval: int,
    rng: np.random.Generator,
) -> None:
    steps_per_epoch = max(1, int(np.ceil(len(train_indices) / batch_size)))
    total_steps = max(1, epochs) * steps_per_epoch
    step = 0
    progress = trange(total_steps, desc="offline bc")
    for _ in range(max(1, epochs)):
        epoch_indices = rng.permutation(train_indices)
        for start in range(0, len(epoch_indices), batch_size):
            batch_indices = epoch_indices[start : start + batch_size]
            loss, accuracy = train_batch(
                model, observations, action_masks, actions, batch_indices
            )
            step += 1
            progress.update(1)
            progress.set_postfix(loss=f"{loss:.3f}", acc=f"{accuracy:.3f}")
            maybe_print_validation(
                model,
                observations,
                action_masks,
                actions,
                validation_indices,
                batch_size,
                eval_interval,
                step,
                progress,
            )
    progress.close()


def sample_batch(
    train_indices: np.ndarray,
    batch_size: int,
    rng: np.random.Generator,
) -> np.ndarray:
    replace = len(train_indices) < batch_size
    return rng.choice(train_indices, size=batch_size, replace=replace)


def train_batch(
    model: DiscreteMAPPO,
    observations: np.ndarray,
    action_masks: np.ndarray,
    actions: np.ndarray,
    batch_indices: np.ndarray,
) -> tuple[float, float]:
    obs_tensor = torch.as_tensor(
        observations[batch_indices], dtype=torch.float32, device=model.device
    )
    mask_tensor = torch.as_tensor(
        action_masks[batch_indices], dtype=torch.bool, device=model.device
    )
    target_actions = torch.as_tensor(
        actions[batch_indices], dtype=torch.long, device=model.device
    )

    dist = model.model.action_distribution(obs_tensor, mask_tensor)
    loss = F.cross_entropy(dist.logits, target_actions)

    model.optimizer.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_norm_(model.model.parameters(), model.max_grad_norm)
    model.optimizer.step()

    predictions = dist.probs.argmax(dim=-1)
    accuracy = (predictions == target_actions).to(torch.float32).mean()
    return float(loss.item()), float(accuracy.item())


def maybe_print_validation(
    model: DiscreteMAPPO,
    observations: np.ndarray,
    action_masks: np.ndarray,
    actions: np.ndarray,
    validation_indices: np.ndarray,
    batch_size: int,
    eval_interval: int,
    step: int,
    progress: Any,
) -> None:
    if eval_interval <= 0 or len(validation_indices) == 0:
        return
    if step % eval_interval != 0:
        return

    val_loss, val_accuracy = evaluate_bc(
        model, observations, action_masks, actions, validation_indices, batch_size
    )
    progress.write(
        f"step={step} val_loss={val_loss:.4f} val_accuracy={val_accuracy:.3f}"
    )


def evaluate_bc(
    model: DiscreteMAPPO,
    observations: np.ndarray,
    action_masks: np.ndarray,
    actions: np.ndarray,
    indices: np.ndarray,
    batch_size: int,
) -> tuple[float, float]:
    losses: list[float] = []
    correct = 0
    total = 0
    with torch.no_grad():
        for start in range(0, len(indices), batch_size):
            batch_indices = indices[start : start + batch_size]
            obs_tensor = torch.as_tensor(
                observations[batch_indices],
                dtype=torch.float32,
                device=model.device,
            )
            mask_tensor = torch.as_tensor(
                action_masks[batch_indices],
                dtype=torch.bool,
                device=model.device,
            )
            target_actions = torch.as_tensor(
                actions[batch_indices], dtype=torch.long, device=model.device
            )
            dist = model.model.action_distribution(obs_tensor, mask_tensor)
            loss = F.cross_entropy(dist.logits, target_actions)
            predictions = dist.probs.argmax(dim=-1)
            correct += int((predictions == target_actions).sum().item())
            total += int(target_actions.numel())
            losses.append(float(loss.item()) * int(target_actions.numel()))

    if total == 0:
        return 0.0, 0.0
    return float(sum(losses) / total), float(correct / total)


def parse_device(value: str) -> torch.device | None:
    if value == "auto":
        return None
    return torch.device(value)


if __name__ == "__main__":
    main()
