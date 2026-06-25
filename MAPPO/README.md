# UTM MAPPO Environment

This directory contains a Python training environment for a CTDE/MAPPO fallback policy used when the centralized LaCAM-UTM solver cannot provide a plan in time.

The environment is intentionally aligned with the Unreal/C++ system:

- 3D discrete grid cells
- One decentralized action per active mission
- Actions: wait, +/-X, +/-Y, +/-Z
- Static occupancy constraints
- Temporal no-fly zone constraints
- Dynamic conflicts: vertex, edge, and downwash
- Each drone body occupies exactly one grid cell
- Fixed downwash conflict checks: the cell directly below a drone is unsafe, including transition sweep checks
- Finished agents remain as stationary goal anchors
- Safety gate behavior: unsafe proposals are arbitrated into hold actions and penalized

The environment is not intended to replace LaCAM-UTM. It is a training target for a distributed emergency policy whose outputs still need to pass the C++ safety gate before execution.

## Quick Start

Install the Python dependencies in an isolated environment, then run a random rollout:

```powershell
cd D:\UTM-MAPFame\MAPPO
pip install -e ".[dev]"
python -m pytest
python examples\random_rollout.py
```

To run the MAPPO training skeleton, install the training extra:

```powershell
pip install -e ".[dev,train]"
python -m pytest
python pretrain_bc.py configs\crossing.yaml --model-dir models\crossing_bc_priority
python train_mappo.py configs\crossing.yaml --init-model-dir models\crossing_bc_priority --model-dir models\crossing_attention --timesteps 300000 --learning-rate 5e-5 --bc-anchor-coef 0.1 --entropy-coef 0.001 --freeze-actor-encoder --eval-interval 0 --rollout-steps 1024 --batch-size 1024
```

Behavior cloning is optional, but it is the recommended first step. It teaches the decentralized actor a PIBT-style priority-aware shortest-path prior before MAPPO has to learn multi-agent coordination. The expert chooses one-step actions with dynamic priority, wait-based priority boosts, and recursive local yielding when a desired cell is occupied. Pure MAPPO from scratch can look poor at low timesteps.
The training script also keeps a small optional BC anchor during MAPPO updates. This prevents the learned policy from collapsing into an all-WAIT deterministic policy while still letting PPO learn conflict avoidance.
The centralized critic uses per-agent encodings with multi-head attention and masked/pooling aggregation instead of a flat concatenated MLP. The actor and critic have separate encoders so critic updates do not overwrite BC-trained actor features during MAPPO fine-tuning. Older BC checkpoints that contain the previous shared encoder can still initialize the compatible actor and critic encoder weights; incompatible critic tensors are skipped.

When tuning poor rollouts, run the deterministic diagnostic script before looking at the GIF:

```powershell
python analyze_rollout.py configs\crossing.yaml --expert-only
python analyze_rollout.py configs\crossing.yaml --model-dir models\crossing
```

It prints whether each agent reached its goal, its first-step policy probabilities, plus move, wait, oscillation, invalid-action, no-fly-hold, unsafe-hold, and selected-action counts. If you trained a model before the conflict arbitration or reward settings changed, retrain it before comparing results.

To visualize a trained policy outside Unreal Engine:

```powershell
pip install -e ".[dev,train,viz]"
python visualize_policy.py configs\crossing.yaml --model-dir models\crossing --output runs\crossing.gif
```

The visualization renders one XY grid per Z layer. Gray cells are static obstacles, red translucent areas are active no-fly zones, circles are drones, and stars are mission goals.

For multi-scenario and staged larger-scale training, use the scripts in `curriculum/`:

```powershell
python curriculum\generate_scenarios.py --agents 8 --count 100 --out-dir configs\generated\train_8 --disjoint-start-goal --obstacle-rate 0.0 --obstacle-count 0 --no-fly-zones 0
python curriculum\pretrain_bc_multi.py --scenario-dir configs\generated\train_8 --init-model-dir models\crossing_attention_stable --model-dir models\curriculum_8_bc --updates 20000
python curriculum\train_mappo_multi.py --scenario-dir configs\generated\train_8 --init-model-dir models\curriculum_8_bc --model-dir models\curriculum_8 --timesteps 1000000
python curriculum\evaluate_scenarios.py --scenario-dir configs\generated\train_8 --model-dir models\curriculum_8
```

The main class is `utm_mappo.env.UTMMAPFEnv`. It implements the PettingZoo `ParallelEnv` interface and can be wrapped by SuperSuit for MAPPO training.

The training entry point is `train_mappo.py`. It uses a discrete MAPPO implementation with:

- shared decentralized actor over per-agent observations
- centralized attention/pooling critic over shared per-agent observation encodings
- categorical action distribution for the 7 UTM actions
- team reward for cooperative emergency behavior
- active-agent masks so finished agents remain in the global state but do not contribute actor loss

## Scenario Format

Scenarios can be loaded from a Python dictionary or YAML file:

```yaml
grid:
  dimensions: [100, 100, 10]
  blocked_cells:
    - [5, 5, 0]
missions:
  - mission_id: 1
    start: [1, 1, 2]
    goal: [8, 8, 2]
no_fly_zones:
  - zone_id: 1
    enabled: true
    min_cell: [4, 4, 1]
    max_cell: [6, 6, 1]
    start_time_step: 4
    end_time_step: 8
```

See `examples/random_rollout.py` for a minimal runnable scenario.
