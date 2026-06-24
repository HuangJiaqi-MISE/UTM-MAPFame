# MAPPO Curriculum Scripts

These scripts move training from one fixed scenario to staged multi-scenario training.

Run commands from the `MAPPO` directory.

## Generate Scenarios

```bash
python curriculum/generate_scenarios.py \
  --agents 8 \
  --count 100 \
  --grid 20 20 4 \
  --out-dir configs/generated/train_8 \
  --seed 8
```

For larger stages, increase both grid size and max time:

```bash
python curriculum/generate_scenarios.py --agents 16 --count 100 --grid 24 24 4 --max-time-steps 220 --out-dir configs/generated/train_16 --seed 16
python curriculum/generate_scenarios.py --agents 32 --count 100 --grid 32 32 5 --max-time-steps 320 --out-dir configs/generated/train_32 --seed 32
```

## Behavior Clone One Stage

Use offline behavior cloning first to teach the actor from expert demonstrations on the new scenario distribution.

First collect expert rollouts. The default teacher is a centralized space-time reservation planner for offline demonstration generation. By default this keeps only clean successes: every agent reaches its goal with zero unsafe, invalid, or no-fly holds. Failed or unsafe traces are skipped so the actor does not learn stuck behavior.

Datasets are tied to the current observation shape. Regenerate them after changing observation features.

```bash
python curriculum/collect_expert_dataset.py \
  --scenario-dir configs/generated/train_8 \
  --dataset-dir datasets/train_8_expert
```

Then train the actor from shuffled minibatches.

```bash
python curriculum/train_bc_dataset.py \
  --dataset-dir datasets/train_8_expert \
  --scenario-dir configs/generated/train_8 \
  --init-model-dir models/crossing_attention_stable \
  --model-dir models/curriculum_8_bc \
  --epochs 80 \
  --batch-size 512 \
  --learning-rate 1e-4
```

The older online script `curriculum/pretrain_bc_multi.py` is still available for quick experiments, but the offline dataset path is the preferred default because samples are replayed and shuffled instead of collected from one correlated online rollout stream.

To reproduce the older PIBT-style single-step teacher during collection, pass `--teacher pibt`. The default `--teacher space-time` is stronger for dataset generation, but it is still a prioritized planner rather than a complete centralized solver; scenarios it cannot solve are skipped.

## Train One Stage

Each training stage expects all YAML files in the directory to have the same mission ids and observation shape.

```bash
python curriculum/train_mappo_multi.py \
  --scenario-dir configs/generated/train_8 \
  --init-model-dir models/curriculum_8_bc \
  --model-dir models/curriculum_8 \
  --timesteps 1000000 \
  --learning-rate 5e-5 \
  --bc-anchor-coef 0.05 \
  --entropy-coef 0.001 \
  --freeze-actor-encoder \
  --rollout-steps 1024 \
  --batch-size 1024 \
  --eval-interval 20 \
  --eval-episodes 8
```

Then continue to the next scale:

```bash
python curriculum/collect_expert_dataset.py --scenario-dir configs/generated/train_16 --dataset-dir datasets/train_16_expert
python curriculum/train_bc_dataset.py --dataset-dir datasets/train_16_expert --scenario-dir configs/generated/train_16 --init-model-dir models/curriculum_8 --model-dir models/curriculum_16_bc --epochs 80 --batch-size 1024 --learning-rate 1e-4
python curriculum/train_mappo_multi.py --scenario-dir configs/generated/train_16 --init-model-dir models/curriculum_16_bc --model-dir models/curriculum_16 --timesteps 1500000 --learning-rate 5e-5 --bc-anchor-coef 0.03 --entropy-coef 0.001 --freeze-actor-encoder --rollout-steps 1024 --batch-size 1024
```

## Evaluate

Check the expert first. If the expert fails badly, more BC updates will only imitate a weak teacher.

```bash
python curriculum/evaluate_scenarios.py \
  --scenario-dir configs/generated/train_8 \
  --expert-only
```

```bash
python curriculum/evaluate_scenarios.py \
  --scenario-dir configs/generated/train_8 \
  --model-dir models/curriculum_8 \
  --csv runs/curriculum_8_eval.csv
```

Primary metrics:

- `success_rate`: fraction of scenarios where every agent reaches its goal.
- `mean_time_steps`: average rollout length.
- `mean_unsafe`: safety-gate holds from dynamic conflicts.
- `mean_oscillations`: backtracking count.
