# MAPPO Curriculum Scripts

These scripts move training from one fixed scenario to staged multi-scenario training.

Run commands from the `MAPPO` directory.

## Generate Scenarios

```bash
python curriculum/generate_scenarios.py \
  --agents 8 \
  --count 100 \
  --grid 10 10 5 \
  --disjoint-start-goal \
  --min-goal-distance 2 \
  --max-goal-distance 8 \
  --obstacle-rate 0.0 \
  --obstacle-count 0 \
  --no-fly-zones 0 \
  --out-dir configs/generated/train_8 \
  --seed 8
```

For larger stages, keep the same `10 x 10 x 5` environment and increase the
agent count only after the smaller stage is stable:

```bash
python curriculum/generate_scenarios.py --agents 16 --count 100 --grid 10 10 5 --max-time-steps 220 --out-dir configs/generated/train_16 --seed 16
python curriculum/generate_scenarios.py --agents 32 --count 100 --grid 10 10 5 --max-time-steps 320 --out-dir configs/generated/train_32 --seed 32
```

Useful scenario controls:

- `--disjoint-start-goal`: require all starts and goals to be globally unique.
- `--min-goal-distance N` and `--max-goal-distance N`: constrain each mission's Manhattan start-goal distance.
- `--obstacle-rate R`: sample independent single-cell obstacles. Defaults to 0.
- `--obstacle-count N`: sample N cuboid obstacle blocks.
- `--obstacle-size-min X Y Z` and `--obstacle-size-max X Y Z`: cuboid obstacle size range.
- `--no-fly-zones N`: sample N temporal no-fly cuboids.
- `--no-fly-size-min X Y Z` and `--no-fly-size-max X Y Z`: no-fly zone size range.

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

## Compare PIBT And CBS Teachers

Use the same candidate scenarios when comparing teachers. This tells us whether a stronger centralized CBS teacher produces better student policies than the faster PIBT-style online heuristic.

Start with a small pool because the current CBS teacher is a bounded CPU baseline. It is intended for teacher-quality experiments before CUDA integration.

```bash
python curriculum/generate_scenarios.py \
  --agents 8 \
  --count 50 \
  --grid 10 10 5 \
  --max-time-steps 180 \
  --out-dir configs/generated/teacher_compare_8_train_50 \
  --seed 8801 \
  --pattern mixed \
  --disjoint-start-goal \
  --min-goal-distance 2 \
  --max-goal-distance 8 \
  --obstacle-rate 0.0 \
  --obstacle-count 0 \
  --no-fly-zones 0
```

Collect PIBT clean trajectories:

```bash
python curriculum/collect_expert_dataset.py \
  --scenario-dir configs/generated/teacher_compare_8_train_50 \
  --dataset-dir datasets/teacher_compare_8_pibt_clean \
  --teacher pibt \
  --overwrite
```

Collect CBS clean trajectories. By default, CBS runs without a time, node, or low-level expansion budget, so difficult scenarios may take a long time:

```bash
python curriculum/collect_expert_dataset.py \
  --scenario-dir configs/generated/teacher_compare_8_train_50 \
  --dataset-dir datasets/teacher_compare_8_cbs_clean \
  --teacher cbs \
  --overwrite
```

Then train one BC model per teacher:

```bash
python curriculum/train_bc_dataset.py \
  --dataset-dir datasets/teacher_compare_8_pibt_clean \
  --scenario-dir configs/generated/teacher_compare_8_train_50 \
  --init-model-dir models/crossing_attention_stable \
  --model-dir models/teacher_compare_8_pibt_bc \
  --epochs 60 \
  --batch-size 512 \
  --learning-rate 1e-4

python curriculum/train_bc_dataset.py \
  --dataset-dir datasets/teacher_compare_8_cbs_clean \
  --scenario-dir configs/generated/teacher_compare_8_train_50 \
  --init-model-dir models/crossing_attention_stable \
  --model-dir models/teacher_compare_8_cbs_bc \
  --epochs 60 \
  --batch-size 512 \
  --learning-rate 1e-4
```

Use a separate test pool for policy comparison:

```bash
python curriculum/generate_scenarios.py \
  --agents 8 \
  --count 100 \
  --grid 10 10 5 \
  --max-time-steps 180 \
  --out-dir configs/generated/teacher_compare_8_test_100 \
  --seed 8802 \
  --pattern mixed \
  --disjoint-start-goal \
  --min-goal-distance 2 \
  --max-goal-distance 8 \
  --obstacle-rate 0.02 \
  --obstacle-count 1 \
  --obstacle-size-min 1 1 1 \
  --obstacle-size-max 2 2 1 \
  --no-fly-zones 0

python curriculum/evaluate_scenarios.py \
  --scenario-dir configs/generated/teacher_compare_8_test_100 \
  --model-dir models/teacher_compare_8_pibt_bc \
  --csv runs/teacher_compare_8_pibt_bc_test.csv

python curriculum/evaluate_scenarios.py \
  --scenario-dir configs/generated/teacher_compare_8_test_100 \
  --model-dir models/teacher_compare_8_cbs_bc \
  --csv runs/teacher_compare_8_cbs_bc_test.csv
```

Compare both teacher datasets first. If CBS stores far more clean scenarios or produces shorter/safer trajectories, then its BC model has a meaningful chance to outperform PIBT. If CBS is too slow on difficult scenarios, you can re-run with explicit safety limits such as `--cbs-max-seconds 60`, `--cbs-max-nodes 50000`, or `--cbs-max-low-level-expansions 1000000`.

## Add DAgger Recovery Data

Clean expert rollouts only show the actor states that the teacher visits. If the trained actor drifts into a different state, it may not know how to recover. Use DAgger-style collection to roll in with the current actor and label those visited states with the online priority-aware teacher.

This command copies the clean BC dataset into a new aggregate dataset, then appends recovery samples from the current policy:

```bash
python curriculum/collect_dagger_dataset.py \
  --scenario-dir configs/generated/train_8_easy_30 \
  --base-dataset-dir datasets/train_8_easy_30_pibt_clean_obs645 \
  --policy-model-dir models/curriculum_8_easy_30_bc_obs645 \
  --dataset-dir datasets/train_8_easy_30_dagger_obs645 \
  --episodes-per-scenario 2 \
  --epsilon 0.05 \
  --overwrite
```

Then continue BC from the same actor on the aggregate dataset:

```bash
python curriculum/train_bc_dataset.py \
  --dataset-dir datasets/train_8_easy_30_dagger_obs645 \
  --scenario-dir configs/generated/train_8_easy_30 \
  --init-model-dir models/curriculum_8_easy_30_bc_obs645 \
  --model-dir models/curriculum_8_easy_30_bc_obs645_dagger \
  --epochs 40 \
  --batch-size 512 \
  --learning-rate 5e-5
```

Evaluate before moving to MAPPO fine-tuning:

```bash
python curriculum/evaluate_scenarios.py \
  --scenario-dir configs/generated/train_8_easy_30 \
  --model-dir models/curriculum_8_easy_30_bc_obs645_dagger
```

Useful DAgger collection switches:

- `--epsilon`: adds random valid roll-in actions, which creates more recovery states.
- `--stochastic`: samples from the actor policy instead of using argmax actions.
- `--teacher-rollin-prob`: occasionally executes teacher actions while still labeling every stored state with teacher actions.
- `--only-disagreements`: stores only states where actor and teacher actions differ.

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
