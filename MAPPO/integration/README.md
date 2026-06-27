# UE-MAPPO Emergency Recovery Prototype

This directory contains a dependency-light Python prototype for connecting UE fault injection snapshots to the trained MAPPO actor.

## Model Files

Copy a trained model directory from the server to the local machine. The directory must contain `model.pt`.

Recommended first checkpoint:

```text
models/utm8_100x100_obs_mappo_ft2_deadlock/model.pt
```

The service can run on CPU:

```bash
python -m integration.emergency_service \
  --mode mappo_shield \
  --model-dir models/utm8_100x100_obs_mappo_ft2_deadlock \
  --request integration/demo_request.json \
  --device cpu
```

## Modes

```text
no_recovery   Always returns WAIT.
rule          Greedy rule-based local movement with safety filtering.
mappo         MAPPO argmax action without dynamic safety filtering.
mappo_shield  MAPPO action ranking with safety filtering and fallback.
```

## One-Shot Inference

Run from the `MAPPO` directory:

```bash
python -m integration.emergency_service \
  --mode mappo_shield \
  --model-dir models/utm8_100x100_obs_mappo_ft2_deadlock \
  --request integration/demo_request.json
```

The response contains raw MAPPO probabilities, selected actions, safety filter status, and timing.

## HTTP Service

Run:

```bash
python -m integration.emergency_service \
  --serve \
  --host 127.0.0.1 \
  --port 8765 \
  --mode mappo_shield \
  --model-dir models/utm8_100x100_obs_mappo_ft2_deadlock
```

Then POST an emergency-step JSON request to:

```text
http://127.0.0.1:8765/step
```

The prototype uses only `POST /step`. The UE client should send one emergency decision step per request.

## Smoke Tests

Run all four modes and generated safety-filter conflict cases:

```bash
python -m integration.run_smoke_tests \
  --model-dir models/utm8_100x100_obs_mappo_ft2_deadlock \
  --device cpu
```

To also write the generated conflict requests as JSON files:

```bash
python -m integration.run_smoke_tests \
  --model-dir models/utm8_100x100_obs_mappo_ft2_deadlock \
  --device cpu \
  --write-requests integration/test_requests
```

The generated cases cover:

```text
rid_occupied_block
no_fly_block
downwash_block
failed_occupancy_block
swap_risk_block
```

## Observation Contract

The prototype builds the same 645-dimensional observation used by the current MAPPO training setup:

```text
20 state features + 5 local channels * 5 * 5 * 5 = 645
```

RID neighbors are inserted into the local occupancy channel. The first prototype assumes RID positions are already converted to grid cells.
