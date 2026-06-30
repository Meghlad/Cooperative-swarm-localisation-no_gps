# Coordinated Swarm ADMM Drones

This project walks through a small swarm-localization story over six progressive days:

- Day 1: centralized localization with a Biswas-Ye SDP relaxation
- Day 2: rigidity, Procrustes alignment, and the tightness phase diagram
- Day 3: distributed localization with consensus ADMM over a neighbor-only network
- Day 4: robust outlier detection and certification for range measurements
- Day 5: dynamic swarm motion with predict-correct tracking
- Day 6: batch smoothing with a GTSAM factor graph

The code simulates a set of drones with noisy distance measurements to one another and to known anchors, then shows how localization and motion estimation can be solved centrally, distributedly, and with smoothing.

## Project structure

- `day1_snl.py` — centralized SDP-based localization
- `day2_rigidity.py` — alignment, rigidity analysis, and phase-diagram experiments
- `day3_distributed.py` — neighbor-only ADMM distributed solver
- `day4_outlier_detection.py` — robust range outlier detection experiments
- `day4_robust_certify.py` — certification and robust verification code
- `day5_dynamic.py` — dynamic predict-correct swarm tracking
- `day6_gtsam_smoother.py` — GTSAM batch smoothing over the full trajectory
- `requirements.txt` — Python dependencies

## Setup

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Day 1: Centralized SDP localization

Run:

```bash
python day1_snl.py
```

This generates a localization plot and saves `day1_localization.png`.

## Day 2: Rigidity and phase diagram

Run:

```bash
python day2_rigidity.py
```

This explores:

- Procrustes alignment for comparing shapes under rotation/flip/translation
- Rigidity of the sensing graph
- A phase diagram showing how connectivity and noise affect localization quality

## Day 3: Distributed ADMM localization

Run:

```bash
python day3_distributed.py
```

This simulates a distributed swarm that reaches consensus using only neighbor-to-neighbor communication and saves figures for the storyboard and convergence behavior.

## Day 4: Robust outlier detection

Run:

```bash
python day4_outlier_detection.py
```

This shows how to detect and handle bad range measurements.

Run:

```bash
python day4_robust_certify.py
```

This verifies robustness and certification under adversarial conditions.

## Day 5: Dynamic predict-correct tracking

Run:

```bash
python day5_dynamic.py
```

This simulates moving drones and evaluates a causal tracker that predicts motion and corrects with sensor measurements.

## Day 6: Batch smoother with GTSAM

Run:

```bash
python day6_gtsam_smoother.py
```

This builds a full factor graph over all times and solves it with a batch smoother, comparing performance to the Day 5 causal tracker.

## Notes

- The examples are intentionally small and educational.
- The code is written to be easy to read and to connect the ideas from one day to the next.
- If you want to extend this project, the next natural step is to make the network and motion models more realistic.
