# Coordinated Swarm ADMM Drones

This project walks through a swarm-localization story over eight progressive days, building from textbook centralized estimation up to an incremental real-time smoother running on realistic drone physics and UWB radio noise:

- Day 1: centralized localization with a Biswas-Ye SDP relaxation
- Day 2: rigidity, Procrustes alignment, and the tightness phase diagram
- Day 3: distributed localization with consensus ADMM over a neighbor-only network
- Day 4: robust outlier detection and certification for range measurements
- Day 5: dynamic swarm motion with predict-correct tracking
- Day 6: batch smoothing with a GTSAM factor graph
- Day 7: realistic PID dynamics + UWB noise model + robust (Huber) smoother
- Day 8: iSAM2 incremental real-time smoother on synthetic and real flight trajectories

The code simulates a set of drones with noisy distance measurements to one another and to known anchors, then shows how localization and motion estimation can be solved centrally, distributedly, robustly, and in real time.

## Project structure

- `day1_snl.py` — centralized SDP-based localization
- `day2_rigidity.py` — alignment, rigidity analysis, and phase-diagram experiments
- `day3_distributed.py` — neighbor-only ADMM distributed solver
- `day4_robust_certify.py` — robust outlier detection, certification, and adversarial verification
- `day5_dynamic.py` — dynamic predict-correct swarm tracking
- `day6_gtsam_smoother.py` — GTSAM batch smoothing over the full trajectory
- `day7_realistic_robust.py` — PID-controlled quadrotor dynamics, realistic UWB noise (NLOS bias, multipath spikes, dropouts), plain vs Huber-robust smoother
- `day8_isam2.py` — iSAM2 incremental smoother on the same synthetic PID world as Day 7
- `day8_isam2_traj.py` — iSAM2 on a real flight trajectory loaded from `trajectory.npy`
- `animate_swarm.py` — animation helper: renders `swarm_real_flight.gif` from arrays produced by Day 8
- `gym-pybullet-drones/` — Crazyflie drone simulation environment (forked); `export_trajectory.py` inside it generates `trajectory.npy`
- `check.py` — quick GTSAM sanity-check script
- `trajectory.npy` — pre-recorded real flight trajectory from the PyBullet simulation
- `requirements.txt` — Python dependencies

### Output files produced

| File | Produced by |
|------|-------------|
| `day1_localization.png` | `day1_snl.py` |
| `day2_phase_diagram.png` | `day2_rigidity.py` |
| `day3_storyboard.png` | `day3_distributed.py` |
| `day4_outlier_detection.png` | `day4_robust_certify.py` |
| `day5_rmse_over_time.png` | `day5_dynamic.py` |
| `day5_swarm.gif` | `day5_dynamic.py` |
| `day6_smoother_vs_causal.png` | `day6_gtsam_smoother.py` |
| `day7_robust_smoother.png` | `day7_realistic_robust.py` |
| `day8_isam2.png` | `day8_isam2.py` or `day8_isam2_traj.py` |
| `swarm_real_flight.gif` | `animate_swarm.py` |

## Setup

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Verify the GTSAM install:

```bash
python check.py   # prints "gtsam OK"
```

## Day 1: Centralized SDP localization

```bash
python day1_snl.py
```

Solves a sensor network localization (SNL) problem with a Biswas-Ye SDP relaxation. Saves `day1_localization.png`.

## Day 2: Rigidity and phase diagram

```bash
python day2_rigidity.py
```

Explores Procrustes alignment for comparing shapes under rotation/flip/translation, rigidity of the sensing graph, and a phase diagram showing how connectivity and noise affect localization quality.

## Day 3: Distributed ADMM localization

```bash
python day3_distributed.py
```

Simulates a distributed swarm that reaches consensus using only neighbor-to-neighbor communication. Saves the convergence storyboard.

## Day 4: Robust outlier detection

```bash
python day4_robust_certify.py
```

Detects and handles bad range measurements via robust estimation and certification under adversarial conditions. Saves `day4_outlier_detection.png`.

## Day 5: Dynamic predict-correct tracking

```bash
python day5_dynamic.py
```

Simulates moving drones with a causal tracker: constant-velocity prediction followed by a range-measurement correction step. Saves `day5_rmse_over_time.png` and `day5_swarm.gif`.

## Day 6: Batch smoother with GTSAM

```bash
python day6_gtsam_smoother.py
```

Builds a full factor graph over all time steps and solves it with a Levenberg-Marquardt batch smoother. Compares batch smoothing to the Day 5 causal tracker on clean Gaussian noise.

## Day 7: Realistic dynamics and robust smoother

```bash
python day7_realistic_robust.py
```

Two upgrades over Day 6:

1. **Real dynamics** — each drone is a PID-controlled point-mass quadrotor tracking a smooth Lissajous path. It accelerates, turns, and saturates like a real vehicle, stressing the constant-velocity motion factor.
2. **Realistic UWB noise** — clean Gaussian noise is replaced by a model with positive NLOS bias (signal arrives late through obstructions), occasional multipath spike outliers, and random link dropouts.

The plain Gaussian smoother degrades on this data. Wrapping the range factors in a Huber m-estimator recovers most of the accuracy. Saves `day7_robust_smoother.png`.

The PID motion block is designed to be a drop-in swap for `gym-pybullet-drones` / PX4-SITL; only that block changes, the estimator is identical.

## Day 8: iSAM2 incremental real-time smoother

Day 7 solved the whole trajectory in one batch pass — a real drone can't wait. iSAM2 updates the estimate as each frame arrives, re-solving only the part of the Bayes tree touched by the new measurements, so each update is fast and bounded regardless of mission length.

### Synthetic trajectory (same world as Day 7)

```bash
python day8_isam2.py
```

Feeds the same PID + UWB data through iSAM2 with the same robust Huber factors. Shows that the incremental live estimate matches batch smoother accuracy, while per-frame update time stays flat vs. a batch re-solve that grows with mission length. Saves `day8_isam2.png`.

### Real flight trajectory from PyBullet

Generate the trajectory first (from inside `gym-pybullet-drones/`):

```bash
cd gym-pybullet-drones
python export_trajectory.py   # writes ../trajectory.npy
cd ..
```

Then run the iSAM2 smoother on it:

```bash
python day8_isam2_traj.py
```

This loads `trajectory.npy` (a real Crazyflie simulation flight) and runs the full iSAM2 pipeline on it.

### Animate the result

```bash
python animate_swarm.py
```

Renders `swarm_real_flight.gif` — frame-by-frame animation of the true flight path, the live iSAM2 estimate, the UWB sensing graph, and per-drone error sticks.

## Notes

- Days 1–6 use clean Gaussian noise to build intuition; Days 7–8 replace that with a realistic UWB model.
- The gym-pybullet-drones block in Day 7 is explicitly marked as the swap point for real simulator / hardware data; the estimator is unchanged.
- Each day builds directly on the previous one: Day 8's iSAM2 uses Day 7's dynamics and noise model and Day 5's causal tracker as its initialization oracle.
