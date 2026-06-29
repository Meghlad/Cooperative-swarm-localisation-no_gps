# Coordinated Swarm ADMM Drones

This project walks through a small swarm-localization story in three progressive days:

- Day 1: centralized localization with a Biswas-Ye SDP relaxation
- Day 2: rigidity, Procrustes alignment, and the tightness phase diagram
- Day 3: distributed localization with consensus ADMM over a neighbor-only network

The code simulates a set of drones with noisy distance measurements to one another and to known anchors, then shows how localization can be solved either centrally or in a distributed way.

## Project structure

- `day1_snl.py` — centralized SDP-based localization
- `day2_rigidity.py` — alignment, rigidity analysis, and phase-diagram experiments
- `day3_distributed.py` — neighbor-only ADMM distributed solver
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

## Notes

- The examples are intentionally small and educational.
- The code is written to be easy to read and to connect the ideas from one day to the next.
- If you want to extend this project, the next natural step is to make the network more realistic (dynamic graphs, asynchronous updates, or more complex motion models).
