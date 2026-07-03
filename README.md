# 🛰️ Coordinated Swarm Localization → Real Autopilot

**A GPS-denied cooperative localizer that a real flight stack actually flies on.**

This project builds a swarm-localization estimator from first principles — from a textbook SDP relaxation up to an incremental real-time smoother — and then does the thing most estimator demos never do: **it closes the loop into a real autopilot.** The final stage streams the estimator's output into ArduCopter as the vehicle's sole source of position, with GPS switched fully off. The drone flies on math we wrote from scratch.

> 💡 **The one-sentence pitch:** noisy inter-drone radio ranges go in → a trusted position estimate comes out → that estimate becomes the autopilot's sense of *where it is* → the aircraft flies. No GPS anywhere in the loop.

---

## 📖 Table of Contents

1. [The Big Idea](#-the-big-idea)
2. [What We've Built (Part A — the estimator)](#-part-a--the-estimator-days-18)
3. [Closing the Loop (Part B — the autopilot)](#-part-b--closing-the-loop-into-ardupilot)
4. [System Architecture](#-system-architecture)
5. [Why the Version Matters](#-why-the-version-matters-)
6. [How QGroundControl + MAVLink Fit Together](#-how-qgroundcontrol--mavlink-fit-together)
7. [Why 12 Drones Need 12 Simulators](#-why-12-drones-need-12-simulators)
8. [The Debugging Journey (hard-won knowledge)](#-the-debugging-journey--every-wall-we-hit)
9. [Setup & Run](#-setup--run)
10. [Future Scope](#-future-scope)

---

## 🎯 The Big Idea

A drone normally knows where it is because GPS tells it. Take GPS away — indoors, underground, under jamming, on Mars — and the autopilot is blind. It will refuse to arm, because **a flight controller with no trusted position is a liability, not an aircraft.**

Our answer: let the drones localize *each other*. Every drone measures noisy distances (via UWB radio) to its neighbors and to a few fixed anchors. Individually those ranges are useless. Collectively — solved as one big estimation problem — they pin down every drone's position **without a single satellite.**

Then we take that estimate and feed it into a real autopilot (ArduCopter SITL) through MAVLink, so the flight stack fuses *our* number as its position. That's the whole project: **become the thing GPS used to be.**

---

## 🧮 Part A — The Estimator (Days 1–8)

Eight progressive days, each building directly on the last, taking the same core problem from *"solve it perfectly, slowly, in a lab"* to *"solve it live, robustly, on real flight data."*

| Day | Theme | What it proves | Output |
|----|-------|----------------|--------|
| 1️⃣ | Centralized SDP localization | A convex (Biswas-Ye SDP) relaxation recovers positions from ranges alone | `day1_localization.png` |
| 2️⃣ | Rigidity & alignment | *When* is a range graph even solvable? Procrustes alignment + rigidity phase diagram | `day2_phase_diagram.png` |
| 3️⃣ | Distributed ADMM | The same solve with **neighbor-only** comms — no central computer | `day3_storyboard.png` |
| 4️⃣ | Robust outlier detection | Certify solutions and survive adversarial/bad ranges | `day4_outlier_detection.png` |
| 5️⃣ | Dynamic tracking | Predict-correct on a *moving* swarm | `day5_rmse_over_time.png`, `day5_swarm.gif` |
| 6️⃣ | GTSAM batch smoother | Full factor-graph smoothing over the whole trajectory | `day6_smoother_vs_causal.png` |
| 7️⃣ | Realistic dynamics + robust | PID quadrotor motion + real UWB noise (NLOS bias, multipath, dropouts) + Huber robustness | `day7_robust_smoother.png` |
| 8️⃣ | **iSAM2 incremental** | Real-time smoothing: bounded per-frame cost regardless of mission length | `day8_isam2.png`, `swarm_real_flight.gif` |

**The throughline:** Days 1–6 use clean Gaussian noise to build intuition. Days 7–8 swap in a realistic UWB model and real Crazyflie flight data (`trajectory.npy`, exported from `gym-pybullet-drones`). The estimator code barely changes — only the world around it gets harder. That discipline is deliberate: **the estimator that survives Day 8 is the one we hand to the autopilot.**

> 📌 Day 7's PID-dynamics block is explicitly marked as the swap point for a real simulator or hardware. The estimator on either side of that swap is identical.

---

## 🔗 Part B — Closing the Loop into ArduPilot

This is where the project stops being a plot and starts being an aircraft. `close_the_loop.py` is the finale:

1. **Loads** the real-flight swarm trajectory and rebuilds the Day 8 measurement world (UWB ranges with noise/NLOS/dropouts).
2. **Runs iSAM2 live**, frame by frame, producing drone 0's position estimate.
3. **Streams that estimate** into ArduCopter SITL as a `VISION_POSITION_ESTIMATE` MAVLink message at ~20 Hz.
4. ArduCopter's **EKF3 fuses it** as external navigation — with GPS fully disabled.
5. **Holds the final estimate forever** so the EKF never loses its source (a running autopilot that loses its position source falls out of the sky).

```
your iSAM2 estimator  ──VISION_POSITION_ESTIMATE──▶  EKF3 (external nav)  ──▶  vehicle state  ──▶  it flies
     (Python)                (MAVLink, 20 Hz)            (ArduCopter)
```

> ⚖️ **Scope honesty:** the estimator localizes the *whole* swarm; one SITL instance is *one* aircraft. So we feed **drone 0's** estimate in as *its* position. No pretending one SITL is twelve drones. ([See why below.](#-why-12-drones-need-12-simulators))

---

## 🏗️ System Architecture

```
┌──────────────────────────┐         ┌───────────────────────────────────────┐
│  ESTIMATOR (Python venv)  │         │        ArduCopter SITL (C++)          │
│                          │         │                                       │
│  iSAM2 / GTSAM           │         │   ┌─────────────┐   ┌──────────────┐  │
│  UWB range world         │         │   │   EKF3      │   │  Flight modes │  │
│  close_the_loop.py       │         │   │  IMU0 / IMU1│   │  GUIDED etc.  │  │
└─────────────┬────────────┘         │   └──────┬──────┘   └──────────────┘  │
              │                       └──────────┼────────────────────────────┘
              │ VISION_POSITION_ESTIMATE         │ SITL sensor sim (127.0.0.1:5501)
              │ + SET_GPS_GLOBAL_ORIGIN          │
              ▼ (UDP 14551)                      ▼
       ┌────────────────────────── MAVProxy (sim_vehicle.py) ──────────────────────────┐
       │   master: tcp:127.0.0.1:5760   ·   out → 14550 (QGC)   ·   out → 14551 (script) │
       └───────────────────────────────────┬───────────────────────────────────────────┘
                                            │ UDP 14550
                                            ▼
                                 ┌────────────────────┐
                                 │  QGroundControl    │  🗺️ map · telemetry · HUD
                                 └────────────────────┘
```

**Reading it:** MAVProxy is the hub. SITL talks to MAVProxy over TCP 5760. MAVProxy fans that out to multiple UDP consumers — QGC on 14550 for eyes-on, our script on 14551 to inject position. Keeping those on **separate ports** is not optional (details below).

---

## 🔖 Why the Version Matters ⚠️

We run **ArduPilot Copter-4.5.7**. This is not a footnote — parameter names and behavior shifted across releases, and following a guide written for the wrong version wastes hours chasing ghosts.

| Concept | On our 4.5.7 | On 4.6+ | Trap |
|--------|--------------|---------|------|
| GPS type | `GPS_TYPE` | `GPS1_TYPE` | `param set GPS1_TYPE 0` → *"Unable to find parameter"* |
| External nav enable | `VISO_TYPE` | `VISO_TYPE` | Reboot-required — set-then-forget-to-reboot = silent no-op |
| EKF source set | `EK3_SRC1_*` | `EK3_SRC1_*` | Defaults are **GPS-centric** — see the source table below |

**The lesson:** always read the param name off *your* firmware, not off a blog. Everything in this README was verified against the actual 4.5.7 source in `~/ardupilot`, not assumed.

### 🧭 The EKF3 source set that actually works GPS-denied

The single most confusing part. Each axis of the EKF needs a source, and **the defaults quietly demand GPS**:

| Param | Default | Set to | Meaning |
|-------|---------|--------|---------|
| `EK3_SRC1_POSXY` | 3 (GPS) | **6** | Horizontal position ← external nav ✅ |
| `EK3_SRC1_VELXY` | 3 (GPS) | **0** | Horizontal velocity ← none (we send position only) |
| `EK3_SRC1_POSZ`  | 1 (Baro) | **1** | **Altitude ← baro** (keep it! vision-Z fights the climb) |
| `EK3_SRC1_VELZ`  | 3 (GPS) | **0** | Vertical velocity ← none |
| `EK3_SRC1_YAW`   | 1 (Compass) | **6** | Heading ← external nav ✅ |

> 🩹 **Why `POSZ` stays on baro:** our feed sends a *constant* `down = −2`. If altitude came from external nav, the EKF would insist the drone is pinned at 2 m even as it climbs — the two EKF cores disagree, the primary flip-flops (`EKF3 lane switch 0↔1`), and altitude oscillates wildly. Baro owns height, vision owns horizontal. Clean.

---

## 🎮 How QGroundControl + MAVLink Fit Together

**MAVLink** is the lingua franca — a compact binary message protocol every part of the stack speaks. Position injection, telemetry, arming commands, mode changes: all MAVLink messages.

**One SITL, many listeners.** SITL emits one telemetry stream; MAVProxy multiplexes it. Each consumer gets its **own UDP port**:

```bash
# Launch SITL with a dedicated port for our injector, alongside QGC's default:
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --out 127.0.0.1:14551
```

- **QGC → `udp:14550`** (its default) — the map, HUD, and telemetry you watch. 🗺️
- **`close_the_loop.py` → `udpin:14551`** — where we inject `VISION_POSITION_ESTIMATE`. 💉

> 🚨 **The collision that bites everyone:** two processes cannot bind the same UDP port. Point the script at 14550 while QGC holds it and one of them goes deaf → *"communication lost."* The fix is architectural, not a workaround: **give every MAVLink client its own port.**

**Georeferencing for the map.** Vision gives a *local* NED position with no latitude/longitude. QGC's map needs a global anchor, so the script sends **one `SET_GPS_GLOBAL_ORIGIN`** (SITL default home: `−35.363261, 149.165230`, near Canberra 🇦🇺). After that, the drone appears on the map and the *"AHRS: waiting for home"* prearm clears. If you don't see the vehicle move, **pan the map to that origin** — it's flying there, not wherever QGC happened to open.

---

## 🔢 Why 12 Drones Need 12 Simulators

A natural question: *"we localize a whole swarm — why does only one drone fly?"*

Because **one ArduCopter SITL instance simulates exactly one airframe.** Its EKF, its IMUs, its motors, its flight modes — all model a single vehicle. Our estimator produces positions for all N swarm members, but a single SITL can only *be* one of them.

To actually fly the full swarm you run **N independent SITL instances**, each its own process, port block, and EEPROM, each fed *its* row of the estimate:

```bash
# Each vehicle gets its own instance id (-I), which offsets all its ports:
sim_vehicle.py -v ArduCopter -I0 --out 127.0.0.1:14551   # drone 0  → feed online[t, 0]
sim_vehicle.py -v ArduCopter -I1 --out 127.0.0.1:14561   # drone 1  → feed online[t, 1]
sim_vehicle.py -v ArduCopter -I2 --out 127.0.0.1:14571   # drone 2  → feed online[t, 2]
#  ... one instance per swarm member, N feeder streams from the SAME estimator ...
```

The estimator stays **single and centralized** — it already knows every drone. What multiplies is the *simulated hardware*: 12 drones = 12 flight controllers = 12 SITL processes. That's a fidelity choice, not a limitation of the estimator: we refuse to fake one SITL into pretending it's a formation.

> 🧵 **Production shape:** the real system splits estimation from transport — one estimator thread solving the swarm, and one lightweight ~20 Hz sender per vehicle re-streaming that vehicle's latest position, decoupled from solver speed. `close_the_loop.py` notes exactly where those seams go.

---

## 🧗 The Debugging Journey — Every Wall We Hit

This section is the real gold: the exact failures encountered going from *"SITL connected"* to *"flying GPS-denied,"* and the root cause of each — all traced through the actual firmware source, not guessed.

| 🧱 Symptom | 🔍 Root cause | ✅ Fix |
|-----------|--------------|--------|
| `PreArm: VisOdom: out of memory` | **Not** a memory error. `AP_VisualOdom` reports this whenever its backend driver is null — and the driver is only created at boot when `VISO_TYPE ≠ 0`. Setting the param without rebooting = no driver. | `param set VISO_TYPE 1` → **full SITL restart** (not soft `reboot`) |
| `reboot` → *"EOF on TCP socket / connection refused / no link"* | Soft reboot tears down SITL's TCP server; MAVProxy loses the race to reconnect. | Full `sim_vehicle.py` restart. Params persist in `eeprom.bin` (don't pass `-w`). |
| `Unable to find parameter 'GPS1_TYPE'` | Wrong version's param name. | On 4.5.7 it's **`GPS_TYPE`**. `param set GPS_TYPE 0` |
| `PreArm: AHRS: waiting for home` | Vision gives a *local* origin only; home needs a *global* lat/lon. `SET_GPS_GLOBAL_ORIGIN` is guarded — it's dropped if an origin already exists. | Script sends `SET_GPS_GLOBAL_ORIGIN` after priming the stream |
| `PreArm: AHRS: EK3 sources require GPS` | `EK3_SRC1_VELZ` still at its **GPS default** — one GPS-referencing source is enough to trip the check. | `param set EK3_SRC1_VELZ 0` (+ audit the whole source set) |
| Script starts → **QGC "communication lost"** | UDP port collision — both bound to 14550. | Give the script its own port (`--out …:14551`, `udpin:14551`) |
| `EKF3 lane switch 0↔1` + `height 5↔15` flapping | `EK3_SRC1_POSZ = 6`: constant vision-Z fights the real climb; cores disagree, primary flip-flops. | `param set EK3_SRC1_POSZ 1` (baro owns altitude) |

> 🧠 **Meta-lesson:** every one of these was solvable in seconds *once the actual firmware source was read.* Error strings lie (`"out of memory"` meant "no driver"); param guides go stale; defaults betray you. Ground truth lives in `~/ardupilot`.

---

## ⚙️ Setup & Run

### Install (estimator)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python check.py            # prints "gtsam OK"
```

### Run the estimator demos

```bash
python day1_snl.py          # ... through ...
python day8_isam2_traj.py    # iSAM2 on real flight data
python animate_swarm.py      # renders swarm_real_flight.gif
```

### Fly it (the finale)

```bash
# 1. Launch SITL with a dedicated injector port (QGC keeps 14550):
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --out 127.0.0.1:14551

# 2. One-time params (in the MAVProxy console), then RESTART SITL:
param set VISO_TYPE 1
param set GPS_TYPE 0
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_VELXY 0
param set EK3_SRC1_POSZ  1
param set EK3_SRC1_VELZ  0
param set EK3_SRC1_YAW   6

# 3. Run the loop (in the estimator venv), then arm & take off:
python close_the_loop.py
#   MAVProxy:  mode guided → arm throttle → takeoff 2
```

Watch for the payoff line: **`EKF3 IMU0 is using external nav data`** — that's your localizer becoming the drone's sense of place. 🎉

---

## 🔮 Future Scope

- 🐝 **True multi-vehicle flight** — N SITL instances driven by the single centralized estimator, flying a coordinated formation with zero GPS.
- 🧵 **Threaded production transport** — split estimation from the per-vehicle 20 Hz senders, as the code already signposts.
- 📡 **Velocity fusion** — send `VISION_SPEED_ESTIMATE` so the EKF gets velocity too (then `VELXY/VELZ` can move to external nav), tightening the estimate.
- 🛰️ **Real hardware** — swap SITL for Crazyflies + real UWB (Loco/DWM1000). The estimator is already validated on exported Crazyflie flight data.
- 🧭 **Full 6-DOF** — extend the 2-D localizer to 3-D positions and attitude, feeding a complete pose.
- 🛡️ **Onboard robustness** — push the Day 4 certification and Day 7 Huber robustness into the live loop so bad ranges can't poison the flight estimate.
- 🌍 **Decentralized at scale** — run the Day 3 distributed ADMM solver *onboard each drone*, so localization survives even the loss of the central node.
- 🤖 **PX4 parity** — mirror the pipeline onto PX4-SITL via the same MAVLink external-vision interface.

---

### 📂 Repo Map

| File | Role |
|------|------|
| `day1..day8_*.py` | The estimator, one day per stage ([Part A](#-part-a--the-estimator-days-18)) |
| `close_the_loop.py` | 🏁 The finale — iSAM2 → ArduCopter external nav |
| `feed_position.py` | Minimal external-nav injector (hover-in-place teaching version) |
| `hello_drone.py` | Simplest MAVLink telemetry listener (start here) |
| `animate_swarm.py` | Renders the flight animation GIF |
| `trajectory.npy` | Real Crazyflie flight path (from `gym-pybullet-drones`) |
| `gym-pybullet-drones/` | Physics sim; `export_trajectory.py` generates the trajectory |
| `check.py` | GTSAM sanity check |

---

> _"The drone won't fly because it has no position estimate it trusts. That is the exact hole this estimator exists to fill. Today the crutch is fake GPS. The whole point of the project is to **become the thing that satisfies that check with no GPS at all** — and now it does."_ 🛰️
