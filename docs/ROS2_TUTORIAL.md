# 🤖 ROS 2 Tutorial — Wrapping the Rust Layers as ROS 2 Nodes (rclrs)

**The gap this closes:** the job listing marks "ROS Framework" mandatory, and the project had none. The cheapest fix with the best story is to wrap the Rust layers as ROS 2 nodes using **rclrs** (the Rust client library) — so bearing observations, pose estimates, and plan-accept/reject events become topics. This satisfies a hard requirement **and reinforces the Rust thread** instead of forking effort into a separate Python ROS demo.

---

## 1. The graph

```
              sensor_msgs/Image                swarm_msgs/BearingObservation
  camera ──────────────────────▶ [perception_node] ──────────────────────▶ (to estimator)
  (per vehicle)                    (Rust / rclrs)                            /bearings

  estimator ──▶ /swarm_estimate ──┐   swarm_msgs/SwarmEstimate
                (pose + cov)       │
                                   ▼
  planner ────▶ /mission_plan ──▶ [supervisor_node] ──▶ /plan_decision ──▶ (whole system)
                (Layer 3)           (Rust / rclrs)         accept/reject
                                   swarm_supervisor crate
```

Three ROS 2 packages, plus a Python bringup package:

| Package | Kind | Wraps | Topics |
|---|---|---|---|
| `swarm_msgs` | `ament_cmake` (rosidl) | — | defines `BearingObservation`, `SwarmEstimate`, `MissionPlan`, `Assignment`, `PlanDecision` |
| `swarm_supervisor_node` | `ament_cargo` (rclrs) | `swarm-supervisor` crate | sub `/mission_plan`, `/swarm_estimate` → pub `/plan_decision` |
| `swarm_perception_node` | `ament_cargo` (rclrs) | `swarm-perception` crate + `ort` | sub `/camera/image_raw` → pub `/bearings` |
| `swarm_bringup` | `ament_python` (rclpy) | Layer 2 & 3 replay | pub `/swarm_estimate`, `/mission_plan`; echo `/plan_decision` |

**Files:** `ros2_ws/src/…`, `ros2_ws/Dockerfile`.

---

## 2. Why this reinforces the Rust thread

The supervisor node is **not a re-implementation** — it depends on the exact `swarm-supervisor` crate the standalone binary and the 11 unit tests use:

```toml
# ros2_ws/src/swarm_supervisor_node/Cargo.toml
swarm-supervisor = { path = "../../../rust/swarm-supervisor" }
```

The node body is a thin rclrs shim: cache the latest `SwarmEstimate`, and on each `MissionPlan` call the library's `validate(plan, estimate, config, now)` and publish the `PlanDecision`. Same for perception — `perception_node` reuses `swarm_perception::{find_peaks, centroid, column_to_world_bearing}`, the same functions the file-driven CLI calls (I refactored those into `swarm-perception/src/lib.rs` precisely so both consumers share one source). One safety-critical code path, three front-ends (CLI, ROS node, tests).

The accept/reject decision being a **topic** is the point: any node — a logger, a fleet monitor, a ground station — can subscribe to `/plan_decision` and see exactly why a plan did or didn't reach the aircraft.

---

## 3. Build & run

The Docker daemon was down on the dev box, so the ROS workspace ships as a **reproducible Docker build** (the native Rust libraries it wraps *are* built and tested — 13 tests green). On any machine with Docker or a ROS 2 Jazzy install:

```bash
# from the repo root (build context = repo root; the image needs rust/ and the Python layers)
docker build -f ros2_ws/Dockerfile -t coop-swarm-ros .

docker run --rm -it coop-swarm-ros \
  ros2 launch swarm_bringup supervisor_demo.launch.py \
       instruction:="form a tight circle in the center"
```

Expected trace:
```
[swarm_supervisor]   swarm_supervisor up: /mission_plan + /swarm_estimate -> /plan_decision
[estimate_publisher] replaying 120 frames of r055 estimate -> /swarm_estimate
[plan_publisher]     planner source: offline-geometric; publishing plan '...' with 12 assignments
[swarm_supervisor]   plan '...' ACCEPTED
[plan_publisher]     /plan_decision: plan '...' ACCEPTED
```

Point it at a degraded world and a bad instruction to watch the gate fire on a topic:
```bash
ros2 launch swarm_bringup supervisor_demo.launch.py condition:=r035 \
     instruction:="stack everyone on one point"
# → /plan_decision: plan '...' REJECTED  violations=['SpacingTooClose {...}']
```

The Dockerfile clones `ros2-rust` into the workspace, imports the `ros2_rust_jazzy.repos`, drops the onnxruntime `.so` where the perception node's `load-dynamic` ort backend finds it (`ORT_DYLIB_PATH`), and `colcon build`s everything — the same `load-dynamic` pattern that would point at the JetPack onnxruntime on a Jetson.

### Native half (buildable right now, no ROS)

```bash
cargo build  --release --manifest-path rust/Cargo.toml   # 3 binaries + 2 libs
cargo test   --release --manifest-path rust/Cargo.toml   # 13 tests: 11 supervisor + 2 perception
```

---

## 4. Message design notes

- **`BearingObservation` carries no target identity** — just observer, world bearing, pixel, confidence. Data association is the estimator's job (it has the predicted state); keeping identity off the wire is the same seam the CLI enforces, and keeps the node swappable for a real camera.
- **`SwarmEstimate` carries `cov_trace`** — the per-vehicle marginal covariance is the trust signal the supervisor gates on. It rides the same message as the poses because they come from the same iSAM2 solve.
- **`PlanDecision` is published on every plan**, accept or reject, with human-readable `violations` — the audit trail is a topic, not a log line.

## 5. What to say in the room

- *"ROS was a hard requirement I didn't have, so I wrapped the Rust layers as rclrs nodes rather than writing a throwaway Python demo — the supervisor node depends on the same crate as the standalone binary and the unit tests, so there's one validation code path, not two."*
- *"The plan-accept/reject decision is a topic. Anything in the system can see why a plan was refused."*
- *"The perception node and the CLI share `swarm-perception`'s lib functions — I refactored the detector math into a library so the ROS wrapper couldn't drift from the tested version."*
