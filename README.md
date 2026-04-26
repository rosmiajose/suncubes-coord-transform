# SunCubes Assignment — Embedded Control & Real-Time Software Engineer

## Task 1 — Real-Time Coordinate Transformation Module (Out-of-Tree)

### Overview

This module runs inside PX4 and performs a simple but important check: it transforms the gravity vector between coordinate frames using live attitude data and verifies that the mathematical consistency is consistent across the whole process.
This module is implemented as an out-of-tree module, so PX4 itself is not modified. This makes it easier to maintain and upgrade PX4 separately.

### Architecture

The module uses PX4's **ScheduledWorkItem** pattern on the low-priority work queue (`lp_default`), scheduled at 100 Hz (every 10 ms).

Each `Run()` cycle:

1. Checks for new `vehicle_attitude` data via `uORB::Subscription::update()`
2. If no new data, returns immediately (non-blocking)
3. If new data is available, it extracts the quaternion and builds the rotation matrix R (DCM - Direction Cosine Matrix)
4. Transforms gravity from world to body frame: `g_b = R * g_w`
5. Reconstructs via inverse: `g_hat_w = R^T * g_b`
6. Validates orthogonality, norm preservation, and reconstruction accuracy
7. Logs results via PX4 logging system at 1 Hz

**Design decisions:**

- **ScheduledWorkItem over raw thread:** Used 'ScheduledWorkItem' instead of a custom thread because PX4 already provides a work queue that handles scheduling, priority, timing and stack management. For this kind of periodic task, a separate thread didn't add any benefit. 
- **Non-blocking:** `update()` returns false immediately, exiting if no new data. No busy-waiting or blocking operations. This prevents unnecessary CPU usage and maintains system responsiveness. 
- **Temporal consistency and Synchronization:** All math computations within one `Run()` uses data from a single `vehicle_attitude` message with one attitude timestamp. No cross-timestep data mixing. 
- **RTOS determinism:** Constant computational load per cycle (3 matrix operations, 3 scalar comparisons). Achieves predictable execution, which is important for real-time systems.
- **Out-of-tree:** Module lives entirely outside PX4's source tree, built via `EXTERNAL_MODULES_LOCATION`. This allows clean PX4 version upgrades without merge conflicts.

### Mathematical Approach

**Reference Frames:** PX4 uses:

- World frame: NED (North-East-Down) — PX4's standard inertial frame
- Body frame: FRD (Front-Right-Down) — attached to the vehicle

**Rotation Representation:**

PX4 provides orientation as a unit quaternion `q = [w, x, y, z]` in `vehicle_attitude`. We convert the quaternion to a Direction Cosine Matrix (DCM) `R`, which is a 3×3 orthogonal matrix with `det(R) = +1`, that confirms R is a proper rotation (not a reflection), guaranteed by construction since R is derived from a unit quaternion.

**Transformations:**

- Forward (world → body): `g_b(t) = R(t) · g_w`
- Inverse (body → world): `ĝ_w(t) = R^T(t) · g_b(t)`

We use `R^T` (transpose) instead of the general matrix inverse because for orthogonal matrices `R^(-1) = R^T` by definition. This is O(1) and avoids the numerical issues associated with general matrix inversion.

**Gravity vector** as defined in the assignment: `g_w = [0, 0, -9.81] `

**Validation checks (every cycle at 100 Hz):** Each cycle ensures three checks for accuracy:

- Orthogonality: `||R · R^T - I||_F < 1e-5` - a valid rotation matrix R must be orthogonal, meaning it must equal the identity matrix I. We measure this using the Frobenius norm, which treats the 3×3 matrix as a 9-element vector and computes its Euclidean length. A result near zero confirms R is a valid rotation.
- Norm preservation: `| ||g_b|| - ||g_w|| | < 1e-4` - rotation must not change a vector's magnitude. The length of gravity in the body frame must equal the length of gravity in the world frame (both should be 9.81 m/s²).
- Reconstruction: `||ĝ_w - g_w|| < 1e-4` - after transforming gravity from world-to-body frame and back, the result must match the original. A non-zero error indicates numerical drift or a faulty rotation matrix.

Thresholds are set 10–100× above expected float32 arithmetic errors to avoid false positives while catching genuine failures.

### Building and Running

**Prerequisites:**

- Ubuntu 22.04 or 24.04
- PX4-Autopilot v1.17+ cloned with `--recursive`
- Dependencies installed via: `bash ./Tools/setup/ubuntu.sh`

**Build steps:**

1. Clone this repository:
```bash
git clone https://github.com/rosmiajose/suncubes-coord-transform.git
```

2. Build PX4 SITL with the external module (no PX4 modifications needed):
```bash
cd ~/PX4-Autopilot
HEADLESS=1 EXTERNAL_MODULES_LOCATION=~/suncubes-coord-transform/px4_modules make px4_sitl gz_x500
```

3. At the `pxh>` prompt:
```bash
gravity_transform start    # Start the module
gravity_transform stop     # Stop the module
gravity_transform status   # Show module info
```
**Note:** The module logs at 1 Hz. To stop it, type `gravity_transform stop` at the pxh> prompt (the command will execute even if log output is visible), or press Ctrl+C to exit PX4 entirely.

**Expected output (vehicle level, 1 Hz logging):**
```
INFO [gravity_transform] g_body:[-0.001,0.002,-9.810] recon:[0.000,0.000,-9.810] err:9.5e-07 OK
```

---

## Task 2 — CI/CD and Automated Validation Pipeline

### Overview

An automated SITL flight validation pipeline that executes a complete flight mission and validates the vehicle's control stability during a simulated "laser charging" hover phase.

### Why RMSE Matters for Laser Power Transfer

In SunCubes LUCY, a high-power laser beam must remain aligned with an onboard receiver on the UAV. Position drift during hover directly impacts power transfer:

- Beam divergence means power density drops with distance from beam center
- A 0.5m position error at typical operating distances can reduce received power significantly
- Excessive oscillation risks the beam missing the receiver entirely, causing safety hazards
- RMSE captures both systematic offset and random oscillation in a single metric

The 0.5m threshold represents the maximum acceptable position error for maintaining safe and efficient power transfer during the charging window.

### Flight Mission Script (mission.py)

The script uses **Python MAVSDK** to interface with a PX4 SITL instance and performs:

1. Connects to PX4 SITL and waits for global position estimate
2. Sets OFFBOARD mode with position setpoints
3. Arms the vehicle
4. Takes off to 10m altitude
5. **Charging Phase:** Hovers at target coordinate `(0, 0, -10)` NED for 30 seconds, recording position at ~10 Hz
6. Lands and waits for disarm
7. Computes RMSE of recorded positions vs target setpoint
8. Exits with code 0 (PASS) if RMSE < 0.5m, or code 1 (FAIL) otherwise

**Tested locally: RMSE = 0.0298m — PASS**

### GitHub Actions Pipeline

The `.github/workflows/sitl_validation.yml` file defines the CI/CD pipeline:

1. Sets up Ubuntu 22.04 with PX4 dependencies and Gazebo Harmonic
2. Compiles PX4 SITL firmware
3. Starts SITL simulator in headless mode in the background
4. Runs the flight mission script
5. Fails the build if RMSE exceeds 0.5m or a crash is detected
6. Uploads PX4 logs as artifacts on failure for debugging

Note: The pipeline is set to manual trigger (`workflow_dispatch`) as PX4 SITL builds require more resources than GitHub's free-tier runners typically provide.

### Running SITL Validation Locally

**Prerequisites:**

- PX4-Autopilot built with SITL support
- Python MAVSDK installed: `pip3 install mavsdk`

**Steps:**

Terminal 1 — Start SITL:
```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
```

Terminal 2 — Run the mission (after `pxh>` appears in Terminal 1):
```bash
cd suncubes-coord-transform/sitl_validation
python3 mission.py
```

**Expected output:**
```
SunCubes SITL Validation - Charging Phase Stability Test
Connecting to drone...
Connected!
Waiting for global position estimate...
Global position OK
OFFBOARD mode set
Armed!
Taking off to 10.0m...
--- Charging Phase: hovering for 30s ---
Collected 244 samples
Landing...
Landed and disarmed
RMSE: 0.0298 m
RESULT: PASS (RMSE 0.0298m within 0.5m)
```

---

## Repository Structure

```
suncubes-coord-transform/
├── px4_modules/
│   ├── CMakeLists.txt                          # Out-of-tree entry point
│   └── gravity_transform/
│       ├── CMakeLists.txt                      # Module build definition
│       ├── Kconfig                             # Module registration
│       ├── GravityTransformModule.hpp          # Class declaration
│       └── GravityTransformModule.cpp          # Implementation
├── sitl_validation/
│   └── mission.py                              # Automated flight script (MAVSDK)
├── .github/
│   └── workflows/
│       └── sitl_validation.yml                 # CI/CD pipeline
└── README.md
```
