# SunCubes Assignment — Embedded Control & Real-Time Software Engineer

## Task 1 — Real-Time Coordinate Transformation Module

### Overview

A PX4 module that performs real-time coordinate transformations on the gravity vector using live vehicle attitude data, and validates mathematical consistency every cycle.

### Architecture

The module uses PX4's ScheduledWorkItem pattern on the low-priority work queue, scheduled at 100 Hz (every 10 ms).

Each Run() cycle:
1. Checks for new vehicle_attitude data via uORB Subscription update()
2. If no new data, returns immediately (non-blocking)
3. Extracts the quaternion and builds the rotation matrix R (DCM)
4. Transforms gravity from world to body frame: g_b = R * g_w
5. Reconstructs via inverse: g_hat_w = R^T * g_b
6. Validates orthogonality, norm preservation, and reconstruction accuracy
7. Publishes results on gravity_transform_status uORB topic

Design decisions:
- ScheduledWorkItem over raw thread: PX4 work queues handle scheduling, priority, and stack management.
- Non-blocking: update() returns false immediately if no new data. No busy-waiting.
- Temporal consistency: All math within one Run() uses data from a single vehicle_attitude message.
- RTOS determinism: Constant computational load per cycle. Bounded, predictable execution time.

### Mathematical Approach

Reference Frames:
- World frame: NED (North-East-Down), PX4 standard inertial frame
- Body frame: FRD (Front-Right-Down), attached to the vehicle

PX4 provides orientation as a unit quaternion q = [w, x, y, z] in vehicle_attitude.
We convert to a Direction Cosine Matrix (DCM) R, a 3x3 orthogonal matrix with det(R) = +1.

Forward transformation: g_b(t) = R(t) * g_w
Inverse transformation: g_hat_w(t) = R^T(t) * g_b(t)

We use R^T (transpose) instead of general matrix inverse because for orthogonal matrices R^(-1) = R^T by definition. This is O(1) and avoids numerical issues.

Gravity vector as defined in the assignment: g_w = [0, 0, -9.81] m/s^2

Validation checks every cycle:
- Orthogonality: ||R * R^T - I||_F < 1e-5
- Norm preservation: | ||g_b|| - ||g_w|| | < 1e-4
- Reconstruction: ||g_hat_w - g_w|| < 1e-4

Thresholds are 10-100x above expected float32 errors to avoid false positives.

### Building and Running

Prerequisites:
- Ubuntu 22.04 or 24.04
- PX4-Autopilot v1.17+ cloned with --recursive
- Dependencies installed via: bash ./Tools/setup/ubuntu.sh

Build steps:

1. Copy the module into PX4:
   cp -r px4_modules/gravity_transform ~/PX4-Autopilot/src/modules/
   cp px4_modules/gravity_transform/msg/GravityTransformStatus.msg ~/PX4-Autopilot/msg/

2. Register the message in ~/PX4-Autopilot/msg/CMakeLists.txt
   (add GravityTransformStatus.msg alphabetically in the message list)

3. Enable the module in ~/PX4-Autopilot/boards/px4/sitl/default.px4board:
   CONFIG_MODULES_GRAVITY_TRANSFORM=y

4. Build and run SITL:
   cd ~/PX4-Autopilot
   HEADLESS=1 make px4_sitl gz_x500

5. At the pxh> prompt:
   gravity_transform start
   gravity_transform status
   listener gravity_transform_status

### File Structure

px4_modules/gravity_transform/
    CMakeLists.txt              -- PX4 module build definition
    Kconfig                     -- Module registration for board config
    GravityTransformModule.hpp  -- Class declaration
    GravityTransformModule.cpp  -- Implementation
    msg/
        GravityTransformStatus.msg  -- Custom uORB message definition


## Task 2 — CI/CD and Automated Validation Pipeline

### Overview

A GitHub Actions pipeline that automatically runs a PX4 SITL flight mission and validates control stability during a simulated laser charging hover phase.

### Why RMSE Matters for Laser Power Transfer

In SunCubes LUCY, a high-power laser beam must remain aligned with an onboard receiver on the UAV. Position drift during hover directly impacts power transfer. Beam divergence means power density drops with distance from beam center. A 0.5m position error can reduce received power significantly. Excessive oscillation risks the beam missing the receiver entirely. RMSE captures both systematic offset and random oscillation in a single metric. The 0.5m threshold represents the maximum acceptable position error for safe and efficient power transfer.

### Flight Mission Script (mission.py)

The script connects to PX4 SITL via MAVLink, arms, takes off to 10m, hovers at target coordinates for 30 seconds while recording position at 10 Hz, lands, computes RMSE, and exits with code 0 (pass) or 1 (fail).

### GitHub Actions Pipeline

Triggered on every push and pull request to main. Sets up Ubuntu 22.04, builds PX4 SITL, starts simulation in headless mode, runs the flight script, and fails the build if RMSE exceeds 0.5m or a crash is detected.

### Running SITL Validation Locally

Terminal 1 - Start SITL:
  cd ~/PX4-Autopilot
  HEADLESS=1 make px4_sitl gz_x500_baylands

Terminal 2 - Run mission:
  cd sitl_validation
  python3 mission.py
