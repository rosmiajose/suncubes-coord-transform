#!/usr/bin/env python3
"""
Automated SITL flight mission using MAVSDK for SunCubes CI/CD validation.
"""
import asyncio
import sys
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

TARGET_N = 0.0
TARGET_E = 0.0
TARGET_D = -10.0
HOVER_DURATION = 30
RMSE_THRESHOLD = 0.5

async def main():
    print("=" * 60)
    print("SunCubes SITL Validation - Charging Phase Stability Test")
    print("=" * 60)

    drone = System()
    print("\nConnecting to drone...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            break

    # Set initial setpoint before starting offboard
    print("\nSetting initial offboard setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(TARGET_N, TARGET_E, TARGET_D, 0.0))

    # Start offboard mode
    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        sys.exit(1)
    print("Offboard started")

    # Arm
    print("Arming...")
    await drone.action.arm()
    print("Armed!")

    # Wait for takeoff
    print(f"Taking off to {-TARGET_D}m...")
    await asyncio.sleep(10)

    # Check altitude
    async for pos in drone.telemetry.position_velocity_ned():
        alt = -pos.position.down_m
        print(f"Current altitude: {alt:.1f}m")
        break

    # Charging Phase
    print(f"\n--- Charging Phase: hovering for {HOVER_DURATION}s ---")
    positions = []
    start_time = time.time()

    while time.time() - start_time < HOVER_DURATION:
        await drone.offboard.set_position_ned(PositionNedYaw(TARGET_N, TARGET_E, TARGET_D, 0.0))
        async for pos in drone.telemetry.position_velocity_ned():
            positions.append((pos.position.north_m, pos.position.east_m, pos.position.down_m))
            break
        await asyncio.sleep(0.1)

    print(f"Collected {len(positions)} samples")

    # Stop offboard and land
    print("\nLanding...")
    try:
        await drone.offboard.stop()
    except Exception:
        pass
    await drone.action.land()

    # Wait for disarm
    print("Waiting for landing...")
    async for armed in drone.telemetry.armed():
        if not armed:
            print("Disarmed (landed)")
            break

    # Analysis
    print("\n" + "=" * 60)
    print("Performance Analysis")
    print("=" * 60)

    target = (TARGET_N, TARGET_E, TARGET_D)
    if len(positions) == 0:
        print("No position data collected!")
        sys.exit(1)

    errors = [(p[0]-target[0])**2 + (p[1]-target[1])**2 + (p[2]-target[2])**2 for p in positions]
    rmse = math.sqrt(sum(errors) / len(errors))

    print(f"Target:    ({TARGET_N}, {TARGET_E}, {TARGET_D})")
    print(f"Samples:   {len(positions)}")
    print(f"RMSE:      {rmse:.4f} m")
    print(f"Threshold: {RMSE_THRESHOLD} m")

    if rmse > RMSE_THRESHOLD:
        print(f"RESULT: FAIL (RMSE {rmse:.4f}m exceeds {RMSE_THRESHOLD}m)")
        sys.exit(1)
    else:
        print(f"RESULT: PASS (RMSE {rmse:.4f}m within {RMSE_THRESHOLD}m)")
        sys.exit(0)

if __name__ == "__main__":
    asyncio.run(main())
