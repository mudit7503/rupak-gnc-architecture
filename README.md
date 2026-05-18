# Flight Control System Documentation

This repository contains a Guidance, Navigation, and Control (GNC) documentation baseline for a reusable VTVL flight control system.

## Document Structure

| File | Purpose |
| --- | --- |
| `README.md` | Program-level overview and documentation map |
| `docs/matlab_verification_models.md` | Roll-control MATLAB verification model overview |

## Core Architecture Snapshot

- **Multi-rate control loops:**
  - Guidance loop at **10 Hz**
  - Attitude stabilization loop at **100 Hz**
  - Actuator command loop at **400 Hz**
- **Roll control:** Rapid roll correction by rapidly altering motor speeds to stop the rocket from spinning, implemented through differential throttling across **9 electric pump-driven engine channels**
- **Pitch/Yaw control:** Dual-axis TVC command path
- **Sensor strategy:** Blending sensor data from IMU, NavIC/GNSS, radar altimeter, and vision/LiDAR to keep navigation stable through all flight phases

## Systems Engineering Focus

- Clear requirement traceability from mission goals to implementation and verification
- Defined subsystem interfaces with consistent units, rates, timestamps, and fault behavior
- Practical risk and margin management for reusable operations and degraded-mode handling

## Usage

Read:

1. `docs/matlab_verification_models.md`
