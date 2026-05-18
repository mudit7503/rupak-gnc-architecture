# RUPAK Flight Control Documentation

This repository contains the Guidance, Navigation, and Control (GNC) documentation baseline for RUPAK, a reusable VTVL launch vehicle.

## Document Structure

| File | Purpose |
| --- | --- |
| `README.md` | Program-level overview and documentation map |
| `docs/01_system_requirements.md` | Mission and control requirements with interface responsibilities |
| `docs/02_sensor_configuration.md` | Sensor roles by flight regime and practical blending sensor data approach |
| `docs/03_flight_control_laws.md` | Flight control hierarchy and actuator command strategy |
| `docs/04_ai_supervisory_layer.md` | Supervisory AI for optimization, anomaly support, and landing hazard handling |
| `docs/propulsion_avionics_icd.md` | Propulsion-avionics interface control baseline |
| `docs/system_architecture_map.md` | High-level closed-loop architecture map |
| `docs/matlab_verification_models.md` | Roll-control MATLAB verification model overview |

## Core Architecture Snapshot

- **Multi-rate control loops:**
  - Guidance loop at **10 Hz**
  - Attitude stabilization loop at **100 Hz**
  - Actuator command loop at **400 Hz**
- **Roll control:** Rapid roll correction by rapidly altering motor speeds to stop the rocket from spinning, implemented through differential throttling across **9 Shakti electric propellant pump-driven engine channels**
- **Pitch/Yaw control:** Dual-axis TVC command path
- **Sensor strategy:** Blending sensor data from IMU, NavIC/GNSS, radar altimeter, and vision/LiDAR to keep navigation stable through all flight phases

## Systems Engineering Focus

- Clear requirement traceability from mission goals to implementation and verification
- Defined subsystem interfaces with consistent units, rates, timestamps, and fault behavior
- Practical risk and margin management for reusable operations and degraded-mode handling

## Usage

Read in order:

1. `docs/01_system_requirements.md`
2. `docs/02_sensor_configuration.md`
3. `docs/03_flight_control_laws.md`
4. `docs/04_ai_supervisory_layer.md`

Then use the ICD, architecture map, and MATLAB verification notes as supporting technical references.
