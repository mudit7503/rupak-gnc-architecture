# RUPAK Flight Control Documentation

This repository contains the documentation baseline for the **Guidance, Navigation, and Control (GNC) architecture** of **RUPAK**, a reusable **VTVL (Vertical Takeoff, Vertical Landing)** launch vehicle.

## Document Structure

| File | Purpose |
| --- | --- |
| `README.md` | Program-level architecture overview and documentation map |
| `docs/01_system_requirements.md` | Mission and control-system requirements with interface responsibilities |
| `docs/02_sensor_configuration.md` | Sensor suite roles by flight regime and sensor data blending approach |
| `docs/03_flight_control_laws.md` | Multi-rate control hierarchy and actuator command strategy |
| `docs/04_ai_supervisory_layer.md` | Isolated supervisory AI for optimization, anomaly support, and landing hazard handling |
| `docs/propulsion_avionics_icd.md` | Propulsion-avionics interface control baseline |
| `docs/system_architecture_map.md` | High-level closed-loop architecture map |
| `docs/matlab_verification_models.md` | Verification model overview for roll control and sensor-fusion behavior |

## Core Architecture Snapshot

- **Multi-rate control loops:**
  - Guidance loop at **10 Hz**
  - Attitude stabilization loop at **100 Hz**
  - Actuator mixing and command loop at **400 Hz**
- **Roll control:** Differential command shaping across **9 Shakti electric propellant pump-driven engine channels**
- **Pitch/Yaw control:** **Dual-axis TVC** command path
- **Sensor suite purpose:** Blend IMU, NavIC/GNSS, radar altimeter, and vision/LiDAR data for stable navigation and precision landing support

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
