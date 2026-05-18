# RUPAK Reusable Launch Vehicle (VTVL) — GNC System Architecture

This repository defines the initial Guidance, Navigation, and Control (GNC) architecture baseline for the RUPAK reusable launch vehicle operating in Vertical Takeoff and Vertical Landing (VTVL) mode.

The architecture is organized around core **Systems Engineering** pillars:

## 1) Requirements Traceability
- Mission and safety objectives are translated into measurable GNC performance requirements.
- Requirements are decomposed into navigation, guidance, control, propulsion-interface, and supervisory functions.
- Verification intent (analysis, simulation, hardware-in-loop, flight test) is linked to each requirement to support lifecycle closure.

## 2) Interface Management (Propulsion ↔ Avionics)
- A deterministic avionics-propulsion data contract is defined for state estimation and control allocation.
- Electric pump telemetry (e.g., motor RPM, current draw, pump health/state) is integrated into control loops and fault handling.
- Interface timing, rate groups, and fault flags are managed to ensure stable closed-loop behavior across nominal and off-nominal operation.

## 3) Risk and Margin Management
- GNC design includes explicit operational margins for thrust authority, sensor uncertainty, estimator consistency, and timing jitter.
- Risk-driven architecture includes engine-out accommodation, degraded navigation modes, and landing hazard avoidance supervision.
- Performance margins are continuously monitored and consumed in a controlled way across powered descent and terminal landing.

---

## Documentation Layout

| Document | Scope |
|---|---|
| `docs/01_system_requirements.md` | High-level GNC requirements and propulsion-avionics data interfaces |
| `docs/02_sensor_configuration.md` | Sensor suite by flight regime and Error-State Kalman Filter (ESKF) framework |
| `docs/03_flight_control_laws.md` | Multi-rate control hierarchy, INDI core law, and control allocation |
| `docs/04_ai_supervisory_layer.md` | Safe supervisory AI for trajectory optimization, anomaly detection, and HDA |

This baseline is intended as an initial technical foundation and will evolve with simulation campaigns, subsystem maturation, and integrated flight-test evidence.
