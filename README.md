# RUPAK Flight Control Specification

This repository contains the technical documentation baseline for the **Guidance, Navigation, and Control (GNC) Flight Control Architecture** of **RUPAK**, a reusable **VTVL (Vertical Takeoff, Vertical Landing)** launch vehicle.

## Document Structure

| File | Purpose |
| --- | --- |
| `README.md` | Program-level architecture overview and systems engineering pillars |
| `docs/01_system_requirements.md` | Mission and control-system requirements with subsystem interfaces |
| `docs/02_sensor_configuration.md` | Sensor allocation by flight regime and fusion architecture |
| `docs/03_flight_control_laws.md` | Multi-rate loop hierarchy and INDI-centered control laws |
| `docs/04_ai_supervisory_layer.md` | Isolated supervisory AI for optimization, anomaly handling, and landing hazard avoidance |

## Core Systems Engineering Pillars

### 1) Requirements Traceability

RUPAK GNC requirements are decomposed from mission objectives into verifiable subsystem requirements and control-loop allocations.

- **Top-down flow:** Mission objective -> flight phase objective -> control requirement -> software/hardware allocation
- **Bottom-up verification:** Test data and flight telemetry mapped back to each requirement ID
- **Configuration control:** Requirement changes require impact analysis across guidance, navigation, control, propulsion, and avionics interfaces

**Traceability outcomes**

- Clear requirement ownership
- Reduced integration ambiguity
- Faster anomaly triage during test campaigns

### 2) Interface Management

RUPAK uses explicit interface contracts between avionics, propulsion, sensors, and actuators.

- **Data-level definitions:** Units, rates, coordinate frames, timestamps, validity flags
- **Control authority boundaries:** Guidance commands vs attitude controller outputs vs actuator mixer outputs
- **Failure behavior contracts:** Timeouts, stale-data handling, degraded modes, and handover logic

**Interface management outcomes**

- Deterministic subsystem interaction
- Reduced coupling and integration risk
- Predictable closed-loop behavior under nominal and off-nominal conditions

### 3) Risk and Margin Management

GNC architecture includes quantified margins and risk controls for reusable operations.

- **Performance margins:** Thrust, attitude bandwidth, sensor observability, landing dispersion
- **Fault tolerance:** Engine-out handling, sensor degradation fallback, and conservative mode transitions
- **Operational gates:** Flight-rule thresholds for automated mode entry/exit and abort criteria

**Risk/margin outcomes**

- Controlled robustness against disturbances and partial failures
- Improved landing reliability across varied environmental conditions
- Measurable readiness progression from simulation to flight test

## Usage

Start with `docs/01_system_requirements.md` and read in sequence through `docs/04_ai_supervisory_layer.md`.
