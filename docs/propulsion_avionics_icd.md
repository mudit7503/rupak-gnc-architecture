# RUPAK VTVL — Propulsion / Avionics Interface Control Document

| | |
|---|---|
| **Document ID** | RUPAK-ICD-PROP-AVI-001 |
| **Revision** | Rev D |
| **Classification** | CONFIDENTIAL — PROGRAMME RESTRICTED |
| **Prepared by** | GNC / Propulsion Integration Team |
| **Approved by** | Chief Systems Engineer |
| **Date** | 2026-05-18 |

---

## 1. Scope and Purpose

This ICD defines the interface between the **Shakti electric pump-fed propulsion subsystem** and the **GNC Flight Computer (GFC)**.

It is the controlling reference for:

- Signal definitions across the propulsion/avionics boundary
- Message timing and update rates
- Fault flags and safe-state behavior
- Command authority limits and actuator feedback expectations

The propulsion configuration remains a **9-engine Shakti layout** (1 center + 8 ring) with differential command authority used for roll control.

---

## 2. Interface Overview

### 2.1 Physical and Network Paths

- Redundant CAN-FD command/telemetry network
- Independent arm/inhibit safety lines
- Dedicated discrete fire/abort lines

### 2.2 Functional Data Flow

- **Propulsion -> Avionics:** Pump, motor, chamber, and actuator telemetry
- **Avionics -> Propulsion:** Throttle, RPM targets, TVC commands, and mode commands
- **Feedback closure:** TVC position and propulsion response are continuously monitored

---

## 3. Primary Interface Contracts

### 3.1 Common Message Rules

- Every control-critical message includes `timestamp`, `sequence_id`, and `validity`.
- SI units are mandatory.
- Stale or invalid data is rejected before control updates.

### 3.2 Telemetry Groups (Propulsion to Avionics)

| Group | Example Signals | Typical Rate | Primary Use |
|---|---|---|---|
| Pump telemetry | Voltage, current, temperature, status | 100 Hz | Health and limiter logic |
| BLDC telemetry | RPM, torque estimate, fault flags | 200 Hz | Thrust estimation and anomaly detection |
| Chamber/thrust telemetry | Chamber pressure, thrust estimate | 100-500 Hz | Guidance and allocation quality |
| TVC feedback | Pitch/yaw measured position, servo health | 200-400 Hz | Attitude loop feedback closure |

### 3.3 Command Groups (Avionics to Propulsion)

| Group | Example Signals | Typical Rate | Purpose |
|---|---|---|---|
| Throttle commanding | Per-engine throttle and rate limits | 400 Hz | Rapid thrust realization |
| RPM target commanding | Per-engine BLDC targets | 200-400 Hz | Fine trim and tracking |
| TVC commanding | `tvc_pitch_cmd`, `tvc_yaw_cmd` | 400 Hz | Dual-axis pitch/yaw control |
| Mode and safety | Enable mask, propulsion mode, safe-state control | 10-50 Hz or event | Safety and phase transitions |

---

## 4. Control-Relevant Timing Alignment

| Control Function | Nominal Rate |
|---|---|
| Guidance planning | 10 Hz |
| Attitude stabilization | 100 Hz |
| Actuator command and mixing | 400 Hz |

The interface must preserve deterministic timing so propulsion response supports the full multi-rate control strategy.

---

## 5. Fault Handling and Degraded Mode Behavior

| Condition | Detection | Required Action |
|---|---|---|
| Stale telemetry | Message age beyond timeout | Hold-safe command profile and flag degraded mode |
| Invalid payload | Validity flag not nominal | Exclude from estimation/control update |
| Engine channel loss | Missing updates across persistence window | Trigger reallocation path |
| TVC disagreement | Command/feedback mismatch persists | Limit attitude demand and escalate fault |

---

## 6. Verification Focus

- Interface timing validation under expected bus loading
- Fault-injection checks for stale, invalid, and missing channels
- Engine-out reallocation readiness and stability checks
- TVC tracking and command clipping behavior verification

---

## 7. Configuration Management

All ICD changes require formal review for impacts to:

- Guidance behavior
- Navigation assumptions
- Attitude and mixing logic
- Propulsion firmware and avionics parsing

---

*End of Document — RUPAK-ICD-PROP-AVI-001 Rev D*
