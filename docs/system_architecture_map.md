# RUPAK VTVL — GNC System Architecture Map

| | |
|---|---|
| **Document ID** | RUPAK-ARCH-GNC-001 |
| **Revision** | Rev C |
| **Classification** | CONFIDENTIAL — PROGRAMME RESTRICTED |
| **Prepared by** | GNC Architecture Team |
| **Date** | 2026-05-18 |
| **Diagram Tool** | Mermaid.js v10+ |

---

## 1. Purpose

This document presents a high-level view of the RUPAK closed-loop GNC architecture. It shows how sensor data moves through navigation and control, then into propulsion and attitude actuation.

---

## 2. Closed-Loop Architecture (Functional View)

```mermaid
flowchart TB

    subgraph NAV["Navigation Suite — Sensor Data Blending"]
      IMU["IMU Cluster\nHigh-rate inertial sensing\nPrimary fast motion input"]
      GNSS["NavIC/GNSS\nAbsolute position and velocity correction"]
      RADAR["Radar Altimeter\nReliable descent altitude support"]
      VISION["Flash LiDAR / Cameras\nLanding hazard and relative pose support"]
      FUSION["Navigation Fusion Core\nBlends valid sensor updates\nPublishes stable state estimate"]

      IMU --> FUSION
      GNSS --> FUSION
      RADAR --> FUSION
      VISION --> FUSION
    end

    subgraph GNC["GNC Processing Core — Multi-Rate Loops"]
      G10["Guidance Loop (10 Hz)\nBuilds feasible trajectory references"]
      G100["Attitude Stabilization Loop (100 Hz)\nTracks desired attitude and angular rates"]
      G400["Actuator Mixing Loop (400 Hz)\nGenerates final per-actuator commands"]

      G10 --> G100
      G100 --> G400
    end

    subgraph ACT["Propulsion and Actuation"]
      TVC["Dual-Axis TVC\nPrimary pitch/yaw authority"]
      ROLL["Differential Throttle Across 9 Shakti Engine Channels\nPrimary roll authority while preserving thrust target"]
      AUX["Auxiliary Attitude Actuation\nUsed when needed by phase and envelope"]
    end

    FUSION --> G10
    FUSION --> G100
    G400 --> TVC
    G400 --> ROLL
    G400 --> AUX
```

---

## 3. Architecture Summary

| Subsystem | Primary Role | Typical Rate |
|---|---|---|
| Navigation Suite | Sensor data blending and state estimation | Up to 400 Hz internal propagation |
| Guidance Loop | Builds mission-feasible references | 10 Hz |
| Attitude Loop | Smooth attitude stabilization | 100 Hz |
| Mixing/Allocation Loop | Rapid actuator command realization with limits | 400 Hz |
| TVC Path | Pitch and yaw control | 400 Hz command path |
| Differential Throttle Path | Roll control using 9 Shakti engine channels | 100-400 Hz command participation |

---

## 4. Timing and Integration Notes

- Keep deterministic scheduling boundaries between 10 Hz, 100 Hz, and 400 Hz tasks.
- Prioritize attitude stabilization when actuator limits are reached.
- Maintain robust stale-data handling on all control-critical interfaces.
- Keep feedback closure active for TVC and propulsion telemetry for stable command tracking.

---

*End of Document — RUPAK-ARCH-GNC-001 Rev C*
