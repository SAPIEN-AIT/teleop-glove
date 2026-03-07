# Sapien Custom Data Glove — Controller Firmware

> Part of the **Sapien Project** — a humanoid robot based on the Berkley Humanoid Lite.

**Authors:** Karol Wickel · Edoardo Tadini

---

## Overview

Embedded firmware and sensor fusion library for a wearable glove controller that captures full hand kinematics in real time. It combines **IMU-based orientation tracking** for the wrist and MCP joints with **flex resistor-based angle estimation** for the PIP and DIP joints, fusing all data into a unified hand pose representation.

The system is designed to drive robotic hands, prosthetics, VR hand tracking, or any application requiring precise, low-latency hand pose data over wireless communication.

---

## Hardware

| Component | Part | Purpose |
|---|---|---|
| Microcontroller | ESP32-S3 | Main compute, Wi-Fi/BLE, USB |
| IMU | ICM-20948 (×N) | Wrist + MCP joint orientation |
| I²C Multiplexer | TCA9548A | Address multiple IMUs on one bus |
| Flex sensors | Resistive strip (×8) | PIP + DIP joint angles |
| Bus | I²C | IMU and mux communication |
| ADC | ESP32-S3 internal | Flex sensor readout |

### Joint Coverage

```
         MCP          PIP          DIP
Finger:  [IMU] -----> [Flex] ----> [Flex]
Thumb:   [IMU] -----> [Flex] ----> [Flex]
Wrist:   [IMU]
```

The **MCP joints** (knuckles) and **wrist** use ICM-20948 IMUs because they involve complex 3D rotation that a simple flex sensor cannot capture. The **PIP and DIP joints** (middle and fingertip knuckles) are hinge joints — single-axis bending — which flex strips measure accurately and cheaply.

---

## Software Architecture

```
┌─────────────────────────────────────────────────┐
│                  Application Layer               │
│       (pose serialization, REST API over Wi-Fi)  │
├─────────────────────────────────────────────────┤
│                  Fusion Layer                    │
│    MadgwickFilter    │    FlexAngleEstimator     │
├──────────────────────┼──────────────────────────┤
│              Sensor Driver Layer                 │
│   ICM20948 (I²C)     │    ADC (flex strips)      │
├─────────────────────────────────────────────────┤
│               Math Library                      │
│              Quaternion.cpp/.h                  │
└─────────────────────────────────────────────────┘
```

### Quaternion Library

The project includes a full quaternion math library (`Quaternion.cpp`) implementing:

- Hamilton product `⊗` — the core non-commutative multiplication
- Conjugate, inverse, normalization
- Sandwich product `q ⊗ p ⊗ q⁻¹` — 3D rotation via `rotate_by()`
- Angular velocity derivative `dq/dt = ½ q ⊗ ω`
- Euler and RK4 integration for orientation propagation
- Joint angle extraction via `joint_angle()` — maps quaternion to degrees

### Sensor Fusion

IMU orientation is estimated using the **Madgwick filter** — a gradient-descent complementary filter that fuses accelerometer, gyroscope, and magnetometer data. It was chosen over the Kalman filter for its lower computational load, suitability for the ESP32-S3's processing budget, and single tunable parameter `β`.

The filter outputs a unit quaternion per IMU, which is then used to compute relative joint angles:

```
q_joint = q_proximal × q_distal*
θ = joint_angle(q_joint)
```

---

## ICM-20948 — I²C Setup

The ICM-20948 is a 9-axis IMU (accel + gyro + magnetometer) communicating over I²C.

**Default I²C addresses:**
- `0x68` — AD0 pin LOW
- `0x69` — AD0 pin HIGH

Since the ICM-20948 only supports two addresses, a **TCA9548A I²C multiplexer** is used to connect all IMUs on the same SDA/SCL lines. The TCA9548A exposes 8 independent channels, each addressable by selecting the channel register before communicating with the IMU behind it:

```cpp
// Select TCA9548A channel before talking to IMU
void tca_select(uint8_t channel) {
    Wire.beginTransmission(TCA9548A_ADDR);  // 0x70
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// Example: read IMU on channel 3
tca_select(3);
icm.readSensor();
```

**TCA9548A address:** `0x70` (A0/A1/A2 all LOW) — adjustable up to `0x77` via address pins.

**Recommended IMU configuration:**

| Parameter | Value |
|---|---|
| Gyroscope range | ±500 °/s |
| Accelerometer range | ±4g |
| Sample rate | 100 Hz |
| Low-pass filter | 20 Hz |
| Magnetometer mode | Continuous 100 Hz |

---

## Quaternion Convention

This project uses the **Hamilton convention**: `q = (w, x, y, z)` stored as `(_q1, _q2, _q3, _q4)`.

A rotation by angle θ around unit axis **n̂** is encoded as:

```
q = cos(θ/2) + sin(θ/2) · (nx·i + ny·j + nz·k)
```

The relative orientation between two segments is computed as:

```cpp
Quaternion q_relative = q_proximal * q_distal.conjugate();
float angle_deg = q_relative.joint_angle();
```

---

## Building

### Requirements

- ESP-IDF v5.x or Arduino-ESP32 core
- C++17
- `<cmath>` for quaternion math

### Compile (ESP-IDF)

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

### Compile math library standalone (for testing)

```bash
g++ -std=c++17 -O2 -o test Quaternion.cpp test_main.cpp -lm
./test
```

---

## Communication — REST API

The ESP32-S3 connects over Wi-Fi and exposes a lightweight HTTP REST API to stream hand pose data to a host application (robot controller, simulation, etc.).

### Endpoints

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/pose` | Returns full hand pose as JSON |
| `GET` | `/pose/wrist` | Wrist quaternion only |
| `GET` | `/pose/finger/:id` | Single finger angles (0–4) |
| `POST` | `/calibrate` | Trigger sensor recalibration |
| `GET` | `/status` | Firmware version, sensor health |

### Pose JSON format

```json
{
  "timestamp_ms": 12453,
  "wrist": { "w": 0.98, "x": 0.01, "y": 0.12, "z": 0.02 },
  "fingers": [
    {
      "id": 0,
      "name": "thumb",
      "mcp": { "w": 0.97, "x": 0.0, "y": 0.15, "z": 0.0 },
      "pip_deg": 34.2,
      "dip_deg": 18.7
    },
    {
      "id": 1,
      "name": "index",
      "mcp": { "w": 0.99, "x": 0.0, "y": 0.08, "z": 0.0 },
      "pip_deg": 45.1,
      "dip_deg": 22.3
    }
  ]
}
```

### Example client call

```python
import requests

pose = requests.get("http://sapienhand.local/pose").json()
wrist_q = pose["wrist"]   # quaternion dict
pip_angle = pose["fingers"][1]["pip_deg"]
```

The ESP32-S3 uses **mDNS** to advertise itself as `sapienhand.local` on the local network, removing the need to hardcode an IP address.
