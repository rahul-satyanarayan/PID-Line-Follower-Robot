# High-Speed Closed-Loop PID Line-Follower Robot

An autonomous, high-precision differential robotics platform built with the **Arduino Uno** and **Adafruit L293D Motor Shield v1**. The system features real-time closed-loop **PID (Proportional-Derivative) trajectory tracking** using a 3-channel digital infrared sensor array. It utilizes full-bridge bi-directional motor mixing to dynamically transition into reverse spin during sharp corners and automatically initiates a rotational sweep if the line is lost.

---

## Key Highlights & Features

- **Discrete Linear Error Quantification:** Maps 3 digital IR sensors across a linear error scale $[-2, -1, 0, 1, 2]$ to measure lateral track offset with zero computational lag.
- **Proportional-Derivative (PD) Steering Engine:**
  - **$K_p = 4.9$:** High-gain proportional response for rapid corrective steering along turns.
  - **$K_d = 15.0$:** Derivative damping to suppress overshoot and eliminate high-speed oscillation.
- **Full-Bridge Dynamic Directional Mixing:** Rather than simple speed throttling, the algorithm pushes motor speed into negative values (`BACKWARD`), allowing instantaneous in-place counter-rotations on sharp bends.
- **Active Line-Loss Recovery:** When the track drops out of the sensor array (`0-0-0`), the rover queries `lastError` and spins toward the last known trajectory vector until contact is re-established.
- **Hardware Asymmetry Calibration:** Independent baseline wheel trims (`BASE_L = 135`, `BASE_R = 115`) compensate for gearbox and tire manufacturing tolerances.

---

## Control Architecture & Math

```
[3-Channel IR Array (A0, A1, A2)]
              │
              ▼ (Digital Acquisition: L, C, R)
   [Error Quantizer: e(t) ∈ {-2, -1, 0, 1, 2}]
              │
              ▼
   [PD Controller: u(t) = Kp·e(t) + Kd·Δe/Δt]
              │
              ▼
 [Full-Bridge Actuator Mixer]
   ├── pwrL = BASE_L - u(t) ──► Left Motor (M3 Channel)
   └── pwrR = BASE_R + u(t) ──► Right Motor (M2 Channel)
```

### Governing Equations
$$\text{derivative} = \text{error} - \text{lastError}$$
$$\text{correction} = (K_p \cdot \text{error}) + (K_d \cdot \text{derivative})$$
$$P_{\text{Left}} = \text{BASE\_L} - \text{correction}$$
$$P_{\text{Right}} = \text{BASE\_R} + \text{correction}$$

---

## System Schematic & Wiring Diagram

<p align="center">
  <img src="docs/PID%20Line-Follower%20Robot%20Schematic%20Connection%20Diagram.png" alt="PID Line-Follower Robot Schematic Connection Diagram" width="100%">
</p>

> **Vector Document:** The high-resolution vector PDF is archived under [`docs/PID Line-Follower Robot Schematic Connection Diagram.pdf`](docs/PID%20Line-Follower%20Robot%20Schematic%20Connection%20Diagram.pdf).

---

## Hardware Pinout Matrix

| Subsystem / Peripheral | Sensor Lead | Shield / Uno Pin | Signal Type | Electrical Specification | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **3-Channel IR Array** | `Sensor_L` | **Analog A2** | Digital Input | 5V Logic | Left tracking photodiode |
| | `Sensor_C` | **Analog A1** | Digital Input | 5V Logic | Center tracking photodiode |
| | `Sensor_R` | **Analog A0** | Digital Input | 5V Logic | Right tracking photodiode |
| | `VCC` | `5V` | Power Rail | 5V Regulated | Sensor logic supply |
| | `GND` | `GND` | Ground Reference | 0V | Common system ground |
| **Left Drive Motor (BO)** | Terminal Leads | **Shield M3** | PWM / H-Bridge | Up to 12VDC | Left motor drive channel |
| **Right Drive Motor (BO)**| Terminal Leads | **Shield M2** | PWM / H-Bridge | Up to 12VDC | Right motor drive channel |
| **Power Management** | Battery (+/-) | **Shield EXT_PWR** | Power In | 7.4V - 11.1V | High-current isolated motor rail |

---

## State Transition & Error Truth Table

| Left (`A2`) | Center (`A1`) | Right (`A0`) | Error State | Robot Status | Corrective Action |
| :---: | :---: | :---: | :---: | :--- | :--- |
| `0` | `1` | `0` | $0$ | Centered on Line | Balanced forward travel |
| `1` | `1` | `0` | $+1$ | Minor Left Drift | Soft right differential bias |
| `1` | `0` | `0` | $+2$ | Hard Left Drift | Aggressive right pivot (Left reverses if needed) |
| `0` | `1` | `1` | $-1$ | Minor Right Drift | Soft left differential bias |
| `0` | `0` | `1` | $-2$ | Hard Right Drift | Aggressive left pivot (Right reverses if needed) |
| `0` | `0` | `0` | *Lost* | Total Line Dropout | Continuous pivot toward `lastError` |

---

## Software & Dependencies

### External Libraries (`libraries.txt`)
* `Adafruit Motor Shield V1` (`>= 1.0.1`) — Motor driver abstraction for the L293D shield.

Install via the Arduino Library Manager:
```bash
# Search and install via Arduino IDE Library Manager:
Adafruit Motor Shield library
```

---

## Project Structure

```text
PID-Line-Follower-Robot/
├── .gitignore
├── LICENSE
├── libraries.txt
├── README.md
├── src/
│   └── Line_Follower_AFM.ino
└── docs/
    ├── PID Line-Follower Robot Schematic Connection Diagram.pdf
    └── PID Line-Follower Robot Schematic Connection Diagram.png
```

---

## Getting Started & Tuning

### 1. Hardware Assembly
1. Mount the Adafruit Motor Shield firmly onto the Arduino Uno headers.
2. Remove the **PWR Jumper** on the shield to decouple logic power from the motor power supply.
3. Wire your DC drive motors into screw terminals **M3** (Left) and **M2** (Right).
4. Secure the 3-channel IR array at the front of the chassis, ideally positioned **5 mm – 10 mm** above the driving surface.
5. Connect `Sensor_L`, `Sensor_C`, and `Sensor_R` to pins **A2**, **A1**, and **A0** respectively.
6. Connect an external 2S/3S Li-ion battery pack to the shield's **EXT_PWR** terminal.

### 2. Flashing the Firmware
1. Open `src/Line_Follower_AFM.ino` in the Arduino IDE.
2. Select your board: **Tools > Board > Arduino Uno**.
3. Select your serial port and click **Upload**.

### 3. Tuning Procedure
* **Zero Baseline Trimming:** Lift the rover off the ground, set $K_p = 0$ and $K_d = 0$, and verify both motors turn evenly. Adjust `BASE_L` and `BASE_R` to balance wheel velocities.
* **Proportional Gain ($K_p$):** Gradually increase $K_p$ until the robot hugs curves tightly. If high-frequency shaking appears, lower $K_p$.
* **Derivative Gain ($K_d$):** Increase $K_d$ to dampen steering overshoots and smooth transitions between straightaways and sharp corners.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.