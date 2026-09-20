# NERC Autonomous Line-Following & Target-Shooting Robot

![C++](https://img.shields.io/badge/Language-C%2B%2B-00599C?style=for-the-badge&logo=cplusplus)
![Platform](https://img.shields.io/badge/Platform-Arduino%20Mega-00979D?style=for-the-badge&logo=arduino)
![Course](https://img.shields.io/badge/Course-Mechatronics%20System%20Design%20(DE--45)-blue?style=for-the-badge)

An autonomous differential-drive robot built for line following, IR junction navigation, ratio-based color detection, and 2-DOF target tracking and firing. Designed and implemented in **C++** for the Arduino Mega platform.

---

## 📌 Project Overview

This project integrates differential drive kinematics, reflectance line-tracking, active color thresholding, and an active end-effector launcher into a unified state-machine navigation pipeline.

- **Line Tracking:** 8-channel QTR reflectance sensor array utilizing Proportional-Derivative (PD) feedback control.
- **Junction & Navigation:** Side IR sensor junction counting and locking logic combined with dynamic state-machine routing.
- **Color Perception:** Ratio-based TCS3200 color sensing to identify target faces (S1) and target squares (S2–S5) resilient to ambient light fluctuations.
- **Aiming & Firing:** 2-DOF servo pan/tilt targeting system driven by a active-LOW relay actuator pulse.

---

## 🛠️ Hardware & Components

| Component | Description / Specification |
| :--- | :--- |
| **Microcontroller** | Arduino Mega 2560 |
| **Motors** | 2x N20 DC Gear Motors (12V, 150 RPM) |
| **Motor Drivers** | IBT-2 H-Bridge Driver Channels |
| **Reflectance Sensors** | QTR 8-Channel Analog Sensor Array |
| **Junction Sensors** | 2x IR Sensors (Left & Right) |
| **Color Sensors** | TCS3200 Color Sensors (S1 Face + S2–S5 Targets) |
| **Aiming Mechanism** | 2x Servo Motors (Vertical & Horizontal Aiming) |
| **Firing Actuator** | Solenoid / Launching Mechanism via Active-LOW Relay |

---

## 📂 Code Structure

The source code is written in **C++** and structured into functional modular routines inside [`src/main.cpp`](src/main.cpp):

1. **Drive & Kinematics Control:** Low-level PWM driver logic with motor direction correction.
2. **PD Control System:** Error calculation and feedback loop for high-speed line tracking.
3. **Junction Logic:** Debounced junction locking mechanism using timed delay thresholds.
4. **Color Recognition:** RGB pulse frequency normalization and ratio thresholding.
5. **Target Scanning & Aiming:** Multiplexed sensor output enabling (OE) and servo angle mapping.
6. **State Machine Navigation:** Finite State Machine (FSM) determining route decisions per junction count.

---

## 🚀 Getting Started

### Prerequisites
- **Arduino IDE** or **PlatformIO**
- Installed C++ Libraries:
  - `QTRSensors`
  - `Servo`

### Flashing the Board
1. Clone this repository:
   ```bash
   git clone [https://github.com/your-username/NERC-Autonomous-Robot.git](https://github.com/your-username/NERC-Autonomous-Robot.git)
