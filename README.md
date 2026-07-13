# 5DOF Robotic Arm: Coordinate-Driven Inverse Kinematics Control

An admissions-grade engineering project focused on transforming an Adeept 5DOF robotic arm kit from a joint-angle toy into a coordinate-driven autonomous machine. This system calculates real-time trigonometric trajectories to map 3D spatial points $(X, Y, Z)$ directly into precise servo positions using geometric decomposition and the Law of Cosines.

---

## 🚀 Project Status
* **Phase 1 (Simulation):** COMPLETE. Mathematical engine, coordinate constraints, and virtual joystick logic fully validated via Tinkercad Circuits.
* **Phase 2 (Hardware Integration):** IN PROGRESS. Hardware target baseline scheduled for completion by September 1.

---

## 🛠️ System Architecture

### 1. Hardware Stack
* **Chassis:** Adeept 5DOF Robotic Arm structural linkages
* **Microcontroller:** Arduino Uno (ATmega328P)
* **Actuators:** 5 positional micro servos
* **Power Delivery:** Dual-rail isolation (USB for Arduino logic, external independent DC power supply for servo current requirements with shared common ground)

### 2. Software Architecture
The program operates on a state-machine framework built entirely in C++ (Arduino IDE):
* **IK Engine:** Converts input targets into trigonometric configurations.
* **Trajectory Planner:** Interpolates paths linearly to prevent mechanical stress from instant servo snapping.
* **Boundary Guardrails:** Software filters that reject commands exceeding physical link lengths or servo mechanical limits.

---

## 📐 Kinematic Math Overview

The arm splits 3D targets into two distinct geometric planes:
1. **Horizontal ($X-Y$) Plane:** Calculates base angle ($\theta_1$) via $\operatorname{atan2}(Y, X)$.
2. **Vertical ($Z-r$) Plane:** Solves a localized triangle formed by the upper arm ($L_1$) and forearm ($L_2$) relative to the target radius $r = \sqrt{X^2 + Y^2}$.

### Core Equations Validated in Simulation:
* **Distance to Target ($D$):** $$D = \sqrt{r^2 + Z^2}$$
* **Elbow Angle Interior Angle ($\beta$):** $$\cos(\beta) = \frac{L_1^2 + L_2^2 - D^2}{2 \cdot L_1 \cdot L_2}$$

---

## 📁 Repository Structure

```text
├── src/
│   └── inverse_kinematics_core.ino  # Main production Arduino codebase
├── schematics/
│   └── virtual_wiring_layout.png    # Tinkercad circuit wiring diagram
└── README.md                        # Project documentation
