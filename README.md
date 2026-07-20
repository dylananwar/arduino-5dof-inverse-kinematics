# 5-DOF Robotic Arm: Kinematics & Control Engine

A coordinate-driven control engine built for a 5-Degree-of-Freedom (5-DOF) robotic arm. Moving beyond basic servo angle commands (`servo.write(deg)`), this project uses real-time Inverse Kinematics (IK), spatial vector decomposition, and hardware-level signal handling to map 3D Cartesian target coordinates $(X, Y, Z)$ to physical joint movements.

---

## Technical Features

* **Real-Time Inverse Kinematics Solver:** Calculates joint angles on the fly using planar trigonometric decomposition and the Law of Cosines.
* **Workspace Envelope Safeguarding:** Employs dynamic vector clamping ($d_{\text{max}} = 19.5\text{ cm}$) to scale out-of-bounds targets proportionally and prevent mathematical domain errors ($\arccos$ overflow).
* **Analog Control Mapping:** Translates multi-channel potentiometer inputs into calibrated physical workspace parameters $(X, Y, Z)$.
* **Hardware-Aware Initialization:** Features staggered servo attachments to prevent current draw spikes, paired with linear coordinate interpolation for safe homing sequences.
* **Serial Telemetry Pipeline:** Outputs realtime target vectors and system diagnostics to the terminal interface.

---

## Kinematics & Mathematical Overview

The core program translates target vectors $(X, Y, Z)$ into required servo angles ($\theta_1, \theta_2, \theta_3$) by breaking down the 3D space into 2D geometric components.
