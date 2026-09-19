# Model-Based Design & Hardware-in-the-Loop (HIL) Control of a Two-Wheeled Inverted Pendulum Robot

![Development Phase](https://img.shields.io/badge/Phase-HIL%20%2F%20SIL%20Verification-blue)
![Architecture](https://img.shields.io/badge/V--Model-Model--Based%20Design-success)
![Target MCU](https://img.shields.io/badge/Hardware-STM32F411CEU6%20(Black%20Pill)-orange)
![Toolchain](https://img.shields.io/badge/Toolchain-MATLAB%20%7C%20Simulink%20%7C%20Simscape%20Multibody-red)

An advanced mechatronics project focused on the control system engineering of a dynamically unstable two-wheeled inverted pendulum (self-balancing robot). The development follows a rigorous **Model-Based Design (MBD)** methodology, progressing from mathematical modeling and multi-body physics simulation (SIL) to real-time embedded execution on an ARM Cortex-M4 microcontroller (HIL).

---

## 🏗 System Architecture & Verification Strategy

The project implements a full V-Model workflow to guarantee stability, reject disturbances, and minimize control latency before physical assembly:
