# Model-Based Design & Hardware-in-the-Loop (HIL) Control of a Two-Wheeled Inverted Pendulum Robot

![Development Phase](https://img.shields.io/badge/Phase-HIL%20%2F%20SIL%20Verification-blue)
![Architecture](https://img.shields.io/badge/V--Model-Model--Based%20Design-success)
![Target MCU](https://img.shields.io/badge/Hardware-STM32F411CEU6%20(Black%20Pill)-orange)
![Toolchain](https://img.shields.io/badge/Toolchain-MATLAB%20%7C%20Simulink%20%7C%20Simscape%20Multibody-red)

An advanced mechatronics project focused on the control system engineering of a dynamically unstable two-wheeled inverted pendulum (self-balancing robot). The development follows a rigorous **Model-Based Design (MBD)** methodology, progressing from mathematical modeling and multi-body physics simulation (SIL) to real-time embedded execution on an ARM Cortex-M4 microcontroller (HIL).

---

## 🏗 System Architecture & Verification Strategy

The project implements a full V-Model workflow to guarantee stability, reject disturbances, and minimize control latency before physical assembly:

System Requirements & Dynamics Derivation
                     \                 /   Physical Prototype (In Progress)
       Multi-Body Kinematic Model     /   HIL Target Testing (STM32 Black Pill)
                   \                 /   /
                SIL Closed-Loop Simulation (Simulink)

1. **Plant Modeling:** Rigid multi-body dynamics modeled in **Simscape Multibody**, including joint compliance, inertial parameters, and contact friction.
2. **Software-in-the-Loop (SIL):** Continuous-time and discretized state feedback validation within Simulink.
3. **Hardware-in-the-Loop (HIL):** Execution of the synthesized discrete control algorithm and state estimators directly on the **STM32F411CEU6 (Black Pill)** in real time, interacting synchronously with the numerical plant model via USART/external mode.

---

## 🧮 Control Design & State Estimation

### Linear Quadratic Regulator (LQR / LQI)
The non-linear plant is linearized about the unstable vertical equilibrium ($\theta = 0$):

$$\dot{x} = Ax + Bu, \quad y = Cx$$

* **State Vector:**
  $$x = \begin{bmatrix} p & \dot{p} & \theta & \dot{\theta} \end{bmatrix}^T$$
  *(Linear position, linear velocity, pitch angle, angular pitch velocity)*
* **Optimal Cost Minimization:** Gain matrix $K$ derived via the continuous/discrete algebraic Riccati equation:
  $$J(u) = \int_0^\infty (x^T Q x + u^T R u) \, dt$$
* **Integral Action (LQI):** Augmented state space with an error integrator to eliminate steady-state position offset caused by mechanical asymmetries and unmodeled ground slopes.

### Sensor Fusion & State Observer
* **Kalman Filter (KF):** Fuses high-rate gyroscope integration with accelerometer gravity vectors to deliver drift-free tilt angle $\theta$ and angular velocity $\dot{\theta}$.
* **Encoder Differentiation & Filtering:** Low-pass filtered position tracking derived from high-resolution wheel encoders to prevent high-frequency actuator chatter.

---

## ⚙ Hardware Target Specifications (HIL Stage)

| Subsystem | Specification / Component | Role |
|---|---|---|
| **MCU** | STM32F411CEU6 (ARM Cortex-M4 @ 100 MHz, FPU) | Real-time control loop & estimator execution |
| **IMU (Target)** | MPU-6050 (6-DOF Accelerometer + Gyroscope) | High-speed posture acquisition (I2C Fast Mode) |
| **Actuation** | Brushless Gimbal Motors (iPower GM3506) | Low-cogging, direct-drive dynamic actuation |
| **Inverters** | DRV8313 Triple Half-Bridge Drivers | FOC / Sinusoidal voltage commutation |
| **Telemetry** | High-speed USART (DMA enabled) | Real-time data logging and parameter tuning |

---
