# Automatic Electromechanical Battery Thermal Management System (BTMS) for EVs

A MATLAB-based project that designs and simulates an automatic electromechanical control system to regulate multiple parameters - battery temperature, cooling-fan speed, and coolant-pump speed - in an Electric Vehicle (EV) battery pack using closed-loop PID control.

---

## Table of Contents
- [Overview](#overview)
- [System Architecture & Block Diagram](#system-architecture--block-diagram)
- [How It Works](#how-it-works)
- [Simulation & Results](#simulation--results)
  - [Overall System Response](#1-overall-system-response)
  - [PID Tuning & Comparison](#2-pid-tuning--comparison)
  - [Performance Summaries](#3-performance-summaries)
- [Implementation & Code Snippet](#implementation--code-snippet)
- [Repository Structure](#repository-structure)
- [How to Run](#how-to-run)
- [Author](#author)

---

## Overview
Maintaining an optimal operating temperature in EV lithium-ion battery packs is critical for safety, efficiency, and battery longevity. This project implements a closed-loop PID thermal management system modeled in MATLAB. The system dynamically adjusts fan and coolant pump speeds based on temperature feedback and driving-cycle heat generation.

---

## System Architecture & Block Diagram

![Block Diagram](docs/block_diagram.png)

---

## How It Works

* **Setpoint:** 32 degrees C target battery temperature
* **Feedback:** Battery temperature sensor (T_batt)
* **Controller:** Closed-loop PID controller
* **Actuators:** Cooling fan motor and coolant pump motor (modeled as 1st-order DC motors)
* **Plant Model:** Battery thermal model:
  dT/dt = (Q_generated - Q_removed) / (m * cp)
* **Disturbance:** Simulated EV driving-cycle load current (varying heat generation)

---

## Simulation & Results

### 1. Overall System Response
The primary simulation demonstrates dynamic thermal regulation under variable load current representative of real-world driving conditions.

![Simulation Result](results/simulation_result.png)

> **Key Observation:** The battery temperature is kept close to the 32 degrees C setpoint even while the load current (simulating real EV driving) keeps changing.

---

### 2. PID Tuning & Comparison
Comparison of controller responses under different PID gain settings to evaluate overshoot, settling time, and tracking performance.

![PID Tuning Comparison](results/pid_tuning_comparison.png)

---

### 3. Performance Summaries
Detailed graphical summaries evaluating transient performance, error metrics, and system efficiency.

| Main System Performance | PID Controller Performance |
| :---: | :---: |
| ![Performance Summary Main](results/performance_summary_main.png) | ![Performance Summary PID](results/performance_summary_pid.png) |

---

## Implementation & Code Snippet

Key control loops and differential equation modeling implemented in MATLAB:

![Code Snippet](docs/code_snippet.png)

---

## Repository Structure

```text
├── matlab/
│   └── main_simulation.m            # Main script to run simulation & plot results
├── docs/
│   ├── block_diagram.png            # System block diagram
│   └── code_snippet.png             # Key control algorithm preview
├── results/
│   ├── simulation_result.png        # Temperature & actuator transient response
│   ├── pid_tuning_comparison.png    # PID tuning step responses
│   ├── performance_summary_main.png # Main system evaluation metrics
│   └── performance_summary_pid.png  # Detailed PID metrics summary
└── README.md                        # Project documentation
