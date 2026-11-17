# Finite-Control-Set Model Predictive Control (FCS-MPC) with Multi-Objective Optimization for PMSM Power Electronics Inverter Drives

### Advanced Power Electronics & Embedded Motor Drive Suite | C | Python SIL | MATLAB/Simulink | ESP32 Real-Time Firmware
**Department of Electrical Engineering, Veermata Jijabai Technological Institute (VJTI), Mumbai**  
**Author:** Arpan Kailash Gupta  
**Roll Number:** 241030009  
**Academic Year:** Third Year B.Tech (Electrical Engineering)  

---

## Executive Summary

This project presents the design, mathematical formulation, embedded C implementation, and multi-tier simulation validation of a **Finite-Control-Set Model Predictive Controller (FCS-MPC)** with multi-objective optimization for a 3-phase **2-Level Voltage Source Inverter (2L-VSI)** driving a Permanent Magnet Synchronous Motor (PMSM).

Unlike classical Field-Oriented Control (FOC)—which relies on linear cascaded Proportional-Integral (PI) regulators, anti-windup clamping, and Space Vector Pulse Width Modulators (SVPWM)—**FCS-MPC directly leverages the discrete physical switching nature of the power inverter**. 

Every sampling period ($T_s = 50\,\mu\text{s}$, $f_s = 20\,\text{kHz}$), the controller evaluates a discrete-time predictive model of the motor across all **8 candidate switching voltage vectors** ($V_0 \dots V_7$) and solves an online multi-objective cost optimization that simultaneously minimizes:
1. **Torque and Flux Current Tracking Errors** ($i_q^*$ and $i_d^*$).
2. **Inverter Switching Frequency and Commutation Losses** (IGBT/MOSFET thermal dissipation).
3. **Common-Mode Voltage (CMV) Stresses** (protecting motor bearings against capacitive discharge pitting).
4. **Hard Instantaneous Overcurrent Constraints** (preventing inverter bridge shoot-through and magnetic demagnetization).

To eliminate digital calculation delay inherent in real-time microcontrollers, the algorithm incorporates a **2-step-ahead ($k+2$) state prediction pipeline**, ensuring zero-phase lag and closed-loop stability across the entire speed envelope.

---

## Architectural Comparison: Classical FOC vs. FCS-MPC

| Control Attribute | Classical Field-Oriented Control (FOC) | Proposed FCS-MPC Power Electronics Drive |
| :--- | :--- | :--- |
| **Control Philosophy** | Cascaded linear control with continuous modulation | Discrete-time state prediction & online cost minimization |
| **Current Regulators** | Dual PI controllers ($i_d, i_q$) with cross-coupling decoupling | **None** (direct state-space mathematical prediction) |
| **Modulation Stage** | Space Vector PWM (SVPWM) / Sine PWM (SPWM) | **Modulator-Free** (direct transistor gate pulse gating) |
| **Switching Frequency** | Fixed carrier frequency ($f_{sw} = f_c$) | Variable, dynamically constrained via penalty weight $\lambda_{sw}$ |
| **Dynamic Response Time** | Limited by PI bandwidth and modulator delay ($\sim 3 \text{ to } 8\,\text{ms}$) | **Sub-millisecond** ($\sim 0.15 \text{ to } 0.35\,\text{ms}$, 1-2 control steps) |
| **Multi-Objective Support**| Requires separate hardware filtering or complex multi-loop architectures | **Native** (loss reduction, CMV mitigation, current limits combined in cost $J$) |
| **Calculation Delay** | Degrades phase margin; requires lead compensator | **Exact 2-Step Ahead ($k+2$) Algebraic Compensation** |

---

## 1. System Mathematical Modeling

```
               +-------------------------------------------------------------+
               |               20 kHz FCS-MPC OPTIMIZATION PIPELINE          |
               |                                                             |
   Speed Ref   |  +------------+   iq*  +---------------------------------+  |
  omega_m* --->|->| Speed PI   |------->|                                 |  |
               |  | Controller |        |   2-Step Ahead State Predictor  |  |
               |  +------------+   id*=0|   i_d(k+2), i_q(k+2)            |  |
               |                        +---------------------------------+  |
               |                                        |                    |
               |                                        v                    |
               |                        +---------------------------------+  |
   i_a,b,c --->|-> Clarke / Park ------->|    Multi-Objective Online       |  |
   theta_e --->|-> Transformations       |    Cost Function Optimizer      |  |
   V_dc    --->|                         |    min J(V_0 ... V_7)           |  |
               |                         +---------------------------------+  |
               |                                        |                    |
               |                                        v Optimal Vector     |
               |                              S_opt = (Sa, Sb, Sc)           |
               +----------------------------------------|--------------------+
                                                        |
                                                        v Direct Gate Triggers (No PWM)
                                        +--------------------------------+
                                        |  2-Level 3-Phase Inverter      |
                                        |  (6 Power MOSFETs / IGBTs)     |
                                        |  V_dc = 24V / 48V              |
                                        +--------------------------------+
                                                        |
                                                        v 3-Phase Voltages (va, vb, vc)
                                        +--------------------------------+
                                        |  Permanent Magnet Synchronous  |
                                        |  Motor (PMSM) Plant            |
                                        +--------------------------------+
```

### 1.1 PMSM Discrete-Time State-Space Equations
The electrical dynamics of a surface-mounted PMSM ($L_d = L_q = L_s$) in the synchronous rotor reference frame ($d-q$) are governed by:

$$\frac{di_d}{dt} = -\frac{R_s}{L_d} i_d + \omega_e i_q + \frac{1}{L_d} v_d$$

$$\frac{di_q}{dt} = -\frac{R_s}{L_q} i_q - \omega_e i_d - \frac{\psi_f}{L_q} \omega_e + \frac{1}{L_q} v_q$$

Applying forward-Euler discretization with sampling period $T_s$:

$$i_d(k+1) = \left(1 - \frac{R_s T_s}{L_d}\right) i_d(k) + \frac{L_q T_s}{L_d} \omega_e(k) i_q(k) + \frac{T_s}{L_d} v_d(k)$$

$$i_q(k+1) = \left(1 - \frac{R_s T_s}{L_q}\right) i_q(k) - \frac{L_d T_s}{L_q} \omega_e(k) i_d(k) - \frac{\psi_f T_s}{L_q} \omega_e(k) + \frac{T_s}{L_q} v_q(k)$$

### 1.2 Two-Step Ahead Delay Compensation ($k+2$)
In microcontrollers (DSP, ESP32, STM32), the control execution during interval $[k, k+1]$ introduces a 1-step computation delay. If the optimal vector is applied immediately without compensation, the loop suffers from limit cycles and high current ripple.

1. **Step 1 ($k \to k+1$):** Predict the state at step $k+1$ using the previously latched optimal switching vector $S_{opt}(k)$:
   $$\theta_e(k+1) = \theta_e(k) + \omega_e(k) T_s$$
   $$\hat{\mathbf{i}}_{dq}(k+1) = \mathbf{A} \mathbf{i}_{dq}(k) + \mathbf{B} \mathbf{v}_{dq}(S_{opt}(k)) + \mathbf{E} \omega_e(k)$$

2. **Step 2 ($k+1 \to k+2$):** Evaluate each of the 8 candidate switching vectors $S_j \in \{V_0, V_1, \dots, V_7\}$ starting from $\hat{\mathbf{i}}_{dq}(k+1)$:
   $$\theta_e(k+2) = \theta_e(k+1) + \omega_e(k) T_s$$
   $$\hat{\mathbf{i}}_{dq,j}(k+2) = \mathbf{A} \hat{\mathbf{i}}_{dq}(k+1) + \mathbf{B} \mathbf{v}_{dq,j}(S_j) + \mathbf{E} \omega_e(k)$$

---

## 2. Inverter Topologies and 8 Discrete Switching Vectors

A 3-phase 2L-VSI consists of 3 half-bridge legs with 6 power semiconductor switches. The switching state is represented as $S_j = (S_a, S_b, S_c) \in \{0, 1\}^3$:

$$\mathbf{v}_{\alpha\beta}^{(j)} = \frac{2}{3} V_{dc} \left( S_a + S_b e^{j \frac{2\pi}{3}} + S_c e^{j \frac{4\pi}{3}} \right)$$

| Vector | Switching State $(S_a, S_b, S_c)$ | $v_\alpha$ (V) | $v_\beta$ (V) | Phase Angle | Type | Common-Mode Voltage $V_{cm}$ |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **$V_0$** | $(0, 0, 0)$ | $0$ | $0$ | — | Zero Vector | $-V_{dc} / 2$ |
| **$V_1$** | $(1, 0, 0)$ | $\frac{2}{3} V_{dc}$ | $0$ | $0^\circ$ | Active Vector | $-V_{dc} / 6$ |
| **$V_2$** | $(1, 1, 0)$ | $\frac{1}{3} V_{dc}$ | $\frac{1}{\sqrt{3}} V_{dc}$ | $60^\circ$ | Active Vector | $+V_{dc} / 6$ |
| **$V_3$** | $(0, 1, 0)$ | $-\frac{1}{3} V_{dc}$ | $\frac{1}{\sqrt{3}} V_{dc}$ | $120^\circ$ | Active Vector | $-V_{dc} / 6$ |
| **$V_4$** | $(0, 1, 1)$ | $-\frac{2}{3} V_{dc}$ | $0$ | $180^\circ$ | Active Vector | $+V_{dc} / 6$ |
| **$V_5$** | $(0, 0, 1)$ | $-\frac{1}{3} V_{dc}$ | $-\frac{1}{\sqrt{3}} V_{dc}$ | $240^\circ$ | Active Vector | $-V_{dc} / 6$ |
| **$V_6$** | $(1, 0, 1)$ | $\frac{1}{3} V_{dc}$ | $-\frac{1}{\sqrt{3}} V_{dc}$ | $300^\circ$ | Active Vector | $+V_{dc} / 6$ |
| **$V_7$** | $(1, 1, 1)$ | $0$ | $0$ | — | Zero Vector | $+V_{dc} / 2$ |

---

## 3. Multi-Objective Cost Function Formulation

The optimization cost function evaluates all 8 vectors to select the optimal state $S_{opt} = \arg\min_{j \in \{0..7\}} J_j$:

$$J_j = J_{tracking} + J_{sw} + J_{cmv} + J_{barrier}$$

$$J_j = \lambda_d (i_d^* - \hat{i}_{d,j}(k+2))^2 + \lambda_q (i_q^* - \hat{i}_{q,j}(k+2))^2 + \lambda_{sw} \sum_{p \in \{a,b,c\}} |S_{p,j}(k+1) - S_{p,opt}(k)| + \lambda_{cmv} \left(\frac{|V_{cm,j}|}{V_{dc}}\right)^2 + f_{lim}(\hat{i}_j)$$

### Explanation of Optimization Terms:
1. **Current Tracking Quality ($\lambda_d, \lambda_q$):** Regulates $i_d \to 0$ (Maximum Torque Per Ampere for non-salient PMSM) and $i_q \to i_q^*$ (torque demand from outer speed loop).
2. **Switching Frequency Penalty ($\lambda_{sw}$):** Quantifies the Hamming distance between candidate vector $S_j$ and the currently active vector $S_{opt}(k)$. Penalizing transitions directly reduces switching commutations by up to 45%, slashing semiconductor thermal dissipation.
3. **Common-Mode Voltage Mitigation ($\lambda_{cmv}$):** Zero vectors ($V_0, V_7$) generate extreme common-mode voltages ($V_{cm} = \pm V_{dc}/2$). Penalizing these states forces the optimizer toward active vectors that keep $V_{cm} = \pm V_{dc}/6$, preventing bearing EDM currents.
4. **Hard Overcurrent Barrier ($f_{lim}$):** If $|\hat{\mathbf{i}}_{dq,j}| > I_{max}$, $J_j \to \infty$. This guarantees hardware protection against saturation and inverter leg overcurrent.

---

## 4. Power Electronics Inverter Loss & Thermal Modeling

### 4.1 Conduction Loss ($P_{cond}$)
Conduction loss across the 6 power MOSFETs with on-state resistance $R_{DS(on)} = 12\,\text{m}\Omega$:

$$P_{cond} = (i_a^2 + i_b^2 + i_c^2) \cdot R_{DS(on)}$$

### 4.2 Dynamic Switching Loss ($P_{sw}$)
Energy dissipated during MOSFET turn-on ($t_r = 25\,\text{ns}$) and turn-off ($t_f = 20\,\text{ns}$) across commutations:

$$E_{sw,x} = \frac{1}{2} V_{dc} |i_x| (t_r + t_f)$$

Instantaneous switching power over sample period $T_s$:

$$P_{sw} = \frac{\sum_{x \in \{a,b,c\}} E_{sw,x}}{T_s}$$

### 4.3 Inverter Efficiency
$$\eta = \frac{P_{out}}{P_{out} + P_{cond} + P_{sw}} \times 100\% \quad (\approx 96.5\% - 97.4\%)$$

---

## 5. Repository Structure

```
PMSM-FaultTolerant-Control/
├── Makefile                          # Build automation (C shared lib, unit tests, SIL)
├── platformio.ini                    # PlatformIO configuration for ESP32 target
├── requirements.txt                  # Python scientific environment dependencies
├── src/                              # Pure C & C++ Embedded Firmware
│   ├── motor_params.h                # Physical motor, inverter, and MPC weight parameters
│   ├── inverter_2lvsi.h              # 2L-VSI 8-vector definitions & loss telemetry headers
│   ├── inverter_2lvsi.c              # Inverter physics, commutation & loss implementation
│   ├── mpc_core.h                    # FCS-MPC 2-step prediction & cost function definitions
│   ├── mpc_core.c                    # FCS-MPC optimization core implementation
│   ├── speed_controller.h            # Outer speed PI loop header
│   ├── speed_controller.c            # Anti-windup speed regulator implementation
│   ├── mpc_api.h                     # Host C ABI export interface for SIL / LabVIEW
│   ├── mpc_api.c                     # System coordinator implementing mpc_api
│   ├── main.cpp                      # ESP32 20 kHz hardware timer ISR firmware
│   └── tests/
│       └── test_mpc.c                # C unit test suite (transforms, vectors, optimization)
├── simulation/                       # Multi-Tier Simulation Suite
│   ├── plant/
│   │   ├── pmsm_model.py             # 4th-order non-linear PMSM plant (RK4 solver)
│   │   └── inverter_2lvsi.py         # 2L-VSI Python power electronics model
│   ├── harness/
│   │   ├── config.py                 # Simulation timing and configuration dictionary
│   │   ├── c_bridge.py               # Ctypes bridge to libmpc_controller.dylib
│   │   └── mpc_runner.py             # Benchmark orchestrator & waveform generator
│   └── analysis/
│       ├── metrics.py                # Performance metric calculations (THD, settling time)
│       └── plotter.py                # High-DPI publication waveform plotter
├── simulink/                         # MATLAB / Simulink Co-Simulation
│   ├── sfun_mpc_controller.c         # Level-2 C-MEX S-Function for Simulink
│   └── pmsm_foc_simulink.slx         # Simulink graphical model
└── docs/
    └── figures/                      # Generated 300-DPI publication waveforms
        ├── mpc_dynamic_speed_torque_response.png
        ├── mpc_switching_frequency_pareto.png
        └── mpc_dc_link_sag_rejection.png
```

---

## 6. Building and Running

### 6.1 Prerequisites
- GCC / Clang compiler with C99 support.
- Python 3.10+ with `numpy`, `scipy`, `matplotlib`.
- PlatformIO Core CLI (for flashing ESP32 hardware).

### 6.2 Compile Embedded C Shared Library & Run Unit Tests
```bash
make clean
make all
```
*Output:*
```
gcc -Wall -Wextra -O3 -fPIC -shared src/inverter_2lvsi.c src/mpc_api.c src/mpc_core.c src/speed_controller.c -lm -o build/libmpc_controller.dylib
gcc -Wall -Wextra -O3 -fPIC src/tests/test_mpc.c src/inverter_2lvsi.c src/mpc_api.c src/mpc_core.c src/speed_controller.c -lm -o build/test_mpc
./build/test_mpc

==================================================================
FCS-MPC PMSM Drive C Unit Test Suite
Author: Arpan Kailash Gupta (241030009), VJTI Mumbai
==================================================================
[TEST] Checking Clarke & Park transformations... PASS
[TEST] Checking Inverter 8 switching vectors & voltages... PASS
[TEST] Checking FCS-MPC optimization loop & selection... PASS (Selected V3, Cost = 6.222)
[TEST] Checking outer speed PI controller... PASS (Iq_cmd = 8.00 A)

All C unit tests completed successfully (100% PASS).
```

### 6.3 Run Power Electronics Simulation Benchmarks
```bash
make run
```
*Output:*
```
Executing FCS-MPC PMSM Drive Benchmark Simulation Suite
Department of Electrical Engineering, VJTI Mumbai
Author: Arpan Kailash Gupta (Roll No: 241030009)

[SCENARIO 1] Dynamic Step Speed & Torque Step Response...
[SUCCESS] Saved Scenario 1 plot to docs/figures/mpc_dynamic_speed_torque_response.png

[SCENARIO 2] Multi-Objective Switching Penalty Sweep (Pareto Analysis)...
  -> Simulating with lambda_sw = 0.000...
  -> Simulating with lambda_sw = 0.020...
  -> Simulating with lambda_sw = 0.045...
  -> Simulating with lambda_sw = 0.080...
  -> Simulating with lambda_sw = 0.120...
[SUCCESS] Saved Scenario 2 Pareto plot to docs/figures/mpc_switching_frequency_pareto.png

[SCENARIO 3] DC-Link Supply Sag Disturbance Rejection...
[SUCCESS] Saved Scenario 3 plot to docs/figures/mpc_dc_link_sag_rejection.png
```

---

## 7. Quantitative Benchmark Results

### Benchmark 1: Dynamic Step Speed & Torque Step Response
- **Test Condition:** Speed demand step from 0 to 1500 RPM, followed by an abrupt full rated load step of $T_L = 0.40\,\text{N}\cdot\text{m}$ at $t = 0.40\,\text{s}$.
- **Performance:**
  - **Torque Rise Time:** **0.20 ms** (4 control cycles) without overshoot.
  - **Speed Recovery Time:** **18.5 ms** following load impact.
  - **Speed Dip:** $< 3.2\%$ ($48\,\text{RPM}$ under rated load impact).
  - **Steady-State Error:** $\pm 0.4\,\text{RPM}$ ($< 0.03\%$).

### Benchmark 2: Switching Commutation & Loss Pareto Trade-off
Evaluating the switching frequency penalty weight $\lambda_{sw}$:

| $\lambda_{sw}$ Weight | Average $f_{sw}$ (kHz) | Total Inverter Loss $P_{loss}$ (W) | Current THD (%) | Inverter Efficiency $\eta$ |
| :---: | :---: | :---: | :---: | :---: |
| **$0.000$ (No Penalty)** | $14.2\,\text{kHz}$ | $2.84\,\text{W}$ | $2.14\%$ | $95.8\%$ |
| **$0.020$** | $10.6\,\text{kHz}$ | $2.15\,\text{W}$ | $2.48\%$ | $96.8\%$ |
| **$0.045$ (Optimal)** | **$7.8\,\text{kHz}$** | **$1.62\,\text{W}$** | **$2.95\%$** | **$97.4\%$** |
| **$0.080$** | $5.9\,\text{kHz}$ | $1.29\,\text{W}$ | $3.82\%$ | $97.9\%$ |
| **$0.120$** | $4.4\,\text{kHz}$ | $1.05\,\text{W}$ | $5.12\%$ | $98.3\%$ |

*Conclusion:* Setting $\lambda_{sw} = 0.045$ cuts inverter switching losses by **43.0%** while preserving current THD comfortably below standard IEEE-519 $5\%$ limits.

### Benchmark 3: DC-Link Supply Sag Disturbance Rejection
- **Test Condition:** DC bus supply voltage suddenly drops from nominal $24.0\,\text{V}$ to $18.0\,\text{V}$ ($-25.0\%$ sag) at $t = 0.35\,\text{s}$.
- **Performance:**
  - Because FCS-MPC explicitly feeds the instantaneous measured $V_{dc}$ into its discrete voltage vector predictions:
  - **Rotor Speed Perturbation:** $< 0.8\,\text{RPM}$ (practically imperceptible).
  - **Current Response:** The optimizer automatically lengthens the application of active vectors without experiencing integral windup or controller saturation.

---

## 8. Embedded Real-Time Performance on ESP32

When compiled for the Espressif ESP32 Dev Module (Xtensa Dual-Core LX6 @ 240 MHz):
- **Flash Utilization:** $224\,\text{KB}$ ($17.1\%$ of $1.31\,\text{MB}$).
- **SRAM Utilization:** $18.4\,\text{KB}$ ($5.6\%$ of $327\,\text{KB}$).
- **Worst-Case Execution Time (WCET):** **$14.2\,\mu\text{s}$** per control cycle.
  - The available control period at $20\,\text{kHz}$ is $T_s = 50.0\,\mu\text{s}$.
  - CPU Utilization: **$28.4\%$**, leaving ample margin for serial communication, telemetry, and background safety diagnostics on Core 0.

---

## 9. Academic Credit & Departmental Information

* **Department:** Department of Electrical Engineering  
* **Institution:** Veermata Jijabai Technological Institute (VJTI), Matunga, Mumbai - 400019  
* **Student Author:** Arpan Kailash Gupta  
* **Roll Number:** 241030009  
* **Project Type:** Power Electronics & Motor Drives Advanced Simulation and Embedded Controller Design  
