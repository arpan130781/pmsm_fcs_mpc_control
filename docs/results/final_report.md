# Verification and Audit Report: Sensorless FOC Fault-Tolerant Observer for PMSM

**Target Hardware:** Espressif ESP32 Dev Module (Xtensa LX6 dual-core @ 240 MHz)  
**Simulation Frameworks:** Python SIL (`simulation/`), MATLAB/Simulink R2020b (`simulink/`), PlatformIO Core CLI  
**Control Loop Rate:** 20 kHz current loop ($T_s = 50\,\mu\text{s}$), 1 kHz speed loop ($T_{spd} = 1.0\,\text{ms}$)  
**Motor Specifications:** 100 W SPM PMSM ($R_s = 0.5\,\Omega$, $L_s = 1.0\,\text{mH}$, $\psi_f = 0.015\,\text{Wb}$, $P = 4$ pole pairs, $V_{bus} = 24\,\text{V}$)

---

## 1. Executive Summary

This engineering audit investigated, improved, and formally verified the complete embedded firmware and multi-tier simulation architecture for the **Sensorless FOC Fault-Tolerant Observer for Motors**:
1. **13 / 13 Python SIL Scenarios Verified Passing (100%):** All configured fault injection scenarios pass against strict acceptance criteria without relaxing thresholds (including Scenario 4 angle limit at 15.79° vs. 65.0° threshold).
2. **Robustness Isolation Sweep (144 Cases):** Successfully completed across 4 electrical onset phases, multiple speeds (1500 and 2000 RPM), and fault types (stuck sensor, 0.5 A offset, 2.0 A offset). Every test group met the $\ge 90\%$ target, achieving a **97.9% overall acceptable outcome rate (141/144)**.
3. **ESP32 Real-Time Control & PlatformIO Firmware Build:** Corrected cycle counter calls in `src/main.cpp` to use the target-compatible `esp_cpu_get_ccount()`, moved controller static variables to file scope reset cleanly in `foc_reset()`, and verified a 100% clean build via PlatformIO CLI (RAM: 6.8% [22,316 B], Flash: 22.7% [297,785 B]).
4. **MATLAB / Simulink Co-Simulation:** Recompiled C-MEX S-functions (`sfun_foc_controller.mexmaci64` and `sfun_pmsm_plant.mexmaci64`) and executed the 3.0 s closed-loop simulation in MATLAB R2020b in 43.68 s. Validated 0.96 ms fault latency and 0.052 A RMS current reconstruction, with publication-grade waveforms exported to `simulink/results/`.
5. **Build Pipeline & Repository Hygiene:** Corrected `scripts/build.sh` path resolution from `controller/` to `src/`, verified `make lib`, `make test`, and `make test-all` within execution limits, and updated `.gitignore` to exclude transient MATLAB cache files (`slprj/`, `*.slxc`).

---

## 2. Root Cause Analysis & Core Technical Improvements

### 2.1 Scenario 12 vs. Scenario 5 Arbitration (Dual-Branch Confidence Metric)
- **Problem Statement:** In Scenario 5 (Channel C single offset + load step), the controller must isolate Channel C within 20 ms. In Scenario 12 (symmetric 3-phase offset of +2.0 A on phases A, B, and C), the controller must declare an ambiguous fault and transition into mode 4 (`LIMP`), rather than falsely latching a single channel.
- **Root Cause:** A single DC offset of magnitude $D$ yields $E = i_a + i_b + i_c = D$, and $|(i_x - i_{pred}) - E| \approx 0$ on the faulty channel while accumulating $\approx E_{acc}$ on healthy channels. However, during Scenario 12, each channel has error $D$, so $E = 3D$, producing $|err - E| \approx |D - 3D| = 2D = \frac{2}{3} E_{acc}$ across all channels, with separation $\approx 0$.
- **Implemented Fix:** Implemented a dual-branch confidence rule in `src/fault_monitor.c`:
  ```c
  float separation = second_err - min_err;
  bool confident = (fm->E_acc > 0.0001f) && (
      (separation > 0.35f * fm->E_acc && min_err < 0.65f * fm->E_acc) ||
      (separation > 0.15f * fm->E_acc && min_err < 0.30f * fm->E_acc)
  );
  ```
  Coupled with an ambiguous timer threshold of 30 ms:
  ```c
  if (fm->unresolved_fault_timer >= 0.030f) {
      fm->ambiguous_fault = true;
      fm->fault_isolation_timer = 0.0f;
  }
  ```
  This allows single-phase faults with strong separation ($>35\%$) to isolate within 1–6 ms, while symmetric faults (where separation is $<15\%$) safely time out into mode 4 (`LIMP`).

### 2.2 Controller State Isolation Across Multi-Run Sweeps
- **Root Cause:** Static variables `prev_v_alpha`, `prev_v_beta`, and `speed_est` were declared locally inside `foc_tick()`. Consequently, `foc_reset()` failed to reset them, leading to cross-simulation state pollution during multiprocessing worker pool sweeps.
- **Fix:** Promoted them to file-level statics in `src/foc_core.c` and explicitly cleared them in `foc_reset()`, guaranteeing clean, identical initial conditions on every run.

### 2.3 ESP32 Real-Time Control & Cycle Counter
- **Root Cause:** `src/main.cpp` invoked `esp_cpu_get_cycle_count()`, which is deprecated/unsupported in the ESP32 Arduino framework 2.0.x toolchain.
- **Fix:** Replaced with `esp_cpu_get_ccount()`, resolving the symbol error and maintaining exact cycle-accurate WCET profiling telemetry.

---

## 3. Verified Quantitative Results

### 3.1 Python SIL Scenarios (13/13 Pass)
Command: `bash scripts/run_all_scenarios.sh`

| Scenario Name | Fault Type | Injected Channel | Target Speed | Detection Latency | Mean Speed Error | Angle RMS | Max Angle Error | Mode | Status |
|---|---|---|---|---|---|---|---|---|---|
| `no_fault_baseline` | None | None | 2000 RPM | N/A | 0.93 RPM | 3.37° | 7.75° | 2 (CLOSED) | **PASS** |
| `scenario_1_rs_drift` | Rs Drift (+5%) | None | 2000 RPM | N/A | 0.81 RPM | 3.33° | 7.58° | 2 (CLOSED) | **PASS** |
| `scenario_2_offset_a_2_0` | Sensor Offset (+2.0 A) | A | 2000 RPM | 0.95 ms | 1.09 RPM | 3.31° | 6.95° | 3 (DEGRADED) | **PASS** |
| `scenario_3_stuck_b_2_0` | Sensor Stuck | B | 2000 RPM | 1.05 ms | 1.04 RPM | 3.34° | 7.15° | 3 (DEGRADED) | **PASS** |
| `scenario_4_stuck_a_limit` | Stuck at Limit (10.0 A) | A | 2000 RPM | 0.95 ms | 0.99 RPM | 3.46° | 15.79° | 3 (DEGRADED) | **PASS** |
| `scenario_5_simultaneous` | Offset + Load Step | C | 1800 RPM | 5.95 ms | 0.78 RPM | 2.97° | 4.48° | 3 (DEGRADED) | **PASS** |
| `scenario_6_noise_burst` | Gaussian Noise | A | 2000 RPM | Rec: 0.50 s | 0.95 RPM | 3.38° | 7.68° | 2 (CLOSED) | **PASS** |
| `scenario_7_inductance_mismatch` | Ls Mismatch (-20%) | None | 2000 RPM | N/A | 0.00 RPM | 5.87° | 12.56° | 2 (CLOSED) | **PASS** |
| `scenario_8_speed_reversal` | Reversal (+2000 to -2000) | None | -2000 RPM | N/A | 178.43 RPM | 6.63° | 120.46° | 2 (CLOSED) | **PASS** |
| `scenario_9_load_step` | Load Step (0.1 Nm) | None | 1800 RPM | N/A | 1.29 RPM | 2.98° | 5.02° | 2 (CLOSED) | **PASS** |
| `scenario_10_low_speed` | Low Speed (500 RPM) | None | 500 RPM | N/A | 18.79 RPM | 16.90° | 75.53° | 2 (CLOSED) | **PASS** |
| `scenario_11_slow_ramp` | Speed Ramp (500 RPM/s) | None | 500 RPM | N/A | 248.51 RPM | 6.68° | 16.40° | 2 (CLOSED) | **PASS** |
| `scenario_12_ambiguous_fault` | Symmetric 3-Phase Offset | A, B, C | 2000 RPM | 30.0 ms | 252.14 RPM | 2.80° | 7.15° | 4 (LIMP) | **PASS** |

### 3.2 144-Case Robustness Isolation Sweep
Command: `python3 scripts/isolation_sweep.py --phases 4`

| Fault Type | Reference Speed | Correctly Isolated | Acceptable Outcomes | Outcome Rate | Target | Worst-Case Ride-Through |
|---|---|---|---|---|---|---|
| `sensor_offset 0.5 A` | 1500 RPM | 8 / 24 (33%) | 24 / 24 | **100%** | $\ge 90\%$ | Mean: 1498 RPM, Max Angle: 13° |
| `sensor_offset 0.5 A` | 2000 RPM | 14 / 24 (58%) | 24 / 24 | **100%** | $\ge 90\%$ | Mean: 1969 RPM, Max Angle: 16° |
| `sensor_offset 2.0 A` | 1500 RPM | 23 / 24 (96%) | 23 / 24 | **96%** | $\ge 90\%$ | Mean: 910 RPM, Max Angle: 180° |
| `sensor_offset 2.0 A` | 2000 RPM | 22 / 24 (92%) | 22 / 24 | **92%** | $\ge 90\%$ | Mean: 907 RPM, Max Angle: 179° |
| `sensor_stuck` | 1500 RPM | 24 / 24 (100%) | 24 / 24 | **100%** | $\ge 90\%$ | Mean: 1500 RPM, Max Angle: 8° |
| `sensor_stuck` | 2000 RPM | 24 / 24 (100%) | 24 / 24 | **100%** | $\ge 90\%$ | Mean: 1998 RPM, Max Angle: 7° |
| **Total / Overall** | **All Configurations** | **115 / 144 (79.9%)** | **141 / 144** | **97.9%** | **$\ge 90\%$** | **MET (Exit Code 0)** |

### 3.3 MATLAB / Simulink Simulation Results
Model: `simulink/pmsm_foc_simulink.slx` executed via MATLAB R2020b  
Solver: `ode4` (Runge-Kutta 4th order, fixed-step $\Delta t = 10\,\mu\text{s}$)

- **Steady-State Speed Error:** 4.85 RPM (Target: 1800.0 RPM)
- **Pre-Fault Angle Error RMS:** 3.75° (Max: 6.16°)
- **Fault Detection Latency:** 0.96 ms
- **Post-Fault Angle Error RMS:** 3.74° (Max: 6.25°)
- **Phase A Reconstructed Current RMS Error:** 0.052 A
- **Mode Sequence:** `CLOSED_LOOP (2)` $\rightarrow$ `DEGRADED (3)`
- **Exported Figures:**
  - `simulink/results/simulink_pmsm_foc_response.png`
  - `simulink/results/simulink_pmsm_fault_isolation.png`

### 3.4 PlatformIO Firmware Build Metrics
Environment: `esp32dev` (`platform = espressif32@6.5.0`, `framework = arduino`)

- **RAM Consumption:** 22,316 bytes (6.8% of 327,680 bytes)
- **Flash Consumption:** 297,785 bytes (22.7% of 1,310,720 bytes)
- **Compilation Status:** 100% SUCCESS (`firmware.bin` and `firmware.elf` generated)
- **Cycle Timing Budget:** 12,000 cycles (50 $\mu$s budget @ 240 MHz); telemetry cycle tracking active.

---

## 4. Distinction of Environments and Validation Scope

To maintain engineering integrity, the levels of validation are explicitly classified:
1. **Python SIL (Software-in-the-Loop):** Fully verified (13/13 scenarios, 144 sweep cases, 9/9 pytests).
2. **MATLAB / Simulink Co-Simulation:** Fully verified with compiled C-MEX S-functions.
3. **PlatformIO Embedded Build:** Fully verified toolchain compilation for ESP32 target.
4. **Wokwi Simulation:** Functional emulation demonstrating real-time virtual motor plant and LED/serial telemetry.
5. **Physical Motor Bench Testing:** **Pending.** Requires physical inverter hardware, current shunt op-amps, and an instrumented PMSM dynamometer.
