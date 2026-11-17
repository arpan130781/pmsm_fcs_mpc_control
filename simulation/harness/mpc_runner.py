"""
FCS-MPC PMSM Drive Simulation Engine and Dynamic Benchmark Runner
Department of Electrical Engineering, Veermata Jijabai Technological Institute (VJTI)
Author: Arpan Kailash Gupta (Roll No: 241030009)
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

# Ensure local imports work cleanly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from plant.pmsm_model import PMSMModel
from plant.inverter_2lvsi import Inverter2LVSI
from harness.c_bridge import FCSMPCController
from harness.config import MOTOR_PARAMS, INVERTER_PARAMS, MPC_PARAMS, SIM_PARAMS, PATHS

def run_simulation(scenario_name: str, total_time: float = 0.8,
                   speed_profile=None, load_profile=None, vdc_profile=None,
                   lambda_sw: float = 0.045, lambda_cmv: float = 0.020):
    """
    Executes high-fidelity closed-loop FCS-MPC PMSM drive simulation.
    """
    dt_ctrl = SIM_PARAMS['dt_control']  # 50 us (20 kHz)
    dt_plant = SIM_PARAMS['dt_plant']   # 10 us (100 kHz)
    substeps = int(round(dt_ctrl / dt_plant))
    steps = int(round(total_time / dt_ctrl))
    
    # Instantiate Plant, Inverter and Controller
    motor = PMSMModel(
        Rs=MOTOR_PARAMS['Rs'], Ls=MOTOR_PARAMS['Ls'], psi_f=MOTOR_PARAMS['psi_f'],
        P=MOTOR_PARAMS['P'], J=MOTOR_PARAMS['J'], B=MOTOR_PARAMS['B'],
        Vbus=MOTOR_PARAMS['Vbus']
    )
    inverter = Inverter2LVSI(v_dc=INVERTER_PARAMS['Vbus'], r_ds_on=INVERTER_PARAMS['R_ds_on'])
    
    controller = FCSMPCController(PATHS['lib_path'])
    controller.init(v_dc_nominal=INVERTER_PARAMS['Vbus'])
    controller.set_weights(MPC_PARAMS['lambda_d'], MPC_PARAMS['lambda_q'], lambda_sw, lambda_cmv)
    
    # Telemetry logging arrays
    time_arr = np.zeros(steps)
    speed_ref_arr = np.zeros(steps)
    speed_arr = np.zeros(steps)
    torque_arr = np.zeros(steps)
    load_arr = np.zeros(steps)
    id_arr = np.zeros(steps)
    iq_arr = np.zeros(steps)
    id_ref_arr = np.zeros(steps)
    iq_ref_arr = np.zeros(steps)
    ia_arr = np.zeros(steps)
    ib_arr = np.zeros(steps)
    ic_arr = np.zeros(steps)
    sa_arr = np.zeros(steps, dtype=int)
    sb_arr = np.zeros(steps, dtype=int)
    sc_arr = np.zeros(steps, dtype=int)
    opt_vec_arr = np.zeros(steps, dtype=int)
    v_dc_arr = np.zeros(steps)
    v_cm_arr = np.zeros(steps)
    p_loss_arr = np.zeros(steps)
    eff_arr = np.zeros(steps)
    f_sw_arr = np.zeros(steps)
    
    current_state = (0, 0, 0)
    
    for k in range(steps):
        t = k * dt_ctrl
        time_arr[k] = t
        
        # Determine dynamic profiles
        spd_ref = speed_profile(t) if speed_profile else 1500.0
        t_load = load_profile(t) if load_profile else 0.0
        v_dc = vdc_profile(t) if vdc_profile else INVERTER_PARAMS['Vbus']
        
        speed_ref_arr[k] = spd_ref
        load_arr[k] = t_load
        v_dc_arr[k] = v_dc
        
        # Measure feedback from physical plant
        ia_meas, ib_meas, ic_meas = motor.get_phase_currents()
        theta_e_meas = motor.theta_e
        speed_rpm_meas = motor.speed_rpm
        
        # Execute FCS-MPC step
        telem = controller.step(ia_meas, ib_meas, ic_meas, theta_e_meas, speed_rpm_meas, spd_ref, v_dc)
        
        # Log telemetry
        speed_arr[k] = telem.speed_rpm
        torque_arr[k] = telem.torque_elec
        id_arr[k] = telem.i_d
        iq_arr[k] = telem.i_q
        id_ref_arr[k] = telem.i_d_ref
        iq_ref_arr[k] = telem.i_q_ref
        ia_arr[k] = ia_meas
        ib_arr[k] = ib_meas
        ic_arr[k] = ic_meas
        sa_arr[k] = telem.sa
        sb_arr[k] = telem.sb
        sc_arr[k] = telem.sc
        opt_vec_arr[k] = telem.opt_vector
        v_cm_arr[k] = telem.v_cm
        p_loss_arr[k] = telem.p_total_loss
        eff_arr[k] = telem.efficiency
        f_sw_arr[k] = telem.switching_freq_hz
        
        # Apply optimal switching vector to inverter & plant
        opt_state = (telem.sa, telem.sb, telem.sc)
        v_alpha, v_beta, v_an, v_bn, v_cn, v_cm = inverter.step(
            opt_state, ia_meas, ib_meas, ic_meas, dt_ctrl, v_dc
        )
        
        # Convert stationary alpha-beta voltages to synchronous d-q voltages for motor
        cos_th = math.cos(motor.theta_e)
        sin_th = math.sin(motor.theta_e)
        v_d = v_alpha * cos_th + v_beta * sin_th
        v_q = -v_alpha * sin_th + v_beta * cos_th
        
        # Sub-step continuous physical plant integration (100 kHz RK4)
        for _ in range(substeps):
            motor.step(v_d, v_q, t_load, dt_plant)
            
    return {
        'time': time_arr,
        'speed_ref': speed_ref_arr,
        'speed': speed_arr,
        'torque': torque_arr,
        'load': load_arr,
        'id': id_arr,
        'iq': iq_arr,
        'id_ref': id_ref_arr,
        'iq_ref': iq_ref_arr,
        'ia': ia_arr,
        'ib': ib_arr,
        'ic': ic_arr,
        'sa': sa_arr,
        'sb': sb_arr,
        'sc': sc_arr,
        'opt_vec': opt_vec_arr,
        'v_dc': v_dc_arr,
        'v_cm': v_cm_arr,
        'p_loss': p_loss_arr,
        'efficiency': eff_arr,
        'f_sw': f_sw_arr,
    }

def plot_scenario_1_dynamic_response(data, save_path):
    """Plot Scenario 1: Dynamic Step Speed & Torque Response"""
    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    t = data['time']
    
    # 1. Rotor Speed
    axs[0].plot(t, data['speed_ref'], 'r--', linewidth=2.0, label='Speed Reference $\omega_m^*$ (RPM)')
    axs[0].plot(t, data['speed'], 'b-', linewidth=1.5, label='FCS-MPC Rotor Speed $\omega_m$ (RPM)')
    axs[0].set_ylabel('Speed (RPM)', fontsize=11, fontweight='bold')
    axs[0].set_title('Finite-Control-Set Model Predictive Control (FCS-MPC): Dynamic Speed & Torque Response\nAuthor: Arpan Kailash Gupta (241030009), Department of Electrical Engineering, VJTI Mumbai', fontsize=12, fontweight='bold', pad=10)
    axs[0].grid(True, linestyle=':', alpha=0.7)
    axs[0].legend(loc='lower right', framealpha=0.9)
    
    # 2. Torque & Load
    axs[1].plot(t, data['torque'], 'g-', linewidth=1.5, label='Electromagnetic Torque $T_e$ (N·m)')
    axs[1].plot(t, data['load'], 'k--', linewidth=2.0, label='Applied Shaft Load $T_L$ (N·m)')
    axs[1].set_ylabel('Torque (N·m)', fontsize=11, fontweight='bold')
    axs[1].grid(True, linestyle=':', alpha=0.7)
    axs[1].legend(loc='upper right', framealpha=0.9)
    
    # 3. Synchronous d-q Currents
    axs[2].plot(t, data['iq_ref'], 'm--', linewidth=1.8, label='$i_q^*$ Demand (A)')
    axs[2].plot(t, data['iq'], 'm-', linewidth=1.2, alpha=0.9, label='$i_q$ Actual (A)')
    axs[2].plot(t, data['id'], 'c-', linewidth=1.2, alpha=0.9, label='$i_d$ Actual (A, MTPA target = 0)')
    axs[2].set_ylabel('Current (A)', fontsize=11, fontweight='bold')
    axs[2].grid(True, linestyle=':', alpha=0.7)
    axs[2].legend(loc='upper right', framealpha=0.9)
    
    # 4. Inverter Losses & Efficiency
    ax4_eff = axs[3].twinx()
    l1 = axs[3].plot(t, data['p_loss'], 'r-', linewidth=1.2, label='Inverter Total Loss $P_{loss}$ (W)')
    l2 = ax4_eff.plot(t, data['efficiency'], 'b-', linewidth=1.2, alpha=0.8, label='Inverter Efficiency $\eta$ (%)')
    axs[3].set_ylabel('Loss (W)', fontsize=11, fontweight='bold', color='r')
    ax4_eff.set_ylabel('Efficiency (%)', fontsize=11, fontweight='bold', color='b')
    axs[3].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    axs[3].grid(True, linestyle=':', alpha=0.7)
    
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    axs[3].legend(lines, labels, loc='lower right', framealpha=0.9)
    
    plt.tight_layout()
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.savefig(save_path, dpi=300)
    plt.close()
    print(f"[SUCCESS] Saved Scenario 1 plot to {save_path}")

def plot_scenario_2_switching_tradeoff(results_dict, save_path):
    """Plot Scenario 2: Switching Weight Trade-off Analysis"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    lambdas = []
    avg_fsw = []
    avg_losses = []
    thd_values = []
    
    for l_sw, res in results_dict.items():
        lambdas.append(l_sw)
        # Average switching freq in steady state (t > 0.4s)
        idx_ss = res['time'] > 0.4
        fsw_mean = np.mean(res['f_sw'][idx_ss]) / 1000.0 # kHz
        loss_mean = np.mean(res['p_loss'][idx_ss])
        
        # Calculate current THD on phase A in steady state
        ia_ss = res['ia'][idx_ss]
        fft_vals = np.abs(np.fft.rfft(ia_ss))
        fund_peak = np.max(fft_vals[1:100])
        harm_energy = np.sqrt(np.sum(fft_vals[1:]**2) - fund_peak**2)
        thd = (harm_energy / (fund_peak + 1e-6)) * 100.0
        
        avg_fsw.append(fsw_mean)
        avg_losses.append(loss_mean)
        thd_values.append(thd)
        
    ax1.plot(lambdas, avg_fsw, 'bo-', linewidth=2.0, markersize=8, label='Avg Switching Freq $f_{sw}$ (kHz)')
    ax1.set_xlabel('Switching Penalty Weight $\lambda_{sw}$', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Switching Frequency (kHz)', fontsize=11, fontweight='bold', color='b')
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.set_title('Switching Frequency Reduction via FCS-MPC', fontsize=11, fontweight='bold')
    
    ax1_twin = ax1.twinx()
    ax1_twin.plot(lambdas, avg_losses, 'rs--', linewidth=2.0, markersize=8, label='Inverter Loss $P_{loss}$ (W)')
    ax1_twin.set_ylabel('Total Inverter Power Loss (W)', fontsize=11, fontweight='bold', color='r')
    
    # Pareto Curve: Loss vs THD
    ax2.plot(avg_fsw, thd_values, 'go-', linewidth=2.0, markersize=8)
    for i, txt in enumerate(lambdas):
        ax2.annotate(f'$\lambda_{{sw}}={txt}$', (avg_fsw[i], thd_values[i]), textcoords="offset points", xytext=(0,10), ha='center', fontsize=9, fontweight='bold')
    ax2.set_xlabel('Average Switching Frequency $f_{sw}$ (kHz)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Current Total Harmonic Distortion THD (%)', fontsize=11, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.set_title('Pareto Trade-off: Switching Freq vs Current THD\nAuthor: Arpan Kailash Gupta (241030009), VJTI', fontsize=11, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()
    print(f"[SUCCESS] Saved Scenario 2 Pareto plot to {save_path}")

def run_all_benchmarks():
    out_dir = PATHS['results_dir']
    os.makedirs(out_dir, exist_ok=True)
    
    print("==================================================================")
    print("Executing FCS-MPC PMSM Drive Benchmark Simulation Suite")
    print("Department of Electrical Engineering, VJTI Mumbai")
    print("Author: Arpan Kailash Gupta (Roll No: 241030009)")
    print("==================================================================")
    
    # Scenario 1: Step speed from 0 to 1500 RPM, then 0.4 N*m load at t = 0.4 s
    print("\n[SCENARIO 1] Dynamic Step Speed & Torque Step Response...")
    def spd_prof(t):
        return 1500.0 if t >= 0.05 else 0.0
    def load_prof(t):
        return 0.40 if t >= 0.40 else 0.0
    
    data_sc1 = run_simulation("scenario_1", total_time=0.7, speed_profile=spd_prof, load_profile=load_prof)
    plot_scenario_1_dynamic_response(data_sc1, os.path.join(out_dir, "mpc_dynamic_speed_torque_response.png"))
    
    # Scenario 2: Switching frequency weighting sweep (lambda_sw)
    print("\n[SCENARIO 2] Multi-Objective Switching Penalty Sweep (Pareto Analysis)...")
    res_weights = {}
    for l_sw in [0.0, 0.02, 0.045, 0.08, 0.12]:
        print(f"  -> Simulating with lambda_sw = {l_sw:.3f}...")
        res_weights[l_sw] = run_simulation(f"sw_sweep_{l_sw}", total_time=0.5, speed_profile=spd_prof, load_profile=load_prof, lambda_sw=l_sw)
    plot_scenario_2_switching_tradeoff(res_weights, os.path.join(out_dir, "mpc_switching_frequency_pareto.png"))
    
    # Scenario 3: DC-Link Supply Sag Rejection (24V -> 18V at t = 0.35s)
    print("\n[SCENARIO 3] DC-Link Supply Sag Disturbance Rejection...")
    def vdc_sag_prof(t):
        return 18.0 if t >= 0.35 else 24.0
    data_sc3 = run_simulation("scenario_3_dc_sag", total_time=0.6, speed_profile=spd_prof, load_profile=load_prof, vdc_profile=vdc_sag_prof)
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(data_sc3['time'], data_sc3['v_dc'], 'r-', linewidth=2.0, label='DC-Link Bus Voltage $V_{dc}$ (V)')
    axs[0].axvline(0.35, color='k', linestyle=':', label='25% Supply Voltage Sag Injected')
    axs[0].set_ylabel('Bus Voltage (V)', fontsize=11, fontweight='bold')
    axs[0].set_title('Scenario 3: FCS-MPC DC-Link Voltage Sag Disturbance Rejection\nAuthor: Arpan Kailash Gupta (241030009), VJTI Mumbai', fontsize=12, fontweight='bold')
    axs[0].grid(True, linestyle=':', alpha=0.7)
    axs[0].legend(loc='lower left', framealpha=0.9)
    
    axs[1].plot(data_sc3['time'], data_sc3['speed_ref'], 'r--', linewidth=2.0, label='Speed Reference (RPM)')
    axs[1].plot(data_sc3['time'], data_sc3['speed'], 'b-', linewidth=1.5, label='Actual Speed (RPM)')
    axs[1].set_ylabel('Speed (RPM)', fontsize=11, fontweight='bold')
    axs[1].grid(True, linestyle=':', alpha=0.7)
    axs[1].legend(loc='lower right', framealpha=0.9)
    
    axs[2].plot(data_sc3['time'], data_sc3['iq'], 'm-', linewidth=1.2, label='$i_q$ Torque Current (A)')
    axs[2].plot(data_sc3['time'], data_sc3['id'], 'c-', linewidth=1.2, label='$i_d$ Flux Current (A)')
    axs[2].set_ylabel('Current (A)', fontsize=11, fontweight='bold')
    axs[2].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    axs[2].grid(True, linestyle=':', alpha=0.7)
    axs[2].legend(loc='upper right', framealpha=0.9)
    
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "mpc_dc_link_sag_rejection.png"), dpi=300)
    plt.close()
    print(f"[SUCCESS] Saved Scenario 3 plot to {os.path.join(out_dir, 'mpc_dc_link_sag_rejection.png')}")
    
    print("\n==================================================================")
    print("All FCS-MPC Power Electronics benchmarks completed successfully!")
    print(f"Publication-grade waveform figures saved in: {out_dir}")
    print("==================================================================")

if __name__ == '__main__':
    run_all_benchmarks()
