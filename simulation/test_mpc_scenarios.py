"""
Automated Pytest Suite for FCS-MPC PMSM Drive
Author: Arpan Kailash Gupta (Roll No: 241030009), VJTI Mumbai
"""

import numpy as np
import pytest
from simulation.harness.mpc_runner import run_simulation

def test_mpc_speed_acceleration():
    """Verify that FCS-MPC accelerates the motor toward reference speed."""
    data = run_simulation(
        "test_accel", total_time=0.15,
        speed_profile=lambda t: 1500.0,
        load_profile=lambda t: 0.0
    )
    final_speed = data['speed'][-1]
    assert final_speed > 300.0, f"Expected speed > 300 RPM, got {final_speed:.1f}"

def test_mpc_switching_constraint():
    """Verify that average switching frequency is constrained below carrier frequency (20 kHz)."""
    data = run_simulation(
        "test_sw", total_time=0.20,
        speed_profile=lambda t: 1500.0,
        load_profile=lambda t: 0.05
    )
    # Check average switching frequency reported by telemetry
    f_sw_avg = np.mean(data['f_sw'][int(len(data['time'])*0.5):])
    assert f_sw_avg < 20000.0, f"Average switching frequency {f_sw_avg:.1f} Hz exceeds 20 kHz"
    assert f_sw_avg > 500.0, f"Average switching frequency {f_sw_avg:.1f} Hz unexpectedly low"

def test_mpc_torque_delivery():
    """Verify positive torque generation under positive speed command and load."""
    data = run_simulation(
        "test_torque", total_time=0.20,
        speed_profile=lambda t: 1500.0,
        load_profile=lambda t: 0.10
    )
    mean_torque = np.mean(data['torque'][int(len(data['time'])*0.5):])
    assert mean_torque > 0.05, f"Expected mean torque > 0.05 N*m, got {mean_torque:.3f}"

def test_inverter_efficiency_bounds():
    """Verify inverter electrical conversion efficiency exceeds 95% under normal operation."""
    data = run_simulation(
        "test_eff", total_time=0.20,
        speed_profile=lambda t: 1500.0,
        load_profile=lambda t: 0.10
    )
    mean_eff = np.mean(data['efficiency'][int(len(data['time'])*0.5):])
    assert mean_eff > 95.0, f"Expected inverter efficiency > 95%, got {mean_eff:.2f}%"
