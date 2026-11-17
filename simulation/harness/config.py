"""
Configuration Parameters for FCS-MPC PMSM Power Electronics Drive Simulation
Author: Arpan Kailash Gupta (Roll No: 241030009), VJTI Mumbai
"""

import os
import sys

ext = "dylib" if sys.platform == "darwin" else "so"

MOTOR_PARAMS = {
    'Rs': 0.50,
    'Ls': 1.0e-3,
    'psi_f': 0.015,
    'P': 4,
    'J': 0.000080,
    'B': 0.00010,
    'Vbus': 24.0,
    'In': 12.0
}

INVERTER_PARAMS = {
    'Vbus': 24.0,
    'R_ds_on': 0.012,
    't_rise': 25.0e-9,
    't_fall': 20.0e-9,
    'deadtime': 1.2e-6,
}

MPC_PARAMS = {
    'ts': 50.0e-6,          # 20 kHz current control & optimization loop
    'lambda_d': 1.0,        # d-axis current weight
    'lambda_q': 1.0,        # q-axis current weight
    'lambda_sw': 0.045,     # Switching commutation penalty weight
    'lambda_cmv': 0.020,    # Common-mode voltage reduction weight
    'spd_kp': 0.045,        # Outer speed loop P gain
    'spd_ki': 0.80,         # Outer speed loop I gain
}

SIM_PARAMS = {
    'dt_control': 50.0e-6,  # 20 kHz
    'dt_plant': 10.0e-6,    # 100 kHz continuous plant integration
    'total_time': 1.0,      # Default run duration (seconds)
    'speed_ref_rpm': 1500.0 # Default target speed
}

PATHS = {
    'lib_path': os.path.join(os.path.dirname(__file__), '..', '..', 'build', f'libmpc_controller.{ext}'),
    'results_dir': os.path.join(os.path.dirname(__file__), '..', '..', 'docs', 'figures'),
}
