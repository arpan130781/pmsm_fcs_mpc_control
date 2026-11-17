"""
Ctypes Bridge for FCS-MPC PMSM Controller Shared Library
Author: Arpan Kailash Gupta (Roll No: 241030009), VJTI Mumbai
"""

import ctypes
import os
import sys

class MPCTelemetry(ctypes.Structure):
    _fields_ = [
        ("i_a", ctypes.c_float),
        ("i_b", ctypes.c_float),
        ("i_c", ctypes.c_float),
        ("i_d", ctypes.c_float),
        ("i_q", ctypes.c_float),
        ("i_d_ref", ctypes.c_float),
        ("i_q_ref", ctypes.c_float),
        ("theta_e", ctypes.c_float),
        ("omega_e", ctypes.c_float),
        ("speed_rpm", ctypes.c_float),
        ("speed_ref_rpm", ctypes.c_float),
        ("torque_elec", ctypes.c_float),
        
        ("sa", ctypes.c_uint8),
        ("sb", ctypes.c_uint8),
        ("sc", ctypes.c_uint8),
        ("opt_vector", ctypes.c_uint8),
        ("v_dc", ctypes.c_float),
        ("v_alpha", ctypes.c_float),
        ("v_beta", ctypes.c_float),
        ("v_cm", ctypes.c_float),
        ("p_conduction", ctypes.c_float),
        ("p_switching", ctypes.c_float),
        ("p_total_loss", ctypes.c_float),
        ("efficiency", ctypes.c_float),
        ("switching_freq_hz", ctypes.c_float),
        ("total_commutations", ctypes.c_uint32),
        
        ("min_cost", ctypes.c_float),
    ]

class FCSMPCController:
    def __init__(self, lib_path: str = None):
        if lib_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            ext = "dylib" if sys.platform == "darwin" else "so"
            lib_path = os.path.join(base_dir, "build", f"libmpc_controller.{ext}")
            
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f"Shared library not found at {lib_path}. Run 'make lib' first.")
            
        self.lib = ctypes.CDLL(lib_path)
        
        # Configure function signatures
        self.lib.mpc_system_init.argtypes = [ctypes.c_float]
        self.lib.mpc_system_reset.argtypes = []
        self.lib.mpc_system_set_weights.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.mpc_system_step.argtypes = [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(MPCTelemetry)
        ]
        
    def init(self, v_dc_nominal: float = 24.0):
        self.lib.mpc_system_init(v_dc_nominal)
        
    def reset(self):
        self.lib.mpc_system_reset()
        
    def set_weights(self, lambda_d: float, lambda_q: float, lambda_sw: float, lambda_cmv: float):
        self.lib.mpc_system_set_weights(lambda_d, lambda_q, lambda_sw, lambda_cmv)
        
    def step(self, i_a: float, i_b: float, i_c: float,
             theta_e: float, speed_rpm: float,
             speed_ref_rpm: float, v_dc: float = 24.0) -> MPCTelemetry:
        telem = MPCTelemetry()
        self.lib.mpc_system_step(
            i_a, i_b, i_c,
            theta_e, speed_rpm,
            speed_ref_rpm, v_dc,
            ctypes.byref(telem)
        )
        return telem
