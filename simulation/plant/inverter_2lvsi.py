"""
2-Level 3-Phase Voltage Source Inverter (2L-VSI) Simulation Model
Part of FCS-MPC PMSM Drive Suite
Author: Arpan Kailash Gupta (Roll No: 241030009), VJTI Mumbai
"""

import math
import numpy as np

class Inverter2LVSI:
    VECTORS = [
        (0, 0, 0), # V0
        (1, 0, 0), # V1
        (1, 1, 0), # V2
        (0, 1, 0), # V3
        (0, 1, 1), # V4
        (0, 0, 1), # V5
        (1, 0, 1), # V6
        (1, 1, 1), # V7
    ]
    
    def __init__(self, v_dc: float = 24.0, r_ds_on: float = 0.012, t_rise: float = 25e-9, t_fall: float = 20e-9):
        self.v_dc = v_dc
        self.r_ds_on = r_ds_on
        self.t_rise = t_rise
        self.t_fall = t_fall
        self.current_state = (0, 0, 0)
        self.total_commutations = 0
        self.p_conduction = 0.0
        self.p_switching = 0.0
        self.switching_freq_hz = 0.0
        
    def reset(self):
        self.current_state = (0, 0, 0)
        self.total_commutations = 0
        self.p_conduction = 0.0
        self.p_switching = 0.0
        self.switching_freq_hz = 0.0

    def compute_voltages(self, state: tuple, v_dc: float = None):
        if v_dc is None:
            v_dc = self.v_dc
        sa, sb, sc = state
        
        # Ground referenced potentials
        v_ag = sa * v_dc
        v_bg = sb * v_dc
        v_cg = sc * v_dc
        
        # Neutral point potential
        v_ng = (v_ag + v_bg + v_cg) / 3.0
        
        # Phase-to-neutral voltages
        v_an = v_ag - v_ng
        v_bn = v_bg - v_ng
        v_cn = v_cg - v_ng
        
        # Clarke transformation
        v_alpha = (2.0 / 3.0) * (v_an - 0.5 * v_bn - 0.5 * v_cn)
        v_beta = (2.0 / 3.0) * (math.sqrt(3.0) / 2.0 * (v_bn - v_cn))
        
        # Common-mode voltage relative to DC-link center point
        v_cm = v_ng - 0.5 * v_dc
        
        return v_alpha, v_beta, v_an, v_bn, v_cn, v_cm

    def step(self, next_state: tuple, i_a: float, i_b: float, i_c: float, dt: float, v_dc: float = None):
        if v_dc is not None:
            self.v_dc = v_dc
            
        # Count commutations
        delta_sw = abs(next_state[0] - self.current_state[0]) + \
                   abs(next_state[1] - self.current_state[1]) + \
                   abs(next_state[2] - self.current_state[2])
        self.total_commutations += delta_sw
        
        # Average switching frequency across 6 switches
        inst_fsw = delta_sw / (6.0 * dt)
        self.switching_freq_hz = 0.99 * self.switching_freq_hz + 0.01 * inst_fsw
        
        # Switching loss
        e_sw = 0.0
        t_comm = self.t_rise + self.t_fall
        if next_state[0] != self.current_state[0]:
            e_sw += 0.5 * self.v_dc * abs(i_a) * t_comm
        if next_state[1] != self.current_state[1]:
            e_sw += 0.5 * self.v_dc * abs(i_b) * t_comm
        if next_state[2] != self.current_state[2]:
            e_sw += 0.5 * self.v_dc * abs(i_c) * t_comm
        self.p_switching = e_sw / dt
        
        # Conduction loss
        self.p_conduction = (i_a**2 + i_b**2 + i_c**2) * self.r_ds_on
        
        self.current_state = next_state
        return self.compute_voltages(self.current_state, self.v_dc)
