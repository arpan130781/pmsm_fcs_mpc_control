#include "mpc_api.h"
#include "mpc_core.h"
#include "inverter_2lvsi.h"
#include "speed_controller.h"
#include "motor_params.h"
#include <stddef.h>
#include <math.h>

static FCS_MPC_t g_mpc;
static Inverter2LVSI_t g_inverter;
static SpeedController_t g_speed_ctrl;
static float g_iq_ref = 0.0f;
static uint32_t g_speed_tick_counter = 0;

void mpc_system_init(float v_dc_nominal) {
    mpc_init(&g_mpc);
    inverter_init(&g_inverter, v_dc_nominal);
    speed_controller_init(&g_speed_ctrl);
    g_iq_ref = 0.0f;
    g_speed_tick_counter = 0;
}

void mpc_system_reset(void) {
    mpc_reset(&g_mpc);
    inverter_init(&g_inverter, g_inverter.v_dc);
    speed_controller_reset(&g_speed_ctrl);
    g_iq_ref = 0.0f;
    g_speed_tick_counter = 0;
}

void mpc_system_set_weights(float lambda_d, float lambda_q, float lambda_sw, float lambda_cmv) {
    mpc_set_weights(&g_mpc, lambda_d, lambda_q, lambda_sw, lambda_cmv);
}

void mpc_system_step(float i_a, float i_b, float i_c,
                     float theta_e, float omega_m_rpm,
                     float speed_ref_rpm, float v_dc,
                     MPCTelemetry_t* out_telemetry) {
    /* Speed loop runs every 1.0 ms (decimated by 20 from 20 kHz current loop) */
    g_speed_tick_counter++;
    if (g_speed_tick_counter >= 20) {
        g_speed_tick_counter = 0;
        g_iq_ref = speed_controller_step(&g_speed_ctrl, speed_ref_rpm, omega_m_rpm, SPEED_LOOP_TS);
    }
    
    /* Convert mechanical RPM to electrical angular velocity: omega_e = omega_m * p = (rpm * 2pi / 60) * p */
    float omega_e = (omega_m_rpm * (6.2831853072f / 60.0f)) * (float)MOTOR_POLE_PAIRS;
    
    /* Execute FCS-MPC predictive optimization at 20 kHz */
    float id_ref = 0.0f; /* i_d = 0 control for non-salient surface PMSM (MTPA) */
    mpc_step(&g_mpc, i_a, i_b, i_c, theta_e, omega_e, id_ref, g_iq_ref, v_dc);
    
    /* Apply selected optimal vector to inverter physics & update losses */
    inverter_apply_state(&g_inverter, g_mpc.opt_state_k1, i_a, i_b, i_c, MPC_TS);
    
    /* Calculate instantaneous electromagnetic torque:
     * T_e = 1.5 * p * (psi_f * i_q + (L_d - L_q) * i_d * i_q) = 1.5 * p * psi_f * i_q
     */
    float torque_elec = 1.5f * (float)MOTOR_POLE_PAIRS * MOTOR_FLUX * g_mpc.i_q;
    
    /* Populate telemetry output if requested */
    if (out_telemetry) {
        out_telemetry->i_a = g_mpc.i_a;
        out_telemetry->i_b = g_mpc.i_b;
        out_telemetry->i_c = g_mpc.i_c;
        out_telemetry->i_d = g_mpc.i_d;
        out_telemetry->i_q = g_mpc.i_q;
        out_telemetry->i_d_ref = id_ref;
        out_telemetry->i_q_ref = g_iq_ref;
        out_telemetry->theta_e = g_mpc.theta_e;
        out_telemetry->omega_e = omega_e;
        out_telemetry->speed_rpm = omega_m_rpm;
        out_telemetry->speed_ref_rpm = speed_ref_rpm;
        out_telemetry->torque_elec = torque_elec;
        
        out_telemetry->sa = g_inverter.current_state.sa;
        out_telemetry->sb = g_inverter.current_state.sb;
        out_telemetry->sc = g_inverter.current_state.sc;
        out_telemetry->opt_vector = g_mpc.opt_vector_idx;
        out_telemetry->v_dc = g_inverter.v_dc;
        out_telemetry->v_alpha = g_inverter.v_alpha;
        out_telemetry->v_beta = g_inverter.v_beta;
        out_telemetry->v_cm = g_inverter.common_mode_voltage;
        out_telemetry->p_conduction = g_inverter.p_conduction;
        out_telemetry->p_switching = g_inverter.p_switching;
        out_telemetry->p_total_loss = g_inverter.p_total_loss;
        out_telemetry->efficiency = g_inverter.efficiency;
        out_telemetry->switching_freq_hz = g_inverter.switching_freq_hz;
        out_telemetry->total_commutations = g_inverter.total_commutations;
        
        out_telemetry->min_cost = g_mpc.min_cost;
    }
}

void mpc_get_vector_voltages(uint8_t vector_idx, float v_dc, float* v_alpha, float* v_beta, float* v_cm) {
    if (vector_idx >= 8) vector_idx = 0;
    inverter_get_voltages(&INVERTER_VECTORS[vector_idx], v_dc, v_alpha, v_beta, NULL, NULL, NULL, v_cm);
}
