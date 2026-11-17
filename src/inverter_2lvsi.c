#include "inverter_2lvsi.h"
#include "motor_params.h"
#include <math.h>
#include <stdlib.h>

/* The 8 discrete space voltage switching vectors of a 2-Level VSI */
const InverterSwitchState_t INVERTER_VECTORS[8] = {
    {0, 0, 0}, /* V0: Zero Vector (all low) */
    {1, 0, 0}, /* V1: Active Vector (1,0,0) - 0 deg */
    {1, 1, 0}, /* V2: Active Vector (1,1,0) - 60 deg */
    {0, 1, 0}, /* V3: Active Vector (0,1,0) - 120 deg */
    {0, 1, 1}, /* V4: Active Vector (0,1,1) - 180 deg */
    {0, 0, 1}, /* V5: Active Vector (0,0,1) - 240 deg */
    {1, 0, 1}, /* V6: Active Vector (1,0,1) - 300 deg */
    {1, 1, 1}  /* V7: Zero Vector (all high) */
};

void inverter_init(Inverter2LVSI_t* inv, float v_dc_nominal) {
    if (!inv) return;
    inv->v_dc = (v_dc_nominal > 0.0f) ? v_dc_nominal : VSI_VBUS_NOMINAL;
    inv->current_state.sa = 0;
    inv->current_state.sb = 0;
    inv->current_state.sc = 0;
    inv->total_commutations = 0;
    inv->switching_freq_hz = 0.0f;
    inv->p_conduction = 0.0f;
    inv->p_switching = 0.0f;
    inv->p_total_loss = 0.0f;
    inv->efficiency = 100.0f;
    inv->common_mode_voltage = 0.0f;
    inv->v_alpha = 0.0f;
    inv->v_beta = 0.0f;
    inv->v_a_neutral = 0.0f;
    inv->v_b_neutral = 0.0f;
    inv->v_c_neutral = 0.0f;
}

void inverter_get_voltages(const InverterSwitchState_t* state, float v_dc,
                          float* v_alpha, float* v_beta,
                          float* v_an, float* v_bn, float* v_cn,
                          float* v_cm) {
    if (!state) return;
    
    /* Phase to ground potentials: 0 or V_dc */
    float v_ag = (float)state->sa * v_dc;
    float v_bg = (float)state->sb * v_dc;
    float v_cg = (float)state->sc * v_dc;
    
    /* Neutral to ground potential */
    float v_ng = (v_ag + v_bg + v_cg) * (1.0f / 3.0f);
    
    /* Phase-to-neutral voltages */
    float an = v_ag - v_ng;
    float bn = v_bg - v_ng;
    float cn = v_cg - v_ng;
    
    if (v_an) *v_an = an;
    if (v_bn) *v_bn = bn;
    if (v_cn) *v_cn = cn;
    
    /* Stationary frame Clarke Transformation:
     * v_alpha = (2/3) * (v_an - 0.5*v_bn - 0.5*v_cn) = v_dc/3 * (2*Sa - Sb - Sc)
     * v_beta  = (1/sqrt(3)) * (v_bn - v_cn)          = v_dc/sqrt(3) * (Sb - Sc)
     */
    if (v_alpha) {
        *v_alpha = (v_dc * (1.0f / 3.0f)) * (2.0f * (float)state->sa - (float)state->sb - (float)state->sc);
    }
    if (v_beta) {
        *v_beta = (v_dc * 0.577350269f) * ((float)state->sb - (float)state->sc);
    }
    
    /* Common-mode voltage relative to DC-link mid-point: V_cm = (v_ag + v_bg + v_cg)/3 - V_dc/2 */
    if (v_cm) {
        *v_cm = v_ng - (0.5f * v_dc);
    }
}

uint8_t inverter_commutation_distance(InverterSwitchState_t s1, InverterSwitchState_t s2) {
    uint8_t dist = 0;
    if (s1.sa != s2.sa) dist++;
    if (s1.sb != s2.sb) dist++;
    if (s1.sc != s2.sc) dist++;
    return dist;
}

void inverter_apply_state(Inverter2LVSI_t* inv, InverterSwitchState_t next_state,
                         float i_a, float i_b, float i_c, float dt) {
    if (!inv || dt <= 0.0f) return;
    
    /* 1. Calculate number of switch commutations */
    uint8_t delta_sw = inverter_commutation_distance(inv->current_state, next_state);
    inv->total_commutations += delta_sw;
    
    /* Instantaneous switching frequency across 6 switches */
    float inst_fsw = ((float)delta_sw / (6.0f * dt));
    inv->switching_freq_hz = 0.98f * inv->switching_freq_hz + 0.02f * inst_fsw;
    
    /* 2. Switching Energy Loss for switching transitions:
     * E_sw = 0.5 * V_dc * |I_phase| * (t_rise + t_fall)
     */
    float e_sw_total = 0.0f;
    float t_comm = (VSI_RISE_TIME_NS + VSI_FALL_TIME_NS);
    if (inv->current_state.sa != next_state.sa) {
        e_sw_total += 0.5f * inv->v_dc * fabsf(i_a) * t_comm;
    }
    if (inv->current_state.sb != next_state.sb) {
        e_sw_total += 0.5f * inv->v_dc * fabsf(i_b) * t_comm;
    }
    if (inv->current_state.sc != next_state.sc) {
        e_sw_total += 0.5f * inv->v_dc * fabsf(i_c) * t_comm;
    }
    inv->p_switching = e_sw_total / dt;
    
    /* 3. Conduction Loss across on-state MOSFET resistance */
    inv->p_conduction = (i_a * i_a + i_b * i_b + i_c * i_c) * VSI_RDS_ON;
    inv->p_total_loss = inv->p_conduction + inv->p_switching;
    
    /* 4. Update voltage outputs */
    inv->current_state = next_state;
    inverter_get_voltages(&inv->current_state, inv->v_dc,
                          &inv->v_alpha, &inv->v_beta,
                          &inv->v_a_neutral, &inv->v_b_neutral, &inv->v_c_neutral,
                          &inv->common_mode_voltage);
    
    /* 5. Inverter Efficiency */
    float p_out = inv->v_a_neutral * i_a + inv->v_b_neutral * i_b + inv->v_c_neutral * i_c;
    if (p_out > 0.1f) {
        inv->efficiency = (p_out / (p_out + inv->p_total_loss)) * 100.0f;
        if (inv->efficiency > 100.0f) inv->efficiency = 100.0f;
    } else {
        inv->efficiency = 100.0f;
    }
}
