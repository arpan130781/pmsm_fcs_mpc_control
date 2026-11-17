#ifndef INVERTER_2LVSI_H
#define INVERTER_2LVSI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Discrete 3-Phase Switching State for 2L-VSI */
typedef struct {
    uint8_t sa;     /* Phase A high-side switch state: 1 = High (Vbus), 0 = Low (GND) */
    uint8_t sb;     /* Phase B high-side switch state: 1 = High (Vbus), 0 = Low (GND) */
    uint8_t sc;     /* Phase C high-side switch state: 1 = High (Vbus), 0 = Low (GND) */
} InverterSwitchState_t;

/* Inverter State and Power Dissipation Monitor */
typedef struct {
    float v_dc;                     /* Current DC-link bus voltage (V) */
    InverterSwitchState_t current_state; /* Active switching state */
    uint32_t total_commutations;    /* Cumulative switch state transitions */
    float switching_freq_hz;        /* Instantaneous effective average switching frequency */
    
    /* Loss Calculations */
    float p_conduction;             /* Conduction power loss (W) */
    float p_switching;              /* Dynamic switching power loss (W) */
    float p_total_loss;             /* Total inverter power loss (W) */
    float efficiency;               /* Inverter efficiency percentage (%) */
    
    /* Power Electronics Metrics */
    float common_mode_voltage;      /* Instantaneous common-mode voltage V_cm (V) */
    float v_alpha;                  /* Stator stationary alpha-axis applied voltage */
    float v_beta;                   /* Stator stationary beta-axis applied voltage */
    float v_a_neutral;              /* Phase A terminal-to-neutral voltage */
    float v_b_neutral;              /* Phase B terminal-to-neutral voltage */
    float v_c_neutral;              /* Phase C terminal-to-neutral voltage */
} Inverter2LVSI_t;

/* Constant table of all 8 discrete switching vectors V0..V7 */
extern const InverterSwitchState_t INVERTER_VECTORS[8];

/* Initialize Inverter Monitor */
void inverter_init(Inverter2LVSI_t* inv, float v_dc_nominal);

/* Compute stationary frame (alpha, beta) voltage and neutral voltages for a given switching state */
void inverter_get_voltages(const InverterSwitchState_t* state, float v_dc,
                          float* v_alpha, float* v_beta,
                          float* v_an, float* v_bn, float* v_cn,
                          float* v_cm);

/* Apply state, count commutations, and update power losses */
void inverter_apply_state(Inverter2LVSI_t* inv, InverterSwitchState_t next_state,
                         float i_a, float i_b, float i_c, float dt);

/* Calculate commutations distance between two states */
uint8_t inverter_commutation_distance(InverterSwitchState_t s1, InverterSwitchState_t s2);

#ifdef __cplusplus
}
#endif

#endif /* INVERTER_2LVSI_H */
