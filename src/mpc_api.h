#ifndef MPC_API_H
#define MPC_API_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Top-level system telemetry returned to simulation / host */
typedef struct {
    /* Motor Variables */
    float i_a;
    float i_b;
    float i_c;
    float i_d;
    float i_q;
    float i_d_ref;
    float i_q_ref;
    float theta_e;
    float omega_e;
    float speed_rpm;
    float speed_ref_rpm;
    float torque_elec;
    
    /* Inverter & Power Electronics Variables */
    uint8_t sa;
    uint8_t sb;
    uint8_t sc;
    uint8_t opt_vector;
    float v_dc;
    float v_alpha;
    float v_beta;
    float v_cm;
    float p_conduction;
    float p_switching;
    float p_total_loss;
    float efficiency;
    float switching_freq_hz;
    uint32_t total_commutations;
    
    /* MPC Metrics */
    float min_cost;
} MPCTelemetry_t;

/* API Functions */
void mpc_system_init(float v_dc_nominal);
void mpc_system_reset(void);
void mpc_system_set_weights(float lambda_d, float lambda_q, float lambda_sw, float lambda_cmv);

/* Step function called at 20 kHz by SIL runner or MCU timer */
void mpc_system_step(float i_a, float i_b, float i_c,
                     float theta_e, float omega_m_rpm,
                     float speed_ref_rpm, float v_dc,
                     MPCTelemetry_t* out_telemetry);

/* Direct vector voltage evaluation helper */
void mpc_get_vector_voltages(uint8_t vector_idx, float v_dc, float* v_alpha, float* v_beta, float* v_cm);

#ifdef __cplusplus
}
#endif

#endif /* MPC_API_H */
