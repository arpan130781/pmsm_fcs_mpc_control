#ifndef MPC_CORE_H
#define MPC_CORE_H

#include "inverter_2lvsi.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Finite-Control-Set Model Predictive Controller Data Structure */
typedef struct {
    /* Measured & Transformed Feedback */
    float i_a;
    float i_b;
    float i_c;
    float i_alpha;
    float i_beta;
    float i_d;
    float i_q;
    float theta_e;
    float omega_e;
    float v_dc;
    
    /* References */
    float i_d_ref;
    float i_q_ref;
    
    /* 1-Step Ahead Delay Compensated State (at step k+1) */
    float i_d_k1;
    float i_q_k1;
    float theta_e_k1;
    
    /* Applied and Selected Vectors */
    InverterSwitchState_t opt_state_k;   /* Currently applied vector during [k, k+1] */
    InverterSwitchState_t opt_state_k1;  /* Optimal vector selected for [k+1, k+2] */
    uint8_t opt_vector_idx;              /* Index 0..7 of optimal vector */
    
    /* Cost Function Weights */
    float lambda_d;
    float lambda_q;
    float lambda_sw;
    float lambda_cmv;
    
    /* Optimization Telemetry */
    float min_cost;
    float candidate_costs[8];
    float predicted_i_d[8];
    float predicted_i_q[8];
    
    /* Overcurrent & Safety Flags */
    bool overcurrent_tripped;
    uint32_t execution_cycles;
} FCS_MPC_t;

/* Initialize FCS-MPC controller */
void mpc_init(FCS_MPC_t* mpc);

/* Reset internal state */
void mpc_reset(FCS_MPC_t* mpc);

/* Set cost function weights */
void mpc_set_weights(FCS_MPC_t* mpc, float lambda_d, float lambda_q, float lambda_sw, float lambda_cmv);

/* Core execution step (Runs at Ts = 50 us / 20 kHz) */
void mpc_step(FCS_MPC_t* mpc,
              float i_a, float i_b, float i_c,
              float theta_e, float omega_e,
              float i_d_ref, float i_q_ref,
              float v_dc);

/* Coordinate transformations */
void clarke_transform(float a, float b, float c, float* alpha, float* beta);
void park_transform(float alpha, float beta, float sin_th, float cos_th, float* d, float* q);
void inv_park_transform(float d, float q, float sin_th, float cos_th, float* alpha, float* beta);

#ifdef __cplusplus
}
#endif

#endif /* MPC_CORE_H */
