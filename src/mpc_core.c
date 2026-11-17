#include "mpc_core.h"
#include "motor_params.h"
#include <stddef.h>
#include <math.h>
#include <float.h>

#define TWO_THIRDS         (0.6666666667f)
#define ONE_OVER_SQRT3     (0.5773502692f)
#define TWO_PI             (6.2831853072f)

static inline float wrap_angle(float angle) {
    while (angle > (float)M_PI)  angle -= TWO_PI;
    while (angle < -(float)M_PI) angle += TWO_PI;
    return angle;
}

void clarke_transform(float a, float b, float c, float* alpha, float* beta) {
    if (alpha) *alpha = TWO_THIRDS * (a - 0.5f * b - 0.5f * c);
    if (beta)  *beta  = TWO_THIRDS * (0.8660254038f * (b - c));
}

void park_transform(float alpha, float beta, float sin_th, float cos_th, float* d, float* q) {
    if (d) *d =  alpha * cos_th + beta * sin_th;
    if (q) *q = -alpha * sin_th + beta * cos_th;
}

void inv_park_transform(float d, float q, float sin_th, float cos_th, float* alpha, float* beta) {
    if (alpha) *alpha = d * cos_th - q * sin_th;
    if (beta)  *beta  = d * sin_th + q * cos_th;
}

void mpc_init(FCS_MPC_t* mpc) {
    if (!mpc) return;
    mpc_reset(mpc);
    mpc->lambda_d = MPC_WEIGHT_ID;
    mpc->lambda_q = MPC_WEIGHT_IQ;
    mpc->lambda_sw = MPC_WEIGHT_SW;
    mpc->lambda_cmv = MPC_WEIGHT_CMV;
}

void mpc_reset(FCS_MPC_t* mpc) {
    if (!mpc) return;
    mpc->i_a = 0.0f;
    mpc->i_b = 0.0f;
    mpc->i_c = 0.0f;
    mpc->i_alpha = 0.0f;
    mpc->i_beta = 0.0f;
    mpc->i_d = 0.0f;
    mpc->i_q = 0.0f;
    mpc->theta_e = 0.0f;
    mpc->omega_e = 0.0f;
    mpc->v_dc = VSI_VBUS_NOMINAL;
    
    mpc->i_d_ref = 0.0f;
    mpc->i_q_ref = 0.0f;
    
    mpc->i_d_k1 = 0.0f;
    mpc->i_q_k1 = 0.0f;
    mpc->theta_e_k1 = 0.0f;
    
    mpc->opt_state_k = INVERTER_VECTORS[0];
    mpc->opt_state_k1 = INVERTER_VECTORS[0];
    mpc->opt_vector_idx = 0;
    
    mpc->min_cost = 0.0f;
    mpc->overcurrent_tripped = false;
    mpc->execution_cycles = 0;
    
    for (int i = 0; i < 8; i++) {
        mpc->candidate_costs[i] = 0.0f;
        mpc->predicted_i_d[i] = 0.0f;
        mpc->predicted_i_q[i] = 0.0f;
    }
}

void mpc_set_weights(FCS_MPC_t* mpc, float lambda_d, float lambda_q, float lambda_sw, float lambda_cmv) {
    if (!mpc) return;
    mpc->lambda_d = lambda_d;
    mpc->lambda_q = lambda_q;
    mpc->lambda_sw = lambda_sw;
    mpc->lambda_cmv = lambda_cmv;
}

void mpc_step(FCS_MPC_t* mpc,
              float i_a, float i_b, float i_c,
              float theta_e, float omega_e,
              float i_d_ref, float i_q_ref,
              float v_dc) {
    if (!mpc) return;
    
    mpc->execution_cycles++;
    mpc->i_a = i_a;
    mpc->i_b = i_b;
    mpc->i_c = i_c;
    mpc->theta_e = wrap_angle(theta_e);
    mpc->omega_e = omega_e;
    mpc->i_d_ref = i_d_ref;
    mpc->i_q_ref = i_q_ref;
    mpc->v_dc = (v_dc > 5.0f) ? v_dc : VSI_VBUS_NOMINAL;
    
    /* 1. Coordinate transformations for measured currents at step k */
    clarke_transform(i_a, i_b, i_c, &mpc->i_alpha, &mpc->i_beta);
    float sin_th = sinf(mpc->theta_e);
    float cos_th = cosf(mpc->theta_e);
    park_transform(mpc->i_alpha, mpc->i_beta, sin_th, cos_th, &mpc->i_d, &mpc->i_q);
    
    /* 2. Step 1: Delay Compensation - Predict state at step k+1 using opt_state_k */
    float v_alpha_k, v_beta_k;
    inverter_get_voltages(&mpc->opt_state_k, mpc->v_dc, &v_alpha_k, &v_beta_k, NULL, NULL, NULL, NULL);
    
    float v_d_k, v_q_k;
    park_transform(v_alpha_k, v_beta_k, sin_th, cos_th, &v_d_k, &v_q_k);
    
    float dt = MPC_TS;
    float Ld = MOTOR_LD;
    float Lq = MOTOR_LQ;
    float Rs = MOTOR_RS;
    float psi = MOTOR_FLUX;
    
    /* Discrete-time PMSM forward model to k+1:
     * i_d(k+1) = (1 - Rs*dt/Ld)*i_d(k) + (Lq*dt/Ld)*omega_e*i_q(k) + (dt/Ld)*v_d(k)
     * i_q(k+1) = (1 - Rs*dt/Lq)*i_q(k) - (Ld*dt/Lq)*omega_e*i_d(k) - (psi*dt/Lq)*omega_e + (dt/Lq)*v_q(k)
     */
    mpc->i_d_k1 = (1.0f - (Rs * dt / Ld)) * mpc->i_d + ((Lq * dt / Ld) * omega_e * mpc->i_q) + ((dt / Ld) * v_d_k);
    mpc->i_q_k1 = (1.0f - (Rs * dt / Lq)) * mpc->i_q - ((Ld * dt / Lq) * omega_e * mpc->i_d) - ((psi * dt / Lq) * omega_e) + ((dt / Lq) * v_q_k);
    mpc->theta_e_k1 = wrap_angle(mpc->theta_e + omega_e * dt);
    
    /* Electrical angle at k+1 and k+2 */
    float sin_th_k1 = sinf(mpc->theta_e_k1);
    float cos_th_k1 = cosf(mpc->theta_e_k1);
    
    /* 3. Step 2: Evaluate all 8 candidate switching vectors for interval [k+1, k+2] */
    float min_cost = FLT_MAX;
    uint8_t best_idx = 0;
    
    for (uint8_t j = 0; j < 8; j++) {
        InverterSwitchState_t cand = INVERTER_VECTORS[j];
        
        float v_alpha_j, v_beta_j, v_cm_j;
        inverter_get_voltages(&cand, mpc->v_dc, &v_alpha_j, &v_beta_j, NULL, NULL, NULL, &v_cm_j);
        
        float v_d_j, v_q_j;
        park_transform(v_alpha_j, v_beta_j, sin_th_k1, cos_th_k1, &v_d_j, &v_q_j);
        
        /* Predict current at step k+2 */
        float pred_id = (1.0f - (Rs * dt / Ld)) * mpc->i_d_k1 + ((Lq * dt / Ld) * omega_e * mpc->i_q_k1) + ((dt / Ld) * v_d_j);
        float pred_iq = (1.0f - (Rs * dt / Lq)) * mpc->i_q_k1 - ((Ld * dt / Lq) * omega_e * mpc->i_d_k1) - ((psi * dt / Lq) * omega_e) + ((dt / Lq) * v_q_j);
        
        mpc->predicted_i_d[j] = pred_id;
        mpc->predicted_i_q[j] = pred_iq;
        
        /* Objective 1: Current tracking error */
        float err_d = i_d_ref - pred_id;
        float err_q = i_q_ref - pred_iq;
        float j_tracking = mpc->lambda_d * (err_d * err_d) + mpc->lambda_q * (err_q * err_q);
        
        /* Objective 2: Switching commutation penalty (distance from currently applied opt_state_k) */
        uint8_t delta_sw = inverter_commutation_distance(mpc->opt_state_k, cand);
        float j_sw = mpc->lambda_sw * (float)delta_sw;
        
        /* Objective 3: Common-Mode Voltage (CMV) bearing stress penalty */
        float normalized_cmv = fabsf(v_cm_j) / mpc->v_dc;
        float j_cmv = mpc->lambda_cmv * (normalized_cmv * normalized_cmv);
        
        /* Objective 4: Overcurrent barrier constraint */
        float i_mag_sq = pred_id * pred_id + pred_iq * pred_iq;
        float j_penalty = 0.0f;
        if (i_mag_sq > (MOTOR_PEAK_CURRENT * MOTOR_PEAK_CURRENT)) {
            j_penalty = MPC_WEIGHT_OVERCURRENT * (i_mag_sq - (MOTOR_PEAK_CURRENT * MOTOR_PEAK_CURRENT));
        }
        
        float total_cost = j_tracking + j_sw + j_cmv + j_penalty;
        mpc->candidate_costs[j] = total_cost;
        
        if (total_cost < min_cost) {
            min_cost = total_cost;
            best_idx = j;
        }
    }
    
    /* 4. Select and latch the optimal vector for execution at k+1 */
    mpc->min_cost = min_cost;
    mpc->opt_vector_idx = best_idx;
    mpc->opt_state_k1 = INVERTER_VECTORS[best_idx];
    
    /* Advance the pipeline: opt_state_k1 becomes the applied vector for next sample */
    mpc->opt_state_k = mpc->opt_state_k1;
}
