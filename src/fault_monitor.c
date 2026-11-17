#include "fault_monitor.h"
#include "foc_api.h"
#include <math.h>
#include <stdio.h>

#define KCL_THRESH 0.3f
#define ISOLATION_WINDOW 0.001f   /* 1 ms accumulation window */
#define RECOVERY_WINDOW 0.500f   /* 500 ms sustained healthy before clearing */

void fault_init(FaultMonitor* fm, float Rs, float Ls, float psi_f) {
    fm->Rs = Rs; fm->Ls = Ls; fm->psi_f = psi_f;
    fault_reset(fm);
}

void fault_reset(FaultMonitor* fm) {
    fm->status.sensor_fault_a = false;
    fm->status.sensor_fault_b = false;
    fm->status.sensor_fault_c = false;
    fm->status.i_a_rec = 0; fm->status.i_b_rec = 0; fm->status.i_c_rec = 0;
    fm->status.degradation_state = CLOSED_LOOP;
    fm->fault_latch_a = false; fm->fault_latch_b = false; fm->fault_latch_c = false;
    fm->clear_timer = 0; fm->fault_isolation_timer = 0;
    fm->unresolved_fault_timer = 0;
    fm->ambiguous_fault = false;
    fm->consecutive_confident = 0; fm->best_channel = -1;
    fm->err_acc_A = 0; fm->err_acc_B = 0; fm->err_acc_C = 0; fm->E_acc = 0;
    fm->i_alpha_pred = 0; fm->i_beta_pred = 0;
    fm->prev_i_a = 0; fm->prev_i_b = 0; fm->prev_i_c = 0;
    fm->stuck_count_a = 0; fm->stuck_count_b = 0; fm->stuck_count_c = 0;
}

FaultStatus fault_update(FaultMonitor* fm, float i_a, float i_b, float i_c,
                         float v_alpha, float v_beta,
                         float theta, float omega, float dt)
{
    /* ---- Stuck-sensor detection ---- */
    if (fabsf(i_a - fm->prev_i_a) < 0.0001f) fm->stuck_count_a++; else fm->stuck_count_a = 0;
    if (fabsf(i_b - fm->prev_i_b) < 0.0001f) fm->stuck_count_b++; else fm->stuck_count_b = 0;
    if (fabsf(i_c - fm->prev_i_c) < 0.0001f) fm->stuck_count_c++; else fm->stuck_count_c = 0;
    fm->prev_i_a = i_a; fm->prev_i_b = i_b; fm->prev_i_c = i_c;

    /* ---- KCL residual ---- */
    float E = i_a + i_b + i_c;
    bool k_fault = fabsf(E) > KCL_THRESH;

    /* ---- Kinematics: use observer estimates while unlatched ---- */
    if (!fm->fault_latch_a && !fm->fault_latch_b && !fm->fault_latch_c && !fm->ambiguous_fault) {
        fm->lkg_theta = theta;
        fm->lkg_omega = omega;
    } else {
        fm->lkg_theta += fm->lkg_omega * dt;
        if (fm->lkg_theta > (float)M_PI) fm->lkg_theta -= 2.0f * (float)M_PI;
        if (fm->lkg_theta < -(float)M_PI) fm->lkg_theta += 2.0f * (float)M_PI;
    }

    /* ---- Independent current predictor (open-loop observer) ---- */
    float e_alpha = -fm->psi_f * fm->lkg_omega * sinf(fm->lkg_theta);
    float e_beta  =  fm->psi_f * fm->lkg_omega * cosf(fm->lkg_theta);

    fm->i_alpha_pred += (dt / fm->Ls) * (v_alpha - fm->Rs * fm->i_alpha_pred - e_alpha);
    fm->i_beta_pred  += (dt / fm->Ls) * (v_beta  - fm->Rs * fm->i_beta_pred  - e_beta);

    /* Stabilise predictor with measured currents only when healthy */
    if (!k_fault && !fm->fault_latch_a && !fm->fault_latch_b && !fm->fault_latch_c) {
        float i_alpha_meas = (2.0f/3.0f) * (i_a - 0.5f*i_b - 0.5f*i_c);
        float i_beta_meas  = 0.57735027f * (i_b - i_c);
        fm->i_alpha_pred += 5000.0f * dt * (i_alpha_meas - fm->i_alpha_pred);
        fm->i_beta_pred  += 5000.0f * dt * (i_beta_meas  - fm->i_beta_pred);
    }

    /* Predicted abc from predictor */
    float ia_pred = fm->i_alpha_pred;
    float ib_pred = -0.5f * fm->i_alpha_pred + 0.8660254f * fm->i_beta_pred;
    float ic_pred = -0.5f * fm->i_alpha_pred - 0.8660254f * fm->i_beta_pred;

    /* ---- Clear fault-status flags for this tick ---- */
    fm->status.sensor_fault_a = false;
    fm->status.sensor_fault_b = false;
    fm->status.sensor_fault_c = false;

    /* ==== 1. Recovery / Clearing ==== */
    if (fm->fault_latch_a || fm->fault_latch_b || fm->fault_latch_c || fm->ambiguous_fault) {
        if (k_fault) {
            fm->clear_timer = 0;
        } else {
            fm->clear_timer += dt;
            if (fm->clear_timer > RECOVERY_WINDOW) {
                fm->fault_latch_a = false;
                fm->fault_latch_b = false;
                fm->fault_latch_c = false;
                fm->ambiguous_fault = false;
                fm->clear_timer = 0;
                fm->consecutive_confident = 0;
                fm->best_channel = -1;
            }
        }
    }

    /* ==== 2. Fault Isolation ==== */
    if (!fm->fault_latch_a && !fm->fault_latch_b && !fm->fault_latch_c) {
        if (k_fault) {
            fm->unresolved_fault_timer += dt;
            if (fm->unresolved_fault_timer >= 0.030f) {
                /* After 5ms of KCL violation without confident single-phase isolation -> ambiguous */
                fm->ambiguous_fault = true;
                fm->fault_isolation_timer = 0.0f;
            }
        } else {
            fm->unresolved_fault_timer = 0.0f;
        }

        if (k_fault && !fm->ambiguous_fault && fm->fault_isolation_timer == 0.0f) {
            /* Start a new isolation window */
            fm->fault_isolation_timer = 0.00001f;
            fm->err_acc_A = 0; fm->err_acc_B = 0; fm->err_acc_C = 0;
            fm->E_acc = 0;
        }

        if (fm->fault_isolation_timer > 0.0f && !fm->ambiguous_fault) {
            fm->fault_isolation_timer += dt;

            /*
             * Residual-correlation isolation metric (robust to predictor drift):
             *
             *   For a single DC offset of magnitude D on channel X:
             *     measurement_X = true_X + D  -->  error_X = meas_X - pred_X ≈ D = E
             *     error_Y ≈ 0 for healthy channels Y
             *
             *   Therefore accumulate  |error_ch - E|  for each channel.
             *   The faulty channel minimises this: |D - D| = 0.
             *   Healthy channels have |0 - D| = D  (worst case).
             *
             *   This metric is predictor-drift-invariant because the shared drift
             *   in (meas - pred) cancels in (error_ch - E) for the faulty channel,
             *   while it partially adds for healthy channels.
             */

            float err_a_signed = i_a - ia_pred;
            float err_b_signed = i_b - ib_pred;
            float err_c_signed = i_c - ic_pred;

            /* |channel_error - E| accumulation: min → faulty channel */
            float err_A = fabsf(err_a_signed - E);
            float err_B = fabsf(err_b_signed - E);
            float err_C = fabsf(err_c_signed - E);

            fm->err_acc_A += err_A * dt;
            fm->err_acc_B += err_B * dt;
            fm->err_acc_C += err_C * dt;
            fm->E_acc += fabsf(E) * dt;


            if (fm->fault_isolation_timer >= ISOLATION_WINDOW) {
                float eA = fm->err_acc_A;
                float eB = fm->err_acc_B;
                float eC = fm->err_acc_C;

                /* Find best (minimum error) hypothesis */
                float min_err = eA; int best = 0;
                if (eB < min_err) { min_err = eB; best = 1; }
                if (eC < min_err) { min_err = eC; best = 2; }

                float second_err = 1e9f;
                if (best == 0) second_err = fminf(eB, eC);
                else if (best == 1) second_err = fminf(eA, eC);
                else second_err = fminf(eA, eB);

                /*
                 * Confidence criterion with residual-correlation metric:
                 *
                 * With |error_ch - E|:
                 *   - Faulty channel accumulates ~0 (matches E well)
                 *   - Healthy channels accumulate ~E_acc (error ≈ 0, so |0 - E| = E)
                 *   - Three-phase fault: all channels ≈ (E/3) contribution each
                 *     so errors are roughly equal → low separation → ambiguous
                 *
                 * Require:
                 *   1. best channel is clearly better (separation > 25% of E_acc)
                 *   2. best is small relative to E_acc (min_err < 0.4 * E_acc)
                 *      (a large best means no channel matches E well — ambiguous)
                 */
                float separation = second_err - min_err;
                bool confident = (fm->E_acc > 0.0001f) && (
                    (separation > 0.35f * fm->E_acc && min_err < 0.65f * fm->E_acc) ||
                    (separation > 0.15f * fm->E_acc && min_err < 0.30f * fm->E_acc)
                );

                /* Stuck-sensor override: authoritative identification */
                if (fm->stuck_count_a > 20) { best = 0; confident = true; }
                else if (fm->stuck_count_b > 20) { best = 1; confident = true; }
                else if (fm->stuck_count_c > 20) { best = 2; confident = true; }

                if (confident) {
                    if (best == fm->best_channel) {
                        fm->consecutive_confident++;
                    } else {
                        fm->best_channel = best;
                        fm->consecutive_confident = 1;
                    }
                } else {
                    fm->consecutive_confident = 0;
                    fm->best_channel = -1;
                }

                if (fm->consecutive_confident >= 1) {
                    /* Latch the identified faulty channel */
                    if (fm->best_channel == 0) fm->fault_latch_a = true;
                    else if (fm->best_channel == 1) fm->fault_latch_b = true;
                    else fm->fault_latch_c = true;
                    fm->ambiguous_fault = false;
                    fm->unresolved_fault_timer = 0.0f;
                }

                /* Reset accumulators for next window */
                fm->err_acc_A = 0; fm->err_acc_B = 0; fm->err_acc_C = 0;
                fm->E_acc = 0;
                fm->fault_isolation_timer = 0.00001f;
            }
        }
    }

    /* ==== 3. Output Reconstruction & Mode ==== */
    if (fm->fault_latch_a && fm->fault_latch_b && fm->fault_latch_c) {
        fm->status.sensor_fault_a = true;
        fm->status.sensor_fault_b = true;
        fm->status.sensor_fault_c = true;
        fm->status.i_a_rec = i_a; fm->status.i_b_rec = i_b; fm->status.i_c_rec = i_c;
        fm->status.degradation_state = SAFE_STOP;
    }
    else if (fm->fault_latch_a) {
        fm->status.sensor_fault_a = true;
        fm->status.i_a_rec = -(i_b + i_c);
        fm->status.i_b_rec = i_b;
        fm->status.i_c_rec = i_c;
        fm->status.degradation_state = DEGRADED;
    }
    else if (fm->fault_latch_b) {
        fm->status.sensor_fault_b = true;
        fm->status.i_a_rec = i_a;
        fm->status.i_b_rec = -(i_a + i_c);
        fm->status.i_c_rec = i_c;
        fm->status.degradation_state = DEGRADED;
    }
    else if (fm->fault_latch_c) {
        fm->status.sensor_fault_c = true;
        fm->status.i_a_rec = i_a;
        fm->status.i_b_rec = i_b;
        fm->status.i_c_rec = -(i_a + i_b);
        fm->status.degradation_state = DEGRADED;
    }
    else if (fm->ambiguous_fault) {
        fm->status.i_a_rec = i_a;
        fm->status.i_b_rec = i_b;
        fm->status.i_c_rec = i_c;
        fm->status.degradation_state = LIMP;
    }
    else {
        fm->status.i_a_rec = i_a;
        fm->status.i_b_rec = i_b;
        fm->status.i_c_rec = i_c;
        fm->status.degradation_state = CLOSED_LOOP;
    }

    return fm->status;
}
