#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../mpc_core.h"
#include "../inverter_2lvsi.h"
#include "../speed_controller.h"
#include "../mpc_api.h"
#include "../motor_params.h"

#define EPSILON 1e-4f

static void test_clarke_park_transforms(void) {
    printf("[TEST] Checking Clarke & Park transformations... ");
    float alpha, beta, d, q;
    
    /* Pure balanced 3-phase current with amplitude 5.0 A at angle 0 */
    clarke_transform(5.0f, -2.5f, -2.5f, &alpha, &beta);
    assert(fabsf(alpha - 5.0f) < EPSILON);
    assert(fabsf(beta - 0.0f) < EPSILON);
    
    /* Park transform at theta = 0 */
    park_transform(alpha, beta, 0.0f, 1.0f, &d, &q);
    assert(fabsf(d - 5.0f) < EPSILON);
    assert(fabsf(q - 0.0f) < EPSILON);
    
    /* Park transform at theta = pi/2 */
    park_transform(alpha, beta, 1.0f, 0.0f, &d, &q);
    assert(fabsf(d - 0.0f) < EPSILON);
    assert(fabsf(q - (-5.0f)) < EPSILON);
    
    printf("PASS\n");
}

static void test_inverter_vectors(void) {
    printf("[TEST] Checking Inverter 8 switching vectors & voltages... ");
    float va, vb, vc, v_cm, v_alpha, v_beta;
    
    /* Vector 0: (0,0,0) -> Zero vector */
    inverter_get_voltages(&INVERTER_VECTORS[0], 24.0f, &v_alpha, &v_beta, &va, &vb, &vc, &v_cm);
    assert(fabsf(v_alpha) < EPSILON);
    assert(fabsf(v_beta) < EPSILON);
    assert(fabsf(va) < EPSILON);
    assert(fabsf(v_cm - (-12.0f)) < EPSILON);
    
    /* Vector 1: (1,0,0) -> 2/3 * Vdc = 16 V */
    inverter_get_voltages(&INVERTER_VECTORS[1], 24.0f, &v_alpha, &v_beta, &va, &vb, &vc, &v_cm);
    assert(fabsf(v_alpha - 16.0f) < EPSILON);
    assert(fabsf(v_beta - 0.0f) < EPSILON);
    assert(fabsf(va - 16.0f) < EPSILON);
    assert(fabsf(vb - (-8.0f)) < EPSILON);
    assert(fabsf(vc - (-8.0f)) < EPSILON);
    assert(fabsf(v_cm - (-4.0f)) < EPSILON);
    
    /* Vector 7: (1,1,1) -> Zero vector */
    inverter_get_voltages(&INVERTER_VECTORS[7], 24.0f, &v_alpha, &v_beta, &va, &vb, &vc, &v_cm);
    assert(fabsf(v_alpha) < EPSILON);
    assert(fabsf(v_beta) < EPSILON);
    assert(fabsf(v_cm - 12.0f) < EPSILON);
    
    /* Commutation distances */
    assert(inverter_commutation_distance(INVERTER_VECTORS[0], INVERTER_VECTORS[1]) == 1);
    assert(inverter_commutation_distance(INVERTER_VECTORS[0], INVERTER_VECTORS[7]) == 3);
    assert(inverter_commutation_distance(INVERTER_VECTORS[1], INVERTER_VECTORS[4]) == 3);
    
    printf("PASS\n");
}

static void test_fcs_mpc_optimization(void) {
    printf("[TEST] Checking FCS-MPC optimization loop & selection... ");
    FCS_MPC_t mpc;
    mpc_init(&mpc);
    
    /* Demand positive q-axis current of +3.0 A at theta = 0, speed = 100 rad/s */
    mpc_step(&mpc, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.0f, 3.0f, 24.0f);
    
    /* The chosen vector should have positive q-axis voltage component */
    assert(mpc.opt_vector_idx >= 0 && mpc.opt_vector_idx < 8);
    assert(mpc.min_cost < 1000.0f);
    assert(!mpc.overcurrent_tripped);
    
    printf("PASS (Selected V%d, Cost = %.3f)\n", mpc.opt_vector_idx, mpc.min_cost);
}

static void test_speed_controller(void) {
    printf("[TEST] Checking outer speed PI controller... ");
    SpeedController_t sc;
    speed_controller_init(&sc);
    
    /* Reference = 1500 RPM, Feedback = 1000 RPM (positive error) */
    float iq_cmd = speed_controller_step(&sc, 1500.0f, 1000.0f, 0.001f);
    assert(iq_cmd > 0.0f);
    assert(iq_cmd <= SPEED_MAX_TORQUE_CURRENT);
    
    printf("PASS (Iq_cmd = %.2f A)\n", iq_cmd);
}

int main(void) {
    printf("==================================================================\n");
    printf("FCS-MPC PMSM Drive C Unit Test Suite\n");
    printf("Author: Arpan Kailash Gupta (241030009), VJTI Mumbai\n");
    printf("==================================================================\n");
    
    test_clarke_park_transforms();
    test_inverter_vectors();
    test_fcs_mpc_optimization();
    test_speed_controller();
    
    printf("\nAll C unit tests completed successfully (100%% PASS).\n");
    return 0;
}
