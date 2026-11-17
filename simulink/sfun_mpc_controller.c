/*
 * sfun_mpc_controller.c
 * 
 * Level-2 MATLAB/Simulink C MEX S-Function wrapping Finite-Control-Set Model
 * Predictive Control (FCS-MPC) with Multi-Objective Optimization for PMSM Drives.
 *
 * Department of Electrical Engineering, Veermata Jijabai Technological Institute (VJTI)
 * Author: Arpan Kailash Gupta (Roll No: 241030009)
 *
 * Inputs (Port 0, Width 7):
 *   u[0..2] : [i_a, i_b, i_c]         - Measured 3-phase currents (A)
 *   u[3]    : theta_e                 - Electrical angle (rad)
 *   u[4]    : omega_m_rpm             - Rotor speed feedback (RPM)
 *   u[5]    : speed_ref_rpm           - Speed reference demand (RPM)
 *   u[6]    : v_dc                    - DC-link voltage (V)
 *
 * Outputs (Port 0, Width 14):
 *   y[0..2] : [Sa, Sb, Sc]            - Discrete switching states {0, 1}
 *   y[3]    : opt_vector              - Optimal vector index (0..7)
 *   y[4..5] : [id, iq]                - Measured d-q currents (A)
 *   y[6..7] : [id_ref, iq_ref]        - Reference d-q demands (A)
 *   y[8]    : torque_elec             - Electromagnetic torque (N*m)
 *   y[9]    : v_cm                    - Common-mode voltage (V)
 *   y[10]   : p_loss                  - Total inverter losses (W)
 *   y[11]   : efficiency              - Inverter efficiency (%)
 *   y[12]   : f_sw                    - Switching frequency (Hz)
 *   y[13]   : min_cost                - Optimal MPC cost value
 */

#define S_FUNCTION_NAME  sfun_mpc_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>
#include "../src/mpc_api.h"
#include "../src/motor_params.h"

#define NUM_PARAMS 1
#define PARAM_TS(S) ssGetSFcnParam(S, 0)

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* 1 Input Port of Width 7: [ia, ib, ic, theta_e, speed_rpm, speed_ref_rpm, v_dc] */
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 7);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    /* 1 Output Port of Width 14 */
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 14);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    real_T Ts = mxGetScalar(PARAM_TS(S));
    ssSetSampleTime(S, 0, Ts);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S) {
    mpc_system_init(VSI_VBUS_NOMINAL);
    mpc_system_set_weights(MPC_WEIGHT_ID, MPC_WEIGHT_IQ, MPC_WEIGHT_SW, MPC_WEIGHT_CMV);
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid) {
    const real_T *u = (const real_T*) ssGetInputPortSignal(S, 0);
    real_T       *y = (real_T*) ssGetOutputPortSignal(S, 0);

    float ia            = (float)u[0];
    float ib            = (float)u[1];
    float ic            = (float)u[2];
    float theta_e       = (float)u[3];
    float speed_rpm     = (float)u[4];
    float speed_ref_rpm = (float)u[5];
    float v_dc          = (float)u[6];

    MPCTelemetry_t telem;
    mpc_system_step(ia, ib, ic, theta_e, speed_rpm, speed_ref_rpm, v_dc, &telem);

    y[0]  = (real_T)telem.sa;
    y[1]  = (real_T)telem.sb;
    y[2]  = (real_T)telem.sc;
    y[3]  = (real_T)telem.opt_vector;
    y[4]  = (real_T)telem.i_d;
    y[5]  = (real_T)telem.i_q;
    y[6]  = (real_T)telem.i_d_ref;
    y[7]  = (real_T)telem.i_q_ref;
    y[8]  = (real_T)telem.torque_elec;
    y[9]  = (real_T)telem.v_cm;
    y[10] = (real_T)telem.p_total_loss;
    y[11] = (real_T)telem.efficiency;
    y[12] = (real_T)telem.switching_freq_hz;
    y[13] = (real_T)telem.min_cost;
}

static void mdlTerminate(SimStruct *S) {
    mpc_system_reset();
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
