#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Permanent Magnet Synchronous Motor (PMSM) Parameters
 * High-performance 100 W Surface PMSM for Robotic/EV Actuation
 * ========================================================================== */
#define MOTOR_RS                0.50f       /* Stator phase resistance (Ohms) */
#define MOTOR_LD                0.0010f     /* d-axis stator inductance (Henry, 1.0 mH) */
#define MOTOR_LQ                0.0010f     /* q-axis stator inductance (Henry, 1.0 mH) */
#define MOTOR_FLUX              0.015f      /* Permanent magnet rotor flux linkage (Weber, 15.0 mWb) */
#define MOTOR_POLE_PAIRS        4           /* Number of pole pairs (p = 4) */
#define MOTOR_J_INERTIA         0.000080f   /* Rotor + load inertia (kg*m^2) */
#define MOTOR_B_FRICTION        0.00010f    /* Viscous friction coefficient (N*m*s/rad) */
#define MOTOR_RATED_TORQUE      0.65f       /* Continuous rated electromagnetic torque (N*m) */
#define MOTOR_PEAK_CURRENT      12.0f       /* Maximum instantaneous phase current limit (Amperes) */
#define MOTOR_RATED_SPEED_RPM   2000.0f     /* Rated mechanical rotor speed (RPM) */

/* ==========================================================================
 * 2-Level 3-Phase Voltage Source Inverter (2L-VSI) Power Electronics
 * ========================================================================== */
#define VSI_VBUS_NOMINAL        24.0f       /* Nominal DC bus supply voltage (Volts) */
#define VSI_VBUS_MIN            16.0f       /* Minimum DC-link under-voltage fault threshold */
#define VSI_VBUS_MAX            32.0f       /* Maximum DC-link over-voltage protection threshold */
#define VSI_RDS_ON              0.012f      /* Power MOSFET on-state resistance (12 mOhms) */
#define VSI_VDIODE_DROP         0.70f       /* Anti-parallel freewheeling diode forward drop (Volts) */
#define VSI_RISE_TIME_NS        25.0e-9f    /* Switch turn-on rise time (25 ns) */
#define VSI_FALL_TIME_NS        20.0e-9f    /* Switch turn-off fall time (20 ns) */
#define VSI_DEADTIME_SEC        1.2e-6f     /* Inverter dead-time blanking interval (1.2 us) */
#define VSI_GATE_CHARGE_NC      28.0f       /* Total gate charge Qg (nC) */

/* ==========================================================================
 * Finite-Control-Set Model Predictive Control (FCS-MPC) Configuration
 * ========================================================================== */
#define MPC_TS                  50.0e-6f    /* Sampling & control prediction period (50 us -> 20 kHz) */
#define MPC_FS                  20000.0f    /* Control loop execution rate (20 kHz) */
#define MPC_PREDICTION_HORIZON  1           /* Horizon step (with 2-step ahead k+2 delay compensation) */
#define MPC_NUM_VECTORS         8           /* 8 discrete voltage vectors V0..V7 of 2L-VSI */

/* Multi-objective Cost Function Weights */
#define MPC_WEIGHT_IQ           1.0f        /* Weight for q-axis torque-producing current tracking */
#define MPC_WEIGHT_ID           1.0f        /* Weight for d-axis field-weakening / MTPA current tracking */
#define MPC_WEIGHT_SW           0.045f      /* Weight for switching commutation penalty (reduces f_sw & loss) */
#define MPC_WEIGHT_CMV          0.020f      /* Weight for Common-Mode Voltage (CMV) bearing mitigation */
#define MPC_WEIGHT_OVERCURRENT  1000.0f     /* Hard barrier penalty for currents exceeding limit */

/* Outer Speed Loop (Cascaded PI Loop for Reference Generation) */
#define SPEED_LOOP_TS           0.0010f     /* Speed loop sampling interval (1.0 ms -> 1 kHz) */
#define SPEED_KP                0.045f      /* Proportional gain for speed regulation */
#define SPEED_KI                0.80f       /* Integral gain for speed regulation */
#define SPEED_MAX_TORQUE_CURRENT 8.0f       /* Maximum q-axis current reference demand (Amperes) */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAMS_H */
