#include "speed_controller.h"
#include <math.h>

void speed_controller_init(SpeedController_t* sc) {
    if (!sc) return;
    sc->kp = SPEED_KP;
    sc->ki = SPEED_KI;
    sc->integrator = 0.0f;
    sc->out_max = SPEED_MAX_TORQUE_CURRENT;
    sc->out_min = -SPEED_MAX_TORQUE_CURRENT;
    sc->filtered_speed_rpm = 0.0f;
}

void speed_controller_reset(SpeedController_t* sc) {
    if (!sc) return;
    sc->integrator = 0.0f;
    sc->filtered_speed_rpm = 0.0f;
}

float speed_controller_step(SpeedController_t* sc, float speed_ref_rpm, float speed_feedback_rpm, float dt) {
    if (!sc || dt <= 0.0f) return 0.0f;
    
    /* 1st order low-pass filter on speed feedback to reduce encoder discretization jitter */
    float alpha = 0.85f;
    sc->filtered_speed_rpm = alpha * sc->filtered_speed_rpm + (1.0f - alpha) * speed_feedback_rpm;
    
    float error = speed_ref_rpm - sc->filtered_speed_rpm;
    
    /* Proportional term */
    float p_term = sc->kp * error;
    
    /* Integral term with conditional anti-windup clamping */
    float next_integrator = sc->integrator + (sc->ki * error * dt);
    
    float unsaturated_output = p_term + next_integrator;
    float output = unsaturated_output;
    
    if (output > sc->out_max) {
        output = sc->out_max;
        /* Clamp integrator to prevent windup */
        if (error < 0.0f) {
            sc->integrator = next_integrator;
        }
    } else if (output < sc->out_min) {
        output = sc->out_min;
        if (error > 0.0f) {
            sc->integrator = next_integrator;
        }
    } else {
        sc->integrator = next_integrator;
    }
    
    return output;
}
