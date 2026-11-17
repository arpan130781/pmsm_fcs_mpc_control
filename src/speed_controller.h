#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "motor_params.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float integrator;
    float out_max;
    float out_min;
    float filtered_speed_rpm;
} SpeedController_t;

void speed_controller_init(SpeedController_t* sc);
void speed_controller_reset(SpeedController_t* sc);
float speed_controller_step(SpeedController_t* sc, float speed_ref_rpm, float speed_feedback_rpm, float dt);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_CONTROLLER_H */
