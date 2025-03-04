//
// Created by jarry_goon on 24. 5. 22.
//

#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
export "C" {
#endif

#include "idle_time.h"

#define P 3.0
#define I 2.0

#define POTENTIAL_OPERATE_V 2.5
#define POTENTIAL_V2ANGLE   (2.0 * 360.0 * 0.2 * 35.0 / 144.0)

#define MANUAL_MODE (char)0
#define AUTO_MODE   (char)1

#define SQUARE(X) ((X) * (X))
/*=================================================== Extern Value ===================================================*/

typedef struct
{
    double speed;
    double steer;
    double current_steer;
    char   running;
    char   drive_mode;
} CarState;

extern CarState CAR_STATE;

/*================================================== Define function =================================================*/

void init_car_state();

void clear_car_state();

void set_steering();

void set_speed();
//수정
void default_set_steering(void);
/*============================================== Define inline function ==============================================*/

inline double I_controller(const double error, const double prev_error)
{
    return I * TIME.sampling * (error + prev_error) * 0.5;
}

#ifdef __cplusplus
}
#endif

#endif //CONTROL_H
