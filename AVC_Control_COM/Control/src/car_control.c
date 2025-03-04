//
// Created by jarry_goon on 24. 5. 22.
//

#include "car_control.h"
#include "DAQHal.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define VOLTAGE2ANGLE(V) (((V) - POTENTIAL_OPERATE_V) * POTENTIAL_V2ANGLE)

#define CW_MIN_V 0.9                                               // [V]
#define CW_MIN_W 0.0                                               // [deg/s]
#define CW_MAX_V 4.0                                               // [V]
#define CW_MAX_W 71.6865                                           // [deg/s]
#define CW_SLOPE ((CW_MAX_W - CW_MIN_W) / (CW_MAX_V - CW_MIN_V))   // [deg/sV]

#define CCW_MIN_V 0.8                                                    // [V]
#define CCW_MIN_W 0.0                                                    // [deg/s]
#define CCW_MAX_V 4.0                                                    // [V]
#define CCW_MAX_W 72.101                                                 // [deg/s]
#define CCW_SLOPE ((CCW_MAX_W - CCW_MIN_W) / (CCW_MAX_V - CCW_MIN_V))    // [deg/sV]

#define MAX_V           4.0             // [V]      Maximum voltage
#define MAX_W           71.6865         // [deg/s]  Maximum angular velocity
#define MAPPING_SLOPE   (MAX_W / MAX_V) // [deg/sV]
#define MAPPING_DEAD    0.02            // [V]

#define CW_MAPPING(V)  (((V) >= MAPPING_DEAD) * ((MAPPING_SLOPE * (V) - CW_MIN_W) / CW_SLOPE + CW_MIN_V))
#define CCW_MAPPING(V) (((V) >= MAPPING_DEAD) * ((MAPPING_SLOPE * (V) - CCW_MIN_V) / CCW_SLOPE + CCW_MIN_V))
#define MAPPING(V)     (((V) >= 0.0) * CW_MAPPING(V) + ((V) < 0.0) * CCW_MAPPING(-V))
#define DIRECTION(V)   (char)((V) < 0.0)

#define MAX_STEER 20.0
#define MAX_V_CMD 4.0

#define SPEED2VOLTAGE(X) ((double)(X > 0.3) * (0.5891 * (X) + 1.30) + (double)(X <= 0.3) * 1.0)
#define MAX_SPEED 4.5

CarState CAR_STATE;

typedef enum {
    // Analog Output
    STEER = 0,
    SPEED,

    // Analog Input
    POTENTIO = 0,

    //Digital Output
    STEER_DIR = 0,
    SPEED_DIR,
} DAQPort;

void init_car_state() {
    double potential_voltage;
    int count = 0;

    DAQ_task_init();

    memset(&CAR_STATE, 0, sizeof(CAR_STATE));
    CAR_STATE.drive_mode = MANUAL_MODE;
    CAR_STATE.running = 1;

    set_speed();

    do {
        set_steering();
        potential_voltage = read_analog_scalar(POTENTIO) - POTENTIAL_OPERATE_V;

        if (potential_voltage < 0.1 && potential_voltage > -0.1)
            count++;
        else
            count = 0;
    } while (count < 500);

    printf("Car Initialize Finish\n");
}

void clear_car_state() {
    CAR_STATE.speed = 0.0;

    set_speed();

    DAQ_task_stop();
}

void set_steering() {
    double potential_voltage;
    double potentail_angle;

    double angle_err;
    static double prev_angle_err = 0.0;
    double P_err;
    static double I_err = 0.0;

    double w_cmd;
    double v_cmd;

    if (CAR_STATE.steer > MAX_STEER) CAR_STATE.steer = MAX_STEER;
    else if (CAR_STATE.steer < -MAX_STEER) CAR_STATE.steer = -MAX_STEER;


    // 1. Get currunt steer angle[deg]
    potential_voltage = read_analog_scalar(POTENTIO);
    potentail_angle = VOLTAGE2ANGLE(potential_voltage);

    // 2. Compute angle error and angular velocity command
    angle_err = CAR_STATE.steer - potentail_angle;
    P_err = P * angle_err;
    I_err += I_controller(angle_err, prev_angle_err);

    // 2-1. Saturation integration error
    if(I_err > MAX_W) I_err = MAX_W;
    else if(I_err < -MAX_W) I_err = -MAX_W;

    w_cmd = P_err + I_err;

    // 3. Convert angular velocity command to voltage
    v_cmd = w_cmd / MAPPING_SLOPE;

    // 4. Saturation voltage
    if (v_cmd > MAX_V_CMD) v_cmd = MAX_V_CMD;
    else if (v_cmd < -MAX_V_CMD) v_cmd = -MAX_V_CMD;

    // 5. Command to DAQ
    write_analog_scalar(MAPPING(v_cmd), STEER);
    write_digital_scalar(DIRECTION(v_cmd), STEER_DIR);

    // 6. Data update
    CAR_STATE.current_steer = potentail_angle;
    prev_angle_err = angle_err;
}

void set_speed() {
    double v_cmd;
    int direction;

    if (CAR_STATE.speed > MAX_SPEED) CAR_STATE.speed = MAX_SPEED;
    else if (CAR_STATE.speed < -MAX_SPEED) CAR_STATE.speed = -MAX_SPEED;

    v_cmd = SPEED2VOLTAGE(fabs(CAR_STATE.speed));
    if(CAR_STATE.speed < 0.0) direction = 1;
    else direction = 0;

    write_digital_scalar(direction, SPEED_DIR);
    write_analog_scalar(v_cmd, SPEED);
}