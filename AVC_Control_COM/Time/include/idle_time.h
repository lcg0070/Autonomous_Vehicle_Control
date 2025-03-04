//
// Created by jarry_goon on 24. 5. 26.
//

#ifndef IDLE_TIME_H
#define IDLE_TIME_H

#define SAMPLING_FREQ 100.0
#define SAMPLING_TIME (1 / SAMPLING_FREQ)

typedef struct
{
    double       start;    // [s] Start time
    double       elapsed;  // [s] Time elapsed since start time
    double       sampling; // [s] Sampling time
} Time;

extern Time TIME;

void init_time();

double get_time();

void idle_time();

#endif //IDLE_TIME_H
