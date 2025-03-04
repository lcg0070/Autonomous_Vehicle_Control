//
// Created by jarry_goon on 24. 6. 10.
//

#ifndef LOCAL_PATH_PLANNING_H
#define LOCAL_PATH_PLANNING_H

#define NUM_RANGE 561
#define ref_angle 35

void init_local_planning();

void local_planning(double N, double E, double yaw);

void clear_local_planning();

#endif //LOCAL_PATH_PLANNING_H
