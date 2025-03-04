//
// Created by jarry_goon on 24. 5. 29.
//

#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#define N_BIAS (0.0)
#define E_BIAS (0.0)

typedef struct
{
    double dot_gamma;
    double delta_w;
    double distance;
    double N;
    double E;
    double yaw;
    double dst_N;
    double dst_E;
} PathPlanningInfo;

extern PathPlanningInfo GUIDANCE_INFO;

void init_path_planning();

void guidance();

void clear_guidance();

#endif //PATH_PLANNING_H
