//
// Created by jarry_goon on 24. 5. 28.
//

#ifndef GLOBAL_PATH_PLANNING_H
#define GLOBAL_PATH_PLANNING_H

// calculation
#define RAD2DEG (180.0 / M_PI)    // [rad/deg]
#define DEG2RAD (M_PI / 180.0)    // [deg/rad]

// const
#define CONTROL_GAIN        2.5
#define DISTANCE_THRESHOLD  1.0 // [m]


// VEHICLE
#define VEHICLE_WHEELBASE   0.729322    // [m]


#define PYTHAGORAS(X, Y) (sqrt((X) * (X) + (Y) * (Y)))

//
#define SEMIMAJOR_AXIS 6378137.0 // [m] a
#define SEMIMINOR_AXIS 6.356752314245179e+06 // [m] b
#define FLATTENING 0.003352810664747

//
#define LAT0 36.1034237805919
#define LON0 129.389642843629
#define ALT0 48.7

#define MIN_DISTANCE 7.0
#define MAX_DISTANCE 15.0

/*=============================================== Define Function ====================================================*/

void init_global_planning();

void clear_global_guidance();

void global_guidance(double N, double E);

void geodetic2ned(const double geo_pos[2], double ned[2]);

#endif //GLOBAL_PATH_PLANNING_H
