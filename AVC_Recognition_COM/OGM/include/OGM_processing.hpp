//
// Created by User on 2024-06-29.
//

#ifndef AVC_RECOGNITION_COM_OGM_PROCESSING_H
#define AVC_RECOGNITION_COM_OGM_PROCESSING_H

#define GRID_SIZE 0.3
#define GRIDMAP_WIDTH 100
#define GRIDMAP_HEIGHT 50

#define SEMIMAJORAXIS 6378137.0 // [m] a
#define SEMIMINORAXIS 6.356752314245179e+06 // [m] b
#define FLATTENING 0.003352810664747
#define HGT 48.7 // hgt

// buffer
#define INTERESTED_INDEX            ((int)561)

void init_OGM();
void OGM_calculate();

void ned2earth(const double N_frame[2], double E_frame[3], double latN, double lonN);
void earth2inertial(const double E_frame[3], double I_frame[3]);

void update_ogm(int x, int y, const char flag);

void clear_OGM();

#endif //AVC_RECOGNITION_COM_OGM_PROCESSING_H

