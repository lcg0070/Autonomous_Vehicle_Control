//
// Created by jarry_goon on 24. 5. 6.
//

#include <math.h>
#include <stdio.h>

#include "DAQhal.h"
#include <Windows.h>

#define FINAL_TIME      10000.                           // [ms]
#define SAMPLING_TIME   (1. / SAMPLING_FREQ * 1000.)    // [ms]
#define SAMPLING_FREQ   200.                            // [Hz]
#define N               (int)(FINAL_TIME / SAMPLING_TIME)

#define MAX_V           4.5             // [V]
#define MAX_W           144.0           // [deg/s]
#define MAPPING_SLOPE   (MAX_W / MAX_V) // [deg/sV]
#define MAPPING_DEAD    0.1             // [V]

#define CW_DEAD_V       0.7                                         // [V]
#define CW_MIN_W        0.0                                         // [deg/s]
#define CW_ORIGIN_SLOPE ((MAX_W - CW_MIN_W) / (MAX_V - CW_DEAD_V))  // [deg/sV]

#define CCW_DEAD_V  0.6                                                 // [V]
#define CCW_MIN_W   0.0                                                 // [deg/s]
#define CCW_ORIGIN_SLOPE ((MAX_W - CCW_MIN_W) / (MAX_V - CCW_DEAD_V))   // [deg/sV]

#define MAPPING_CW(V)   (double)(V > MAPPING_DEAD) * (MAPPING_SLOPE / CW_ORIGIN_SLOPE * V + CW_DEAD_V)      // [deg/sV]
#define MAPPING_CCW(V)  (double)(V < -MAPPING_DEAD) * (-MAPPING_SLOPE / CW_ORIGIN_SLOPE * V + CW_DEAD_V)    // [deg/sV]
#define MAPPING(V)      MAPPING_CW(V) + MAPPING_CCW(V)                                                      // [deg/sV]
#define DIRECTION(V)    (V < 0.)

#define PI 3.1415926535897932385
#define AMPLITUDE 1.0

double get_time()
{
    LARGE_INTEGER end_counter, frequency;

    QueryPerformanceCounter(&end_counter);
    QueryPerformanceFrequency(&frequency);

    return (double)end_counter.QuadPart / (double)frequency.QuadPart * 1000.0;
}

int main()
{
    char* file_name;

    double start_time;
    double currunt_time;
    double pre_time;

    double freq;
    double v_in;
    double v_out;
    double mapping_v;

    double buf_time[N]    = {0.,};
    double buf_v_in[N]    = {0.,};
    double buf_w_out[N]    = {0.,};
    double buf_v_out[N]   = {0.,};
    double buf_ang_in[N] = {0.,};

    int count;

    DAQ_task_init();

    for(int freq_int = 10; freq_int <= 10; freq_int++)
    {
        freq = 4.0;

        printf("Start (Frequency: %.2f [Hz])\n", freq);

        start_time = get_time();
        pre_time   = 0.;

        while(1)
        {
            currunt_time = get_time() - start_time;

            if(currunt_time >= 3000.) break;
        }

        while(1)
        {
            while(1)
            {
                currunt_time = get_time() - start_time;

                if(currunt_time - pre_time >= SAMPLING_TIME) break;
            }

            pre_time     = currunt_time;

            v_in = read_analog_scalar();
            write_digital_scalar(DIRECTION(2.5 - v_in));
            write_analog_scalar(MAPPING(2.0 * (2.5 - v_in)));

            printf("Potential: %f [V]\n", v_in);

            if(fabs(2.5 - v_in) < 0.1)
            {
                write_analog_scalar(0.0);
                break;
            }
        }

        write_analog_scalar(0.0);

        start_time = get_time();

        while(1)
        {
            currunt_time = get_time() - start_time;

            if(currunt_time >= 3000.) break;
        }

        printf("Initialize Finish\n\n");

        start_time = get_time();
        pre_time   = 0.;

        count = 0;

        do
        {
            while(1)
            {
                currunt_time = get_time() - start_time;

                if(currunt_time - pre_time >= SAMPLING_TIME) break;
            }

            pre_time     = currunt_time;
            currunt_time = count * SAMPLING_TIME;

            v_out = AMPLITUDE * sin(2 * PI * freq * currunt_time / 1000.);

            mapping_v = MAPPING(v_out);

            write_analog_scalar(mapping_v);
            write_digital_scalar(DIRECTION(v_out));

            v_in = read_analog_scalar();

            buf_time[count] = currunt_time;
            buf_v_in[count] = v_in;
            buf_ang_in[count] = (v_in - 2.5) * 2.0 * 360.0 * 0.2 * 0.5;
            buf_v_out[count] = v_out;
            buf_w_out[count] = v_out * 144.0 / 4.5;
        } while(++count < N);

        write_analog_scalar(0.0);

        file_name = "";

        FILE* file;
        sprintf_s(file_name, 21, "Dynamic_%3.1f.txt", freq);
        fopen_s(&file, file_name, "w+t");

        for(int i = 0; i < N; i++)
            fprintf_s(file, "%f\t%f\t%f\t%f\t%f\n",
                buf_time[i], buf_v_in[i], buf_ang_in[i], buf_v_out[i], buf_w_out[i]);

        fclose(file);
    }

    DAQ_task_stop();
}
