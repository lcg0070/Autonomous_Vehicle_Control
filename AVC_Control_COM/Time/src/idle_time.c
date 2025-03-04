//
// Created by jarry_goon on 24. 5. 26.
//

#include "idle_time.h"

#include <Windows.h>

Time TIME;

void init_time()
{
    TIME.start    = get_time();
    TIME.elapsed  = 0.0;
    TIME.sampling = SAMPLING_TIME;
}

double get_time()
{
    LARGE_INTEGER end_counter, frequency;

    QueryPerformanceCounter(&end_counter);
    QueryPerformanceFrequency(&frequency);

    return (double)end_counter.QuadPart / (double)frequency.QuadPart;
}

void idle_time()
{
    double current_time;

    do
    {
        current_time = get_time() - TIME.start;
    } while(current_time - TIME.elapsed < TIME.sampling);

    TIME.elapsed += TIME.sampling;
}
