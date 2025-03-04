#include "Function.h"

/*-----------------------------------------------------------------------------------*/
/* Get Window Time (returns [msec])                                                  */
/*-----------------------------------------------------------------------------------*/

/**
 * @brief Get Current OS time
 * @return Current OS time[s]
 */
double GetWindowTime(void)
{
    LARGE_INTEGER liEndCounter, liFrequency;

    QueryPerformanceCounter(&liEndCounter);
    QueryPerformanceFrequency(&liFrequency);

    return (REAL)liEndCounter.QuadPart / (REAL)liFrequency.QuadPart;
}
