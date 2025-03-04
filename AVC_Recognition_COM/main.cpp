// #include "communication.h"
#include "calibration.hpp"
#include "LIDAR_processing.h"
#include "OGM_processing.hpp"

#include <conio.h>

int main(void){

    init_lidar();
    init_camera();
    init_OGM();

    do{
        camera_processing();
        if(lidar_preprocessing()) continue;

        OGM_calculate();
        if(_kbhit())
            break;

    } while(true);

    clear_OGM();
    clear_camera();
    Lidar_Terminate();

    return 0;
}
