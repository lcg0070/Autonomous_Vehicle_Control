//
// Created by jarry_goon on 24. 5. 26.
//

#include "keyboard_input.h"
#include "car_control.h"
#include "idle_time.h"
#include "path_planning.h"
#include "GPSINS.h"
#include "logger.h"

#include <stdio.h>

int main()
{
    init_car_state();
    init_GPSINS();
    init_path_planning();
    init_logger();
    init_time();

    Sleep(5000);
    system("cls");

    while(CAR_STATE.running)
    {

        keyboard_input();

        set_speed();
        set_steering();

        guidance();

        logger();
        idle_time();
    }

    clear_car_state();
    clear_guidance();
    clear_GPSINS();
    clear_logger();

    system("pause");
}
