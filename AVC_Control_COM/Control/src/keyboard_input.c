#include "keyboard_input.h"
#include "car_control.h"

#include <conio.h>
#include <math.h>

#define SPEED_LEVEL 0.1
#define STEER_LEVEL 2.5

void keyboard_input()
{
    int input;

    // If no keyboard input is received, the function terminates.
    if(!_kbhit()) return;

    input = _getch();

    // Driving mode change: 0(MANUAL), 1(AUTO)
    if(input == 'm' || input == 'M')
    {
        CAR_STATE.drive_mode = (char)!CAR_STATE.drive_mode;
        CAR_STATE.speed = 0.0;
    }

    // If it auto mode, it processes not keyboard input.
    if(CAR_STATE.drive_mode == AUTO_MODE) return;

    switch(input)
    {
        case ' ':
            CAR_STATE.speed = 0.0;
            break;

        case 'W':
        case 'w':
            CAR_STATE.speed += SPEED_LEVEL;
            break;

        case 'S':
        case 's':
            CAR_STATE.speed -= SPEED_LEVEL;
            break;

        case 'A':
        case 'a':
            CAR_STATE.steer -= STEER_LEVEL;
            break;

        case 'D':
        case 'd':
            CAR_STATE.steer += STEER_LEVEL;
            break;

        case 27:
            CAR_STATE.running = 0;

        default:
            break;
    }
}
