#ifndef DAQHAL_H
#define DAQHAL_H

#include "NIDAQmx.h"

typedef struct
{
	double ai0;
	double ai1;
	double ai2;
	double ai3;
} AnalogIn;

typedef struct
{
	double ao0;
	double ao1;
} AnalogOut;

typedef struct
{
	uInt8 do0;
	uInt8 do1;
} DigitalOut;

AnalogOut init_AO();

DigitalOut init_DO();

/**
 * Initialize and setting IO task.
 *
 * @note
 * Check your DAQ device name. If your DAQ device name is not "Dev1", use set_DAQ_name before using this.
 */
void DAQ_task_init();

/**
 * Setting DAQ device name.
 * @param name: Device name
 */
void set_DAQ_name(char* name);

AnalogIn read_analog();

double read_analog_scalar();

void write_analog(AnalogOut write_data);

void write_analog_scalar(double write_data);

void write_digital(DigitalOut write_data);

void write_digital_scalar(uInt8 write_data);

void DAQ_task_stop();

void DAQmxCreateAO(TaskHandle task, char* port);

void DAQmxCreateAI(TaskHandle task, char* port);

#endif //DCSP_DAQHAL_H
