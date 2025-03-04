#ifndef DAQHAL_H
#define DAQHAL_H

#ifdef __cplusplus
extern "C" {
#endif

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
	unsigned char do0;
	unsigned char do1;
} DigitalOut;

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
void set_DAQ_name(const char* name);

AnalogIn read_analog();

double read_analog_scalar(int port);

void write_analog(AnalogOut write_data);

void write_analog_scalar(double write_data, int port);

void write_digital(DigitalOut write_data);

void write_digital_scalar(unsigned char write_data, int port);

void DAQ_task_stop();

#ifdef __cplusplus
}
#endif

#endif //DAQHAL_H
