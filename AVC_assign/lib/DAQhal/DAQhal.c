//
// Created by chlch on 2023-10-17.
//

#include "DAQhal.h"
#include "string.h"

#define MAX_NAME_SIZE 10

static TaskHandle task_ao;
static TaskHandle task_ai;
static TaskHandle task_do;

static char device_name[MAX_NAME_SIZE] = "Dev1";

AnalogOut init_AO()
{
	return (AnalogOut) {0., 0.};
}

DigitalOut init_DO()
{
	return (DigitalOut) {0, 0};
}

void DAQ_task_init()
{
	// Set physical channel information
	char ai_port[16] = "";
	char ao_port[16] = "";
	char do_port[24] = "";
	
	strcat_s(ai_port, 16, device_name);
	strcat_s(ai_port, 16, "/ai0:3");
	strcat_s(ao_port, 16, device_name);
	strcat_s(ao_port, 16, "/ao0:1");
	strcat_s(do_port, 24, device_name);
	strcat_s(do_port, 24, "/port0/line0:1");
	
	// Create task
	DAQmxCreateTask("", &task_ai);
	DAQmxCreateTask("", &task_ao);
	DAQmxCreateTask("", &task_do);
	
	// Task setting
	DAQmxCreateAIVoltageChan(task_ai, ai_port, "", DAQmx_Val_Diff, -10., 10., DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(task_ao, ao_port, "", 0., 5., DAQmx_Val_Volts, "");
	DAQmxCreateDOChan(task_do, do_port, "", DAQmx_Val_ChanForAllLines);
	
	// Task Start
	DAQmxStartTask(task_ai);
	DAQmxStartTask(task_ao);
	DAQmxStartTask(task_do);
}

void set_DAQ_name(char* name)
{
	strcpy_s(device_name, MAX_NAME_SIZE, name);
}

AnalogIn read_analog()
{
	double v_in[4];
	DAQmxReadAnalogF64(task_ai, DAQmx_Val_Auto, 5., DAQmx_Val_GroupByChannel, v_in, 4, NULL, NULL);
	
	return (AnalogIn) {v_in[0], v_in[1], v_in[2], v_in[3]};
}

double read_analog_scalar()
{
	double v_in[4];
	DAQmxReadAnalogF64(task_ai, DAQmx_Val_Auto, 10., DAQmx_Val_GroupByChannel, v_in, 4, NULL, NULL);
	
	return v_in[0];
}

void write_analog(AnalogOut write_data)
{
	double v_cmd[2] = {write_data.ao0, write_data.ao1};
	DAQmxWriteAnalogF64(task_ao, 1, 1, 5., DAQmx_Val_GroupByChannel, v_cmd, NULL, NULL);
}

void write_analog_scalar(double write_data)
{
	double v_cmd[2] = {write_data, 0.};
	DAQmxWriteAnalogF64(task_ao, 1, 1, 5., DAQmx_Val_GroupByChannel, v_cmd, NULL, NULL);
}

void write_digital(DigitalOut write_data)
{
	uInt8 v_cmd[2] = {write_data.do0, write_data.do1};
	DAQmxWriteDigitalU8(task_do, 1, 1, 5., DAQmx_Val_GroupByChannel, v_cmd, NULL, NULL);
}

void write_digital_scalar(uInt8 write_data)
{
	uInt8 v_cmd[2] = {write_data, 0};
	DAQmxWriteDigitalLines(task_do, 1, 1, 5., DAQmx_Val_GroupByChannel, v_cmd, NULL, NULL);
}

void DAQ_task_stop()
{
	DAQmxStopTask(task_ai);
	DAQmxClearTask(task_ai);
	DAQmxStopTask(task_ao);
	DAQmxClearTask(task_ao);
	DAQmxStopTask(task_do);
	DAQmxClearTask(task_do);
}

void DAQmxCreateAO(TaskHandle task, char* port)
{
	DAQmxCreateAOVoltageChan(task, port, "", 0., 5.0, DAQmx_Val_Volts, "");
}

void DAQmxCreateAI(TaskHandle task, char* port)
{
	DAQmxCreateAIVoltageChan(task, port, "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, "");
}
