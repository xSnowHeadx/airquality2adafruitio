#pragma once

typedef struct
{
	double avg;
	uint16_t mcount;
} SensorStateStruct_t;

typedef struct
{
	SensorStateStruct_t pm25;
	SensorStateStruct_t humidity;
	SensorStateStruct_t temperature;
	SensorStateStruct_t pressure;
	SensorStateStruct_t co2;
} SensorState_t;

extern SensorState_t state;
