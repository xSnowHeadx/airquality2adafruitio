#pragma once

#include <SoftwareSerial.h>

#include "Types.h"

namespace SerialCom
{
constexpr static const uint8_t PIN_UART_RX = 13; // D2 on Wemos D1 Mini
constexpr static const uint8_t PIN_UART_TX = 10; // UNUSED

SoftwareSerial sensorSerial(PIN_UART_RX, PIN_UART_TX);

uint8_t serialRxBuf[255];
uint8_t rxBufIdx = 0;

void setup()
{
	sensorSerial.begin(9600);
}

void clearRxBuf()
{
	// Clear everything for the next message
	memset(serialRxBuf, 0, sizeof(serialRxBuf));
	rxBufIdx = 0;
}

bool isValidHeader()
{
	bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

	if (!headerValid)
	{
		Serial.println("Received message with invalid header.");
	}

	return headerValid;
}

bool isValidChecksum()
{
	uint8_t checksum = 0;

	for (uint8_t i = 0; i < 20; i++)
	{
		checksum += serialRxBuf[i];
	}

	if (checksum != 0)
	{
		Serial.printf("Received message with invalid checksum. Expected: 0. Actual: %d\n", checksum);
	}

	return checksum == 0;
}

void handleUart(SensorState_t *state)
{
	if (!sensorSerial.available())
	{
		return;
	}

	Serial.print("Receiving:");
	while (sensorSerial.available())
	{
		serialRxBuf[rxBufIdx++] = sensorSerial.read();
		Serial.print(".");

		// Without this delay, receiving data breaks for reasons that are beyond me
		delay(15);

		if (rxBufIdx >= 64)
		{
			clearRxBuf();
		}
	}
	Serial.println("Done.");

	if (isValidHeader() && isValidChecksum())
	{
		uint16_t mpm25 = (serialRxBuf[5] << 8) | serialRxBuf[6];

		Serial.print("received PM25: ");
		Serial.println(mpm25);
		state->pm25.avg += mpm25;
		state->pm25.mcount++;
	}
	else
	{
		clearRxBuf();
	}
}
} // namespace SerialCom
