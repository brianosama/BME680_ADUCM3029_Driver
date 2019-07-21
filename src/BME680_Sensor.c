/*****************************************************************************
 * BME680_Sensor.c
 * Copyright (c) 2019 Brian Osama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *****************************************************************************/

#include <sys/platform.h>
#include "adi_initialize.h"

#include "BME680_Sensor.h"
#include "adi_bme680.h"

#include <common.h>

#include <drivers/spi/adi_spi.h>

int main(int argc, char *argv[]) {
	ADI_BME680_data data;

	adi_initComponents();
	init_peripherals();

	DEBUG_MESSAGE("test");

	uint8_t addr = ADI_SPI_CS1;
	ADI_BME680_Init(addr);

	while (1) {
		delay_ms(1000);
		ADI_BME680_ReadSensorData(&data);

		DEBUG_MESSAGE("T: %.2f degC, P: %.2f hPa, H: %.2f %%rH, G %lu Ohms",
				data.temperature, data.pressure, data.humidity, data.gas_resistance);
	}

	return 0;
}

