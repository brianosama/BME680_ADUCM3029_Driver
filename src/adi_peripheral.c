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

#include "BME680_Sensor.h"

#include <drivers/pwr/adi_pwr.h>
#include <drivers/spi/adi_spi.h>
#include <drivers/tmr/adi_tmr.h>

#include <common.h>

#define SPI_NUM 0								// SPI0 def

#define GP2_TIMER_LOAD_VAL 1625					// for 1ms delay timing

ADI_SPI_HANDLE spi_handle;
uint8_t spi_device_mem[ADI_SPI_MEMORY_SIZE];

static ADI_TMR_CONFIG tmrConfig;
static bool timeout = false;
static uint32_t time_inc = 0;

void GP2CallbackFunction(void *pCBParam, uint32_t Event, void * pArg) {
	/* IF(Interrupt occurred because of a timeout) */
	if ((Event & ADI_TMR_EVENT_TIMEOUT) == ADI_TMR_EVENT_TIMEOUT) {
		time_inc++;
	} /* ENDIF */
}

uint8_t init_peripherals() {
	/* initialize power and clock settings */
	adi_pwr_Init();
	adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1);
	adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1);

	/* initialize spi0 */
	adi_spi_Open(SPI_NUM, spi_device_mem, ADI_SPI_MEMORY_SIZE, &spi_handle);
	adi_spi_SetBitrate(spi_handle, 1000000);
	adi_spi_SetIrqmode(spi_handle, 0u);

	adi_tmr_Init(ADI_TMR_DEVICE_GP2, GP2CallbackFunction, NULL, true);
	tmrConfig.bCountingUp = false;
	tmrConfig.bPeriodic = true;
	tmrConfig.ePrescaler = ADI_TMR_PRESCALER_16;
	tmrConfig.eClockSource = ADI_TMR_CLOCK_HFOSC;
	tmrConfig.bReloading = true;
	tmrConfig.bSyncBypass = false;
	tmrConfig.nLoad = GP2_TIMER_LOAD_VAL;
	tmrConfig.nAsyncLoad = GP2_TIMER_LOAD_VAL;

	adi_tmr_ConfigTimer(ADI_TMR_DEVICE_GP2, &tmrConfig);

	return 0;
}

void delay_ms(uint32_t ms) {
	timeout = false;
	time_inc = 0;

	adi_tmr_Enable(ADI_TMR_DEVICE_GP2, true);

	do {
		__WFI();
	} while(time_inc < ms);

	adi_tmr_Enable(ADI_TMR_DEVICE_GP2, false);
}
