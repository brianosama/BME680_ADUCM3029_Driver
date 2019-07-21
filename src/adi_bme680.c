/*****************************************************************************
 * adi_bme680.c
 *
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

#include "adi_bme680.h"

#include <bme680.h>
#include <drivers/spi/adi_spi.h>

static int8_t spi_read_func(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
		uint16_t len);
static int8_t spi_write_func(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
		uint16_t len);

struct bme680_dev gas_sensor;

extern ADI_SPI_HANDLE spi_handle;
extern void delay_ms(uint32_t ms);

bool ADI_BME680_Init(uint8_t addr) {

#ifdef ADI_BME680_SPI_INTF
	gas_sensor.intf = BME680_SPI_INTF;
	gas_sensor.write = &spi_write_func;
	gas_sensor.read = &spi_read_func;
	gas_sensor.dev_id = addr;
#else
	gas_sensor.intf = BME680_I2C_INTF;
#endif //ADI_BME680_SPI_INTF

	gas_sensor.delay_ms = &delay_ms;

	if (bme680_init(&gas_sensor) != BME680_OK)
		return false;

	gas_sensor.tph_sett.os_hum = BME680_OS_2X;
	gas_sensor.tph_sett.os_pres = BME680_OS_4X;
	gas_sensor.tph_sett.os_temp = BME680_OS_8X;
	gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

	gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	gas_sensor.gas_sett.heatr_temp = 350;
	gas_sensor.gas_sett.heatr_dur = 150;

	gas_sensor.power_mode = BME680_SLEEP_MODE;

	/* Set the power mode */
	bme680_set_sensor_mode(&gas_sensor);

	return true;
}

void ADI_BME680_ReadSensorData(ADI_BME680_data *data) {
	gas_sensor.power_mode = BME680_FORCED_MODE;
	bme680_set_sensor_mode(&gas_sensor);

	uint8_t set_required_settings = BME680_OST_SEL | BME680_OSP_SEL
			| BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

	/* Set the desired sensor configuration */
	bme680_set_sensor_settings(set_required_settings, &gas_sensor);

	/* get wait time of sensor readings */
	uint16_t meas_period;
	bme680_get_profile_dur(&meas_period, &gas_sensor);

	delay_ms(meas_period);

	struct bme680_field_data _data;
	bme680_get_sensor_data(&_data, &gas_sensor);

	data->temperature = (float) _data.temperature / 100.0f;
	data->humidity = (float) _data.humidity / 1000.0f;
	data->pressure = (float) _data.pressure / 100.0f;

	if (_data.status & BME680_HEAT_STAB_MSK)
		data->gas_resistance = _data.gas_resistance;
	else
		data->gas_resistance = 0;

	gas_sensor.power_mode = BME680_SLEEP_MODE;
	bme680_set_sensor_mode(&gas_sensor);
}

/* static functions */

int8_t spi_read_func(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len) {
	ADI_SPI_TRANSCEIVER txvr_def;

	txvr_def.pTransmitter = &reg_addr;
	txvr_def.nTxIncrement = 1;
	txvr_def.TransmitterBytes = 1;

	txvr_def.pReceiver = reg_data;
	txvr_def.ReceiverBytes = len;
	txvr_def.nRxIncrement = 1;

	txvr_def.bDMA = false;
	txvr_def.bRD_CTL = true;

	adi_spi_SetChipSelect(spi_handle, gas_sensor.dev_id);
	int8_t res = adi_spi_MasterReadWrite(spi_handle, &txvr_def);
	adi_spi_SetChipSelect(spi_handle, ADI_SPI_CS_NONE);

	return res;
}

int8_t spi_write_func(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {
	ADI_SPI_TRANSCEIVER txvr_def;

	uint8_t txd[len + 1];

	txd[0] = reg_addr;

	for (uint8_t x = 1; x < len + 1; x++) {
		txd[x] = data[x - 1];
	}

	txvr_def.pTransmitter = txd;
	txvr_def.nTxIncrement = 1;
	txvr_def.TransmitterBytes = len + 1;

	txvr_def.pReceiver = NULL;
	txvr_def.ReceiverBytes = 0;
	txvr_def.nRxIncrement = 1;

	txvr_def.bDMA = true;
	txvr_def.bRD_CTL = false;

	adi_spi_SetChipSelect(spi_handle, gas_sensor.dev_id);
	int8_t res = adi_spi_MasterReadWrite(spi_handle, &txvr_def);
	adi_spi_SetChipSelect(spi_handle, ADI_SPI_CS_NONE);

	return res;
}

