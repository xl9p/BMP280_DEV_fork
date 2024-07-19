/*
  BMP280_DEV is an I2C/SPI compatible library for the Bosch BMP280 barometer.
	
	Copyright (C) Martin Lindupp 2019
	
	V1.0.0 -- Initial release 	
	V1.0.1 -- Added ESP32 HSPI support and change library to unique name
	V1.0.2 -- Modification to allow external creation of HSPI object on ESP32
	V1.0.3 -- Changed library name in the library.properties file
	V1.0.5 -- Fixed bug in BMP280_DEV::getTemperature() function, thanks to Jon M.
	V1.0.6 -- Merged multiple instances and initialisation pull requests by sensslen
	V1.0.8 -- Used default arguments for begin() member function and 
						added example using multiple BMP280 devices with SPI comms in NORMAL mode
	V1.0.9 -- Moved writeMask to Device class and improved measurement detection code
	V1.0.10 -- Modification to allow user-defined pins for I2C operation on the ESP8266
	V1.0.12 -- Allow sea level pressure calibration using setSeaLevelPressure() function
	V1.0.14 -- Fix uninitialised structures, thanks to David Jade investigating and 
						 flagging up this issue
	V1.0.16 -- Modification to allow user-defined pins for I2C operation on the ESP32
	V1.0.17 -- Added getCurrentTemperature(), getCurrentPressure(), getCurrentTempPres() 
						 getCurrentAltitude() and getCurrentMeasurements() functions,
						 to allow the BMP280 to be read directly without checking the measuring bit
	V1.0.18 -- Initialise "device" constructor member variables in the same order they are declared
	V1.0.19 -- Allow for additional TwoWire instances
	V1.0.20 -- Removed default parameter causing ESP32 compilation error with user defined I2C pins
	V1.0.21 -- Fixed uninitialised "Wire" pointer for ESP8266/ESP32 with user defined I2C pins 
	
	The MIT License (MIT)
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
*/

#include <BMP280_DEV.h>


BMP280_DEV::BMP280_DEV(TwoWire* twoWire, uint8_t addr) {
	this->i2cDevice = std::make_unique<Adafruit_I2CDevice>(addr, twoWire);

	this->deviceIDReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::DEVICE_ID, 1U, MSBFIRST);
	this->pressMSBReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::PRES_MSB, 3U, MSBFIRST);
	this->tempMSBReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::TEMP_MSB, 3U, MSBFIRST);
	this->trimParamsStartReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::TRIM_PARAMS, 24U, LSBFIRST);
	this->resetReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::RESET, 1U, MSBFIRST);
	this->statusReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::STATUS, 1U, MSBFIRST);
	this->ctrlMeasReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::CTRL_MEAS, 1U, MSBFIRST);
	this->configReg.emplace(this->i2cDevice.get(), this->spiDevice.get(), ADDRBIT8_HIGH_TOREAD, (uint8_t)BMP280_REGISTERS::CONFIG, 1U, MSBFIRST);

	this->status_IMUPDATE_bits.emplace(&this->statusReg.value(), 1U,(uint8_t)BMP280_STATUS_OFFSETS::IM_UPDATE);
	this->status_MEASURING_bits.emplace(&this->statusReg.value(), 1U, (uint8_t)BMP280_STATUS_OFFSETS::MEASURING);

	this->ctrlmeas_MODE_bits.emplace(&this->ctrlMeasReg.value(), 2U, (uint8_t)BMP280_CTRL_MEAS_OFFSETS::MODE);
}


BMP280SensorOperation BMP280_DEV::begin(const BMP280_Config &sensorConfig) {
	if (!this->isInitialized) {
		if (!this->i2cDevice.get()->begin()) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		
		if (this->deviceIDReg.value().read() != (uint8_t)BMP280_DEFINITIONS::DEVICE_ID) {
			return BMP280SensorOperation(SENSOR_STATUS_BAD_ADDR);
		}

		if (!this->trimParamsStartReg.value().read((uint8_t*)&this->trimParams, sizeof(this->trimParams))) {
			return BMP280SensorOperation(SENSOR_STATUS_CALIB_GET_FAIL);
		}

		bool configFilterResult = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U,
								(uint8_t) BMP280_CONFIG_OFFSETS::FILTER).write((uint8_t)sensorConfig.iirFilter);
		bool configTSBResult = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U,
								(uint8_t) BMP280_CONFIG_OFFSETS::T_SB).write((uint8_t)sensorConfig.timeStandby);
		bool ctrlmeasOSRSPResult = Adafruit_BusIO_RegisterBits(&this->ctrlMeasReg.value(), 3U,
								(uint8_t) BMP280_CTRL_MEAS_OFFSETS::OSRS_P).write((uint8_t)sensorConfig.pressureOversampling);
		bool ctrlmeasOSRSTResult = Adafruit_BusIO_RegisterBits(&this->ctrlMeasReg.value(), 3U,
								(uint8_t) BMP280_CTRL_MEAS_OFFSETS::OSRS_T).write((uint8_t)sensorConfig.temperatureOversampling);

		if (!(configFilterResult && configTSBResult && ctrlmeasOSRSPResult && ctrlmeasOSRSTResult)) {
			return BMP280SensorOperation(SENSOR_STATUS_CONFIG_SET_FAIL);
		}

		this->configuration = sensorConfig;
		this->isInitialized = true;
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
		// switch(this->reset().sensorStatus) {
		// 	case SENSOR_STATUS_OP_OK: {
				
		// 	}
		// 	default: {
		// 		this->isInitialized = false;
		// 		return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		// 	}
		// }
	}
	return BMP280SensorOperation(SENSOR_STATUS_ALREADY_INIT);
}


BMP280SensorOperation BMP280_DEV::reset(void) {
	if (this->isInitialized) {
		if (!this->resetReg.value().write((uint8_t)BMP280_DEFINITIONS::RESET_CODE)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		delay(5);
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}


BMP280SensorOperation BMP280_DEV::startNormalConversion(void) {
	if (this->isInitialized) {
		if (!this->ctrlmeas_MODE_bits.value().write((uint8_t)BMP280_MeasurementMode::NORMAL)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		this->configuration.mode = BMP280_MeasurementMode::NORMAL;
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}


BMP280SensorOperation BMP280_DEV::startForcedConversion(void) {
	if (this->isInitialized) {
		this->configuration.mode = (BMP280_MeasurementMode) this->ctrlmeas_MODE_bits.value().read();
		// if (this->configuration.mode == BMP280_MeasurementMode::SLEEP) {
		if (!this->ctrlmeas_MODE_bits.value().write((uint8_t)BMP280_MeasurementMode::FORCED)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		this->configuration.mode = BMP280_MeasurementMode::FORCED;
		// }
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}


BMP280SensorOperation BMP280_DEV::stopConversion(void) {
	if (this->isInitialized) {
		if (!this->ctrlmeas_MODE_bits.value().write((uint8_t)BMP280_MeasurementMode::SLEEP)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		this->configuration.mode = BMP280_MeasurementMode::SLEEP;
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}


BMP280SensorOperation BMP280_DEV::setPresOversampling(BMP280_Oversampling presOversampling) {
	if (this->isInitialized) {
		if (!Adafruit_BusIO_RegisterBits(&this->ctrlMeasReg.value(), 3U, (uint8_t)BMP280_CTRL_MEAS_OFFSETS::OSRS_P).write((uint8_t)presOversampling)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		this->configuration.pressureOversampling = presOversampling;
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}

BMP280SensorOperation BMP280_DEV::setTempOversampling(BMP280_Oversampling tempOversampling)	{
	if (this->isInitialized) {
		if (!Adafruit_BusIO_RegisterBits(&this->ctrlMeasReg.value(), 3U, (uint8_t)BMP280_CTRL_MEAS_OFFSETS::OSRS_T).write((uint8_t)tempOversampling)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		this->configuration.temperatureOversampling = tempOversampling;
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}

BMP280SensorOperation BMP280_DEV::setIIRFilter(BMP280_IIRFilter iirFilter) {
	if (this->isInitialized) {
		if (!Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U, (uint8_t)BMP280_CONFIG_OFFSETS::FILTER).write((uint8_t)iirFilter)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		this->configuration.iirFilter = iirFilter;
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}

BMP280SensorOperation BMP280_DEV::setTimeStandby(BMP280_TimeStandby timeStandby) {
	if (this->isInitialized) {
		if (!Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U, (uint8_t)BMP280_CONFIG_OFFSETS::T_SB).write((uint8_t)timeStandby)) {
			return BMP280SensorOperation(SENSOR_STATUS_I2C_FAIL);
		}
		this->configuration.timeStandby = timeStandby;
		return BMP280SensorOperation(SENSOR_STATUS_OP_OK);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}


BMP280SensorOperation BMP280_DEV::getTemperature(bool waitDataReadyBit) {
	BMP280SensorOperation opRes = this->getTempPres(waitDataReadyBit);
	switch (opRes.sensorStatus) {
		case SENSOR_STATUS_DATA_OK: {
			if (std::holds_alternative<BMP280_Measurements>(opRes.data)) {
				BMP280_Measurements measurements = std::get<BMP280_Measurements>(opRes.data);

				if (measurements.temperature) {
					return BMP280SensorOperation(SENSOR_STATUS_DATA_OK, measurements.temperature.value());
				}
			}
		}
		// Fall through is intentional
		default: {
			return BMP280SensorOperation(opRes.sensorStatus);
		}
	}
}


// ? Pressure should always be compensated?
BMP280SensorOperation BMP280_DEV::getPressure(bool waitDataReadyBit) {
	BMP280SensorOperation opRes = this->getTempPres(waitDataReadyBit);
	switch (opRes.sensorStatus) {
		case SENSOR_STATUS_DATA_OK: {
			if (std::holds_alternative<BMP280_Measurements>(opRes.data)) {
				BMP280_Measurements measurements = std::get<BMP280_Measurements>(opRes.data);

				if (measurements.pressure) {
					return BMP280SensorOperation(SENSOR_STATUS_DATA_OK, measurements.pressure.value());
				}
			}
		// Fall through is intentional
		}
		default: {
			return BMP280SensorOperation(opRes.sensorStatus);
		}
	}
}


BMP280SensorOperation BMP280_DEV::getAltitude(bool waitDataReadyBit, float seaLevelPressure) {
	BMP280SensorOperation opRes = this->getTempPres(waitDataReadyBit);
	switch (opRes.sensorStatus) {
		case SENSOR_STATUS_DATA_OK: {
			if (std::holds_alternative<BMP280_Measurements>(opRes.data)) {
				BMP280_Measurements measurements = std::get<BMP280_Measurements>(opRes.data);

				if (measurements.temperature && measurements.pressure) {
					float altitude = ((float)powf(seaLevelPressure / measurements.pressure.value(), 0.190223f) - 1.0f) * (measurements.temperature.value() + 273.15f) / 0.0065f;
					return BMP280SensorOperation(SENSOR_STATUS_DATA_OK, altitude);
				}
			}
		}
		// Fall through is intentional
		default: {
			return BMP280SensorOperation(opRes.sensorStatus);
		}
	}
}


BMP280SensorOperation BMP280_DEV::getTempPresAlt(bool waitDataReadyBit, float seaLevelPressure) {
	BMP280SensorOperation opRes = this->getTempPres(waitDataReadyBit);
	switch (opRes.sensorStatus) {
		case SENSOR_STATUS_DATA_OK: {
			if (std::holds_alternative<BMP280_Measurements>(opRes.data)) {
				BMP280_Measurements measurements = std::get<BMP280_Measurements>(opRes.data);

				if (measurements.temperature && measurements.pressure) {
					// Convert pressure from Pascals to hPa
					float pressure_hPa = measurements.pressure.value() / 100.0f;

					// Temperature in Kelvin
					float temperature_K = measurements.temperature.value() + 273.15f;

					// Calculate altitude using the barometric formula
					float altitude = ((powf(seaLevelPressure / pressure_hPa, 0.190223f) - 1.0f) * temperature_K) / 0.0065f;
					
					measurements.altitude = altitude;
					return BMP280SensorOperation(SENSOR_STATUS_DATA_OK, measurements);
				}
			}
		}
		// Fall through is intentional
		default: {
			return BMP280SensorOperation(opRes.sensorStatus);
		}
	}
}


BMP280SensorOperation BMP280_DEV::end(void) {
  if (this->isInitialized) {
    
    return BMP280SensorOperation(SENSOR_STATUS_NOT_IMPLEMENTED);
  }
  return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}




BMP280SensorOperation BMP280_DEV::getTempPres(bool waitDataReadyBit) {
	if (this->isInitialized) {
		if (waitDataReadyBit) {
			// ? timeout?
			while(!this->dataReady())
				delay(1);
		}

		uint8_t data[6];
		if (!this->pressMSBReg.value().read(&data[0], sizeof(data))) {
			return BMP280SensorOperation(SENSOR_STATUS_DATA_BAD);
		}

		int32_t adcTemp = (int32_t)data[3] << 12 | (int32_t)data[4] << 4 | (int32_t)data[5] >> 4;
		int32_t adcPres = (int32_t)data[0] << 12 | (int32_t)data[1] << 4 | (int32_t)data[2] >> 4;
		int32_t temp = this->bmp280_compensate_T_int32(adcTemp);
		uint32_t pres = this->bmp280_compensate_P_int64(adcPres);

		BMP280_Measurements measurements = {
			.temperature = (float)temp / 100.0f, 
			.pressure = (float)pres / 256.0f
		};

		return BMP280SensorOperation(SENSOR_STATUS_DATA_OK, measurements);
	}
	return BMP280SensorOperation(SENSOR_STATUS_NOT_INIT);
}


uint8_t BMP280_DEV::dataReady(void) {
	if (this->configuration.mode == BMP280_MeasurementMode::SLEEP) {
		return 0;
	}
	uint8_t measuring = this->status_MEASURING_bits.value().read();
	if (measuring ^ this->previousMeasuringBit) {
		this->previousMeasuringBit = measuring;
		if (!measuring) {
			if (this->configuration.mode == BMP280_MeasurementMode::FORCED) {
				this->configuration.mode = BMP280_MeasurementMode::SLEEP;
			}
			return 1;
		}
	}
	return 0;
}


/**
 * @brief BMP280 temperature compensation function.
 * @return Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 */
int32_t BMP280_DEV::bmp280_compensate_T_int32(int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)trimParams.dig_T1 << 1))) * ((int32_t)trimParams.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)trimParams.dig_T1)) * ((adc_T >> 4) - ((int32_t)trimParams.dig_T1))) >> 12) *
  ((int32_t)trimParams.dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

/**
 * @brief BMP280 pressure compensation function.
 * @example Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa.
 * @return Pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 */
uint32_t BMP280_DEV::bmp280_compensate_P_int64(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)trimParams.dig_P6;
  var2 = var2 + ((var1 * (int64_t)trimParams.dig_P5) << 17);
  var2 = var2 + (((int64_t)trimParams.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)trimParams.dig_P3) >> 8) + ((var1 * (int64_t)trimParams.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)trimParams.dig_P1) >> 33;
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)trimParams.dig_P9) * (p >> 13) * (p>>13)) >> 25;
  var2 = (((int64_t)trimParams.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)trimParams.dig_P7) << 4);
  return (uint32_t)p;
}
