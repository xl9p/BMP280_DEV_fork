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

#pragma once

#include <memory>
#include <optional>

#include <Wire.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>

#include <SensorCommon.h>


/* BMP280 Definitions */
enum class  BMP280_DEFINITIONS : uint8_t {
	I2C_ADDR 		= 0x77,	// The BMP280 I2C address
	I2C_ADDR_ALT 	= 0x76,	// The BMP280 I2C alternate address
	DEVICE_ID 		= 0x58,	// The BMP280 device ID
	RESET_CODE 		= 0xB6	// The BMP280 reset code
};


/* BMP280 Registers */
enum class  BMP280_REGISTERS : uint8_t {
	TRIM_PARAMS	= 0x88,	// Trim parameter registers' base sub-address
	DEVICE_ID 	= 0xD0,	// Device ID register sub-address
	RESET 		= 0xE0,	// Reset register sub-address
	STATUS      = 0xF3,	// Status register sub-address
	CTRL_MEAS   = 0xF4,	// Control and measurement register sub-address
	CONFIG      = 0xF5,	// Configuration register sub-address
	PRES_MSB    = 0xF7,	// Pressure Most Significant Byte (MSB) register sub-address
	PRES_LSB    = 0xF8,	// Pressure Least Significant Byte (LSB) register sub-address
	PRES_XLSB	= 0xF9,	// Pressure eXtended Least Significant Byte (XLSB) register sub-address
	TEMP_MSB	= 0xFA,	// Pressure Most Significant Byte (MSB) register sub-address
	TEMP_LSB	= 0xFB,	// Pressure Least Significant Byte (LSB) register sub-address
	TEMP_XLSB	= 0xFC	// Pressure eXtended Least Significant Byte (XLSB) register sub-address
};

/* BMP280 STATUS register bit field */
enum class  BMP280_STATUS_OFFSETS : uint8_t {
	IM_UPDATE = 0x00,
	MEASURING = 0x03
};

/* BMP280 CTRL_MEAS register bit field  */
enum class BMP280_CTRL_MEAS_OFFSETS : uint8_t {
	MODE 	= 0x00,
	OSRS_P 	= 0x02,
	OSRS_T 	= 0x05
};

/* BMP280 CONFIG register bit field */
enum class BMP280_CONFIG_OFFSETS : uint8_t {
	SPI3W_EN	= 0x00,
	FILTER		= 0x02,
	T_SB 		= 0x05
};


/* BMP280 Modes */
enum class BMP280_MeasurementMode : uint8_t {
	SLEEP	= 0x00,
	FORCED	= 0x01,
	NORMAL	= 0x03
};


/* BMP280 Oversampling parameters */
enum class BMP280_Oversampling : uint8_t {
	SKIP 	= 0x00,
	X1		= 0x01,
	X2   	= 0x02,
	X4  	= 0x03,
	X8   	= 0x04,
	X16  	= 0x05
};


/* BMP280 IIR filter parameters */
enum class BMP280_IIRFilter : uint8_t {
	OFF	= 0x00,
	X2	= 0x01,
	X4	= 0x02,
	X8	= 0x03,
	X16	= 0x04
};


/* BMP280 Time standby parameters */
enum class BMP280_TimeStandby : uint8_t {
	TS_05MS		= 0x00,
	TS_62MS     = 0x01,
	TS_125MS    = 0x02,
	TS_250MS    = 0x03,
	TS_500MS    = 0x04,
	TS_1000MS   = 0x05,
	TS_2000MS   = 0x06,
	TS_4000MS	= 0x07
};


/* BMP280 Configuration */
typedef struct _bmp280_config {
	BMP280_MeasurementMode mode = BMP280_MeasurementMode::NORMAL;
	BMP280_Oversampling temperatureOversampling = BMP280_Oversampling::X2;
	BMP280_Oversampling pressureOversampling = BMP280_Oversampling::X16;
	BMP280_IIRFilter iirFilter = BMP280_IIRFilter::OFF;
	BMP280_TimeStandby timeStandby = BMP280_TimeStandby::TS_05MS;
} BMP280_Config;


struct BMP280_Measurements{
    std::optional<float> temperature = std::nullopt;
    std::optional<float> pressure = std::nullopt;
    std::optional<float> altitude = std::nullopt;
};

using BMP280SensorOperation = SensorOperation<BMP280_Config, BMP280_Measurements>;


/* BMP280_DEV Class definition */
class BMP280_DEV {
	public:
		BMP280_DEV(TwoWire* twoWire = &Wire, uint8_t addr = (uint8_t) BMP280_DEFINITIONS::I2C_ADDR_ALT);
		// BMP280_DEV(SPIClass& spiClass, SPISettings& spiSettings, uint8_t addr = BMP280_I2C_ADDR);

		BMP280SensorOperation begin(const BMP280_Config &sensorConfig);
		BMP280SensorOperation reset();

		BMP280SensorOperation end(void);

		BMP280SensorOperation startNormalConversion(void);
		BMP280SensorOperation startForcedConversion(void);
		BMP280SensorOperation stopConversion(void);

		BMP280SensorOperation setPresOversampling(BMP280_Oversampling pressOversampling);
		BMP280SensorOperation setTempOversampling(BMP280_Oversampling tempOversampling);
		BMP280SensorOperation setIIRFilter(BMP280_IIRFilter iirFilter);
		BMP280SensorOperation setTimeStandby(BMP280_TimeStandby timeStandby);

		BMP280SensorOperation getConfiguration(bool update);

		BMP280SensorOperation getTemperature(bool waitDataReadyBit);
		BMP280SensorOperation getPressure(bool waitDataReadyBit);
		BMP280SensorOperation getAltitude(bool waitDataReadyBit, float seaLevelPressure = 1013.25f);
		BMP280SensorOperation getTempPresAlt(bool waitDataReadyBit, float seaLevelPressure = 1013.25f);
 	private:
		bool isInitialized = false;
		bool previousMeasuringBit;

		/**
		 * @brief Last written configuration 
		 */
		BMP280_Config configuration;


		std::unique_ptr<Adafruit_I2CDevice> i2cDevice;

		/**
		 * @warning NOT SUPPORTED
		 */
		std::unique_ptr<Adafruit_SPIDevice> spiDevice; 


		std::optional<Adafruit_BusIO_Register> deviceIDReg;
		std::optional<Adafruit_BusIO_Register> pressMSBReg;
		std::optional<Adafruit_BusIO_Register> tempMSBReg;
		std::optional<Adafruit_BusIO_Register> trimParamsStartReg;
		std::optional<Adafruit_BusIO_Register> resetReg;
		std::optional<Adafruit_BusIO_Register> statusReg;
		std::optional<Adafruit_BusIO_Register> ctrlMeasReg;
		std::optional<Adafruit_BusIO_Register> configReg;


		std::optional<Adafruit_BusIO_RegisterBits> status_IMUPDATE_bits;
		std::optional<Adafruit_BusIO_RegisterBits> status_MEASURING_bits;

		std::optional<Adafruit_BusIO_RegisterBits> ctrlmeas_MODE_bits;


		struct {
			uint16_t dig_T1;
			int16_t  dig_T2;
			int16_t  dig_T3;
			uint16_t dig_P1;
			int16_t  dig_P2;
			int16_t  dig_P3;
			int16_t  dig_P4;
			int16_t  dig_P5;
			int16_t  dig_P6;
			int16_t  dig_P7;
			int16_t  dig_P8;
			int16_t  dig_P9;
		} trimParams;
		
		int32_t t_fine;
		int32_t bmp280_compensate_T_int32(int32_t adc_T);
		uint32_t bmp280_compensate_P_int64(int32_t adc_P);


		uint8_t dataReady(void);

		BMP280SensorOperation getTempPres(bool waitDataReadyBit);
};