#ifndef __BMP280_H__
#define __BMP280_H__

#include "i2c.h"
#include <Sensor.hpp>

/*I2C Adresses*/
#define BMP280_ADDR 0x77 // 0xED
// #define BM280_ADDR (0x76); //alternative
/*!
 * Registers available on the sensor.
 */
enum
{
    BMP280_REGISTER_DIG_T1 = 0x88,
    BMP280_REGISTER_DIG_T2 = 0x8A,
    BMP280_REGISTER_DIG_T3 = 0x8C,
    BMP280_REGISTER_DIG_P1 = 0x8E,
    BMP280_REGISTER_DIG_P2 = 0x90,
    BMP280_REGISTER_DIG_P3 = 0x92,
    BMP280_REGISTER_DIG_P4 = 0x94,
    BMP280_REGISTER_DIG_P5 = 0x96,
    BMP280_REGISTER_DIG_P6 = 0x98,
    BMP280_REGISTER_DIG_P7 = 0x9A,
    BMP280_REGISTER_DIG_P8 = 0x9C,
    BMP280_REGISTER_DIG_P9 = 0x9E,
    BMP280_REGISTER_CHIPID = 0xD0,
    BMP280_REGISTER_VERSION = 0xD1,
    BMP280_REGISTER_SOFTRESET = 0xE0,
    BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
    BMP280_REGISTER_STATUS = 0xF3,
    BMP280_REGISTER_CONTROL = 0xF4,
    BMP280_REGISTER_CONFIG = 0xF5,
    BMP280_REGISTER_PRESSUREDATA = 0xF7,
    BMP280_REGISTER_TEMPDATA = 0xFA,
};

class BMP280 : public Sensor<float> {
	
private:
	float tempValue;

    uint8_t dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3,
	dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
public:
	enum sensor_sampling {
		/** No over-sampling. */
		SAMPLING_NONE = 0x00,
		/** 1x over-sampling. */
		SAMPLING_X1 = 0x01,
		/** 2x over-sampling. */
		SAMPLING_X2 = 0x02,
		/** 4x over-sampling. */
		SAMPLING_X4 = 0x03,
		/** 8x over-sampling. */
		SAMPLING_X8 = 0x04,
		/** 16x over-sampling. */
		SAMPLING_X16 = 0x05
	};

	/** Operating mode for the sensor. */
	enum sensor_mode {
		/** Sleep mode. */
		MODE_SLEEP = 0x00,
		/** Forced mode. */
		MODE_FORCED = 0x01,
		/** Normal mode. */
		MODE_NORMAL = 0x03,
		/** Software reset. */
		MODE_SOFT_RESET_CODE = 0xB6
	};

	/** Filtering level for sensor data. */
	enum sensor_filter {
		/** No filtering. */
		FILTER_OFF = 0x00,
		/** 2x filtering. */
		FILTER_X2 = 0x01,
		/** 4x filtering. */
		FILTER_X4 = 0x02,
		/** 8x filtering. */
		FILTER_X8 = 0x03,
		/** 16x filtering. */
		FILTER_X16 = 0x04
	};

	/** Standby duration in ms */
	enum standby_duration {
		/** 1 ms standby. */
		STANDBY_MS_1 = 0x00,
		/** 62.5 ms standby. */
		STANDBY_MS_63 = 0x01,
		/** 125 ms standby. */
		STANDBY_MS_125 = 0x02,
		/** 250 ms standby. */
		STANDBY_MS_250 = 0x03,
		/** 500 ms standby. */
		STANDBY_MS_500 = 0x04,
		/** 1000 ms standby. */
		STANDBY_MS_1000 = 0x05,
		/** 2000 ms standby. */
		STANDBY_MS_2000 = 0x06,
		/** 4000 ms standby. */
		STANDBY_MS_4000 = 0x07
	};

	BMP280(uint8_t register_F4, uint8_t register_F5);
	void calibration();
	void setSensorId(int id);
	int getSensorId();
	float readSensor();
};


#endif /*__BMP280_H__*/
