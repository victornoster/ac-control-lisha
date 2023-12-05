/*
 * AC_controller.h
 */

#ifndef	__ACCONTROLLER_H__
#define __ACCONTROLLER_H__
#include "gpio.h"
#include "tim.h"

class AC_controller {
public:
	AC_controller(GPIO_TypeDef _gpioPort, uint16_t _gpioPin);
	enum Mode {
		OFF = 0x00,
		COOL = 0x04,
		DRY = 0x0C,
		FAN = 0x0A,
		HEAT = 0x08,
		AUTO = 0x02,
	};

	enum Folha {
		FOLHA_ON = 0x00, FOLHA_OFF = 0x01,
	};

	enum Fan {
		FAN1 = 0x04, FAN2 = 0x0C, FAN3 = 0x08, FAN_AUTO = 0x00, //Dry, Hot, Cool
	};

	enum Turbo {
		TURBO_ON = 0x01, TURBO_OFF = 0x00,
	};

	enum Swing {
		SWING_OFF_NV1 = 0x04,
		SWING_OFF_NV2 = 0x02,
		SWING_OFF_NV3 = 0x06,
		SWING_OFF_NV4 = 0x01,
		SWING_OFF_NV5 = 0x05,
		SWING_ON = 0x03
	};
	enum Sheet {
		SHEET_ON = 0x00,
		SHEET_OFF = 0x01
	};

	enum Temperature {
		T32 = 0x3E,
		T31 = 0xDE,
		T30 = 0x5E,
		T29 = 0x9E,
		T28 = 0x1E,
		T27 = 0xEE,
		T26 = 0x6E,
		T25 = 0xAE,
		T24 = 0x2E,
		T23 = 0xCE,
		T22 = 0x4E,
		T21 = 0x8E,
		T20 = 0x0E,
		T19 = 0xF6,
		T18 = 0x76,
		T17 = 0xB6,
		T16 = 0x36,
	};
	/*enum OnOff {
		ON = 01101010, OFF = 0
	};*/

	void sendCommand(Mode mode, Fan fan, Temperature temperature,  Turbo turbo, Swing swing, Sheet sheet);
private:
	GPIO_TypeDef gpioPort;
	uint16_t gpioPin;

	//static char message[8];	

	/*typedef struct {
		unsigned char temp;
		unsigned char mode;
		unsigned char fan;
		unsigned char turbo;
		unsigned char swing;
		unsigned char folha;
		unsigned char turn;
		unsigned char seg;
		unsigned char min;
		unsigned char hora;
	} Message;*/
	void sendBit(unsigned char bit);
	void sendByte(unsigned char byte);
	void sendHeader();
	//void updateAC(uint8_t command, uint8_t command2);
	char checksumCalculate(char *Data);
	char invertByte(char byte);

};

#endif  /*AC_CONTROLLER*/
