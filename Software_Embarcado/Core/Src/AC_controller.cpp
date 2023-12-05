#include <AC_controller.hpp>

AC_controller::AC_controller(GPIO_TypeDef _gpioPort, uint16_t _gpioPin){
	gpioPort = _gpioPort;
	gpioPin = _gpioPin;
}
//PRIVATE FUNCTIONS
void AC_controller::sendBit(unsigned char bit){
	GPIO_Output(&gpioPort, gpioPin, 0);
	delay_us(560);
	GPIO_Output(&gpioPort, gpioPin, 1);
	if (bit == '1')
		delay_us(1500); // Ar de Joinville 1690 ou 1500us
	else
		delay_us(480); //Ar de Joinville Delay < 500us (deve ser 480)
}

void AC_controller::sendByte(unsigned char byte){
	unsigned int i;
	i = 8;
	while (i > 0) {
		if (byte & (1 << (i - 1)))
			sendBit('1');
		else
			sendBit('0');
		i--;
	}
}

void AC_controller::sendHeader(){
	GPIO_Output(&gpioPort, gpioPin, 0);
	delay_us(800); //ar de Joinville
	GPIO_Output(&gpioPort, gpioPin, 1);
	delay_us(18000); //ar Joinville (?)
}

//PUBLIC FUNCTIONS
//REVER SEND COMMAND
void AC_controller::sendCommand(Mode mode, Fan fan, Temperature temperature,  Turbo turbo, Swing swing, Sheet sheet){
    unsigned char hora = 0;
    unsigned char min = 0;
	char MSG[15];
    MSG[0]=0x6A;
    MSG[1]=temperature;
    MSG[2]=0x00;
    MSG[3]=0x00;
    MSG[4]=(fan << 4)| mode;
    if(mode==OFF){ // Se o Modo OFF foi selecionado, valores padrões podem ser passados
        MSG[5]=0X03;
    }
    else{
        if(mode!=COOL){ // Se Modo diferente de COOL, folha não esta disponivel
            sheet=SHEET_OFF;
        }
        MSG[5]=(sheet<<7)|(swing<<3)|0x40;
    }
    MSG[6]=0x00;
    MSG[7]=0x00;
    MSG[8]=0x30;
    MSG[9]=0x00;
    MSG[10]=0x30|turbo;
    MSG[11]=0x00; // Segundos
    MSG[12]=min; // Minutos
    MSG[13]=hora; // Horas
    MSG[14]=checksumCalculate(MSG);

    sendHeader();
    for(int byte=0; byte<15; byte++){
        sendByte(MSG[byte]);
    }
    sendBit('1');

    //clear message
    for(int byte=0; byte<15; byte++){
        MSG[byte]=0;
    }
}

char AC_controller::checksumCalculate(char *Data){
	char LSB = 0;
	char MSB = 0;
	LSB = (invertByte(Data[0] & 0x0F)) >> 4;
	MSB = (invertByte(((Data[0] >> 4) & 0x0F))) >> 4;
	for (int i = 1; i < 14; i++) {
		LSB = ((invertByte(Data[i] & 0x0F)) >> 4) + LSB;
		MSB = ((invertByte(((Data[i] >> 4) & 0x0F))) >> 4) + MSB;

	}
	return invertByte(MSB + LSB);
}

char AC_controller::invertByte(char byte){
    byte = (((byte & 0x0f) << 4) | ((byte & 0xf0) >> 4));
    byte = ((byte & 0x33) << 2) | ((byte & 0xcc) >> 2);
    byte = ((byte & 0x55) << 1) | ((byte & 0xaa) >> 1);

    return byte;
}
