#include <BMP280.hpp>

BMP280::BMP280(uint8_t register_F4, uint8_t register_F5)
{

    I2C_Write_Register(BMP280_ADDR, 0xF4, register_F4);// osrs_t 010 x2, osrs_p 16 101, mode normal 11
	I2C_Write_Register(BMP280_ADDR, 0xF5, register_F5);// standby time 500ms 100, filter 16 100, SPI DIS 0
    calibration();
}

void BMP280::calibration()
{
    uint8_t rx_buff[24], starting_address = 0x88;

    HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, &starting_address, 1, 10000);
    HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDR + 1, &rx_buff[0], 24, 10000);

    dig_T1 = (rx_buff[0]) + (rx_buff[1] << 8);
    dig_T2 = (rx_buff[2]) + (rx_buff[3] << 8);
    dig_T3 = (rx_buff[4]) + (rx_buff[5] << 8);
    dig_P1 = (rx_buff[6]) + (rx_buff[7] << 8);
    dig_P2 = (rx_buff[8]) + (rx_buff[9] << 8);
    dig_P3 = (rx_buff[10]) + (rx_buff[11] << 8);
    dig_P4 = (rx_buff[12]) + (rx_buff[13] << 8);
    dig_P5 = (rx_buff[14]) + (rx_buff[15] << 8);
    dig_P6 = (rx_buff[16]) + (rx_buff[17] << 8);
    dig_P7 = (rx_buff[18]) + (rx_buff[19] << 8);
    dig_P8 = (rx_buff[20]) + (rx_buff[21] << 8);
    dig_P9 = (rx_buff[22]) + (rx_buff[23] << 8);
}

float BMP280::readSensor(){
    uint8_t status, rx_buff[6], starting_address=0xF7;
    signed long temperature_raw;
    do{
		status=I2C_Read_Register(BMP280_ADDR, 0xF3);
	} while(((status&0b00001000)==8)||((status&0b00000001)==1));

    HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, &starting_address, 1, 10000);
	HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDR + 1, &rx_buff[0], 6, 10000);

    temperature_raw=(rx_buff[2]<<12)+(rx_buff[1]<<4)+(rx_buff[0]>>4);
    double aux1, aux2;
    aux1 = (((double)temperature_raw)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
    aux2 = ((((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
    double t_fine = (int32_t)(aux1+aux2);
    tempValue = (aux1+aux2)/5120.0;
    return tempValue;
}
