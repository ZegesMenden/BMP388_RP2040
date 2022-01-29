#ifndef BMP388_H
#define BMP388_H

#include <pico/stdlib.h>
#include <hardware/i2c.h>

// register map

#define CHIP_ID 		0X00
#define ERR           	0x02
#define STATUS 			0x03

#define DATA_PRES_MSB  	0x04
#define DATA_PRES_LSB  	0x05
#define DATA_PRES_XLSB 	0x06

#define DATA_TEMP_MSB 	0x07
#define DATA_TEMP_LSB 	0x08
#define DATA_TEMP_XLSB 	0x09

#define SENSORTIME_MSB 	0x0C
#define SENSORTIME_LSB 	0x0D
#define SENSORTIME_XLSB 0x0E

#define INT_STATUS 		0x11

#define INT_CTRL 		0x19
#define PWR_CTRL 		0x1B

#define OSR 			0x1C
#define ODR		 		0x1D
// OSR/ODR configuration table available in the datasheet at page 37
// https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP388-DS001.pdf

#define CONFIG 			0x1F
#define CALIB_DATA 		0x31
// 0x30 ... 0x57

#define CMD 			0x7E

// commands
#define SOFT_RESET 		0xB6

#define FILTER_COEFF_0  	0b00000000
#define FILTER_COEFF_1  	0b00000001
#define FILTER_COEFF_3  	0b00000010
#define FILTER_COEFF_7  	0b00000011
#define FILTER_COEFF_15 	0b00000100
#define FILTER_COEFF_31 	0b00000101
#define FILTER_COEFF_63 	0b00000110
#define FILTER_COEFF_127 	0b00000111

#define OSR_PRES_NONE 		0b00000
#define OSR_PRES_2X 		0b00001
#define OSR_PRES_4X 		0b00010
#define OSR_PRES_8X 		0b00011
#define OSR_PRES_16X 		0b00100
#define OSR_PRES_32X 		0b00101

#define OSR_TEMP_NONE 		0b00000000
#define OSR_TEMP_2X 		0b00000001
#define OSR_TEMP_4X 		0b00000010
#define OSR_TEMP_8X 		0b00000011
#define OSR_TEMP_16X 		0b00000100
#define OSR_TEMP_32X 		0b00000101

#define ODR_1HZ 			0x00
#define ODR_2HZ 			0x01
#define ODR_4HZ 			0x02
#define ODR_8HZ 			0x03
#define ODR_16HZ 			0x04
#define ODR_32HZ 			0x05
#define ODR_64HZ 			0x06
#define ODR_128HZ 			0x07
#define ODR_256HZ 			0x08
#define ODR_512HZ 			0x09
#define ODR_1024HZ 			0x0A
#define ODR_2048HZ 			0x0B
#define ODR_4096HZ 			0x0C
#define ODR_8192HZ 			0x0D
#define ODR_16384HZ 		0x0E
#define ODR_32768HZ 		0x0F
#define ODR_65536HZ 		0x10
#define ODR_131072HZ 		0x11
// ODR subsampling is 2^odr value
// max ODR value is 17

#define PWR_MODE_NORMAL 	0b11110000
#define OSR_P_T_NORMAL 		0b00010010 // 4x oversampling for pressure and temperature (50HZ output with 200HZ ODR)
#define ODR_P_T_NORMAL 		0b00000000

typedef struct calib_data {

	float PAR_T1;
	float PAR_T2;
	float PAR_T3;

	float PAR_P1;
	float PAR_P2;
	float PAR_P3;
	float PAR_P4;
	float PAR_P5;
	float PAR_P6;
	float PAR_P7;
	float PAR_P8;
	float PAR_P9;
	float PAR_P10;
	float PAR_P11;

} bmp_calib_data_t;

class BMP388
{
	public: 
		
		BMP388(i2c_inst_t i2c, uint8_t addr) { this->i2c = i2c; this->addr = addr; };

		void init();

		void read_calib_data();

		void read();
		void read_raw();

		void compensate_temp(uint32_t temp_uncomp);
		void compensate_pres(uint32_t pres_uncomp);

		void compute_alt();
		void compute_alt_fast();

		float get_alt() { return alt; };

		float get_temp() { return temp; };
		float get_pres() { return pres; };

		uint32_t get_temp_uncomp() { return temp_uncomp; };
		uint32_t get_pres_uncomp() { return pres_uncomp; };

	private:

		uint32_t temp_uncomp, pres_uncomp;

		float temp, pres, alt;

		bmp_calib_data_t calib_data;

		i2c_inst_t i2c;
		uint8_t addr;

		void write_byte(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t data);
    	void read_bytes(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t data, uint8_t len);

};

#endif