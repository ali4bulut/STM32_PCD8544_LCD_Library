/*
 * pcd8544.c
 *
 *  Created on: Dec 20, 2021
 *      Author: alibl
 */
#include "pcd8544.h"

void write(Pcd_Handle_t *lcd, uint8_t *data, enum COMM_MODE mode);

void write(Pcd_Handle_t *lcd, uint8_t *data, enum COMM_MODE mode) {
	CE_PIN_LOW(lcd);
	if (mode == CMD_MODE) {
		DC_PIN_LOW(lcd);
	}
	else {
		DC_PIN_HIGH(lcd);
	}
//	HAL_SPI_Transmit(lcd->spi, data, 1, 1000);
	HAL_SPI_Transmit_DMA(lcd->spi, data, 1);
	CE_PIN_HIGH(lcd);
}

void PCD8544_Init(Pcd_Handle_t *lcd) {

	uint8_t data = 0;

	//Reset the lcd
	CE_PIN_HIGH(lcd);

	RST_PIN_HIGH(lcd);
	HAL_Delay(10);
	RST_PIN_LOW(lcd);
	HAL_Delay(10);
	RST_PIN_HIGH(lcd);

	//Initiallzation
	//NOP
	data = NOP;
	write(lcd, &data, CMD_MODE);
	//Power up and H = 1
	data = FUNCTION_SET | 0x01;
	write(lcd, &data, CMD_MODE);
	//Vop contrast settings
	data = SET_VOP | 0x40;
	write(lcd, &data, CMD_MODE);
	//Bias settings
	data = BIAS_SYS | 0x04;
	write(lcd, &data, CMD_MODE);
	//Power up and H = 0
	data =  FUNCTION_SET;
	write(lcd, &data, CMD_MODE);
	//Y address is 0
	data = SET_Y_ADDR;
	write(lcd, &data, CMD_MODE);
	//X address is 0
	data = SET_X_ADDR;
	write(lcd, &data, CMD_MODE);
	//Display on
	data = DISPLAY_CNTRL | 0x04;
	write(lcd, &data, CMD_MODE);
	//Clear the screen
	Lcd_Clear(lcd);
}

void Lcd_Clear(Pcd_Handle_t *lcd) {
	uint8_t data = 0x00;
	Lcd_Set_Cursor(lcd, 0, 0);
	CE_PIN_LOW(lcd);
	DC_PIN_HIGH(lcd);
	for (uint16_t i = 0; i < 504; i++)
//		HAL_SPI_Transmit(lcd->spi, &data, 1, 1000);
		HAL_SPI_Transmit_DMA(lcd->spi, &data, 1);

	DC_PIN_LOW(lcd);
	CE_PIN_HIGH(lcd);
}

void Lcd_Print(Pcd_Handle_t *lcd, const char *str, enum COLOR color) {
	int i = 0;
	uint8_t data = 0;
	uint8_t inverse = 0;

	inverse = (color == WHITE) ? 0xFF : 0x00;

	CE_PIN_LOW(lcd);
	DC_PIN_HIGH(lcd);

	while (str[i] != '\0') {
		for (uint8_t j = 0; j < 5; j++) {
			data = (ASCII[str[i] - 32][j] ^ inverse);
//			HAL_SPI_Transmit(lcd->spi, &data, 1, 1000);
			HAL_SPI_Transmit_DMA(lcd->spi, &data, 1);
		}
		data = inverse;
//		HAL_SPI_Transmit(lcd->spi, &data, 1, 1000);
		HAL_SPI_Transmit_DMA(lcd->spi, &data, 1);
		i++;
	}

	DC_PIN_LOW(lcd);
	CE_PIN_HIGH(lcd);
}

void Lcd_Set_Constrant(Pcd_Handle_t *lcd, uint8_t constrant) {
	uint8_t temp = 0x7F & constrant;
	uint8_t data = NOP;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET | 0x01;
	write(lcd, &data, CMD_MODE);
	data = SET_VOP | temp;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET;
	write(lcd, &data, CMD_MODE);
}

void Lcd_Set_Bias(Pcd_Handle_t *lcd, uint8_t bias) {
	uint8_t temp = 0x07 & bias;
	uint8_t data = NOP;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET | 0x01;
	write(lcd, &data, CMD_MODE);
	data = BIAS_SYS | temp;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET;
	write(lcd, &data, CMD_MODE);
}

//0 - 83 on X axis  and 0 - 5 on  Y Axis
void Lcd_Set_Cursor(Pcd_Handle_t *lcd, uint8_t row, uint8_t col) {
	uint8_t tRow = (0x07 & row);
	uint8_t tCol = (0x7F & col);
	uint8_t data = FUNCTION_SET;
	write(lcd, &data, CMD_MODE);
	data = SET_Y_ADDR | tRow;
	write(lcd, &data, CMD_MODE);
	data = SET_X_ADDR | tCol;
	write(lcd, &data, CMD_MODE);
}

void Lcd_Display(Pcd_Handle_t *lcd, const uint8_t *buffer, uint8_t rowsPx, uint8_t colsPx, uint8_t posY, uint8_t posX, enum COLOR color) {
	uint8_t data = 0;
	uint8_t inverse = 0;
	uint16_t px = ((rowsPx * colsPx) / 8);

	inverse = (color == WHITE) ? 0xFF : 0x00;

	Lcd_Set_Cursor(lcd, posY, posX);

	for (uint16_t i = 0; i < px; i++) {
		if (i != 0 && (i % colsPx) == 0)
			Lcd_Set_Cursor(lcd, ++posY, posX);
		data = (buffer[i] ^ inverse);
		write(lcd, &data, DATA_MODE);
	}

}

void Lcd_Invert_Color(Pcd_Handle_t *lcd) {
	uint8_t data = NOP;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET;
	write(lcd, &data, CMD_MODE);
	data = DISPLAY_CNTRL | 0x05;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET;
	write(lcd, &data, CMD_MODE);
}

void Lcd_Set_Addressing(Pcd_Handle_t *lcd, enum ADDR_MODE mode) {
	uint8_t data = NOP;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET | 0x01 | 0x2;
	write(lcd, &data, CMD_MODE);
	data = FUNCTION_SET;
	write(lcd, &data, CMD_MODE);
}

void testLcd(Pcd_Handle_t *lcd) {
	uint8_t data = 0x0C;
	write(lcd, &data, CMD_MODE);
	HAL_Delay(500);
	data = 0x0C;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	data = 0x05;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	data = 0x07;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	data = 0x00;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	data = 0x1F;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	data = 0x04;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	data = 0x1F;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	data = 0x0D;
	write(lcd, &data, CMD_MODE);
	HAL_Delay(500);
	data = 0x80;
	write(lcd, &data, CMD_MODE);
	HAL_Delay(500);
	data = 0x00;
	write(lcd, &data, DATA_MODE);
	HAL_Delay(500);
	Lcd_Clear(lcd);

}
