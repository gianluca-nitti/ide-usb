#include "main.h"
#include "ide.h"

/* registers are in format CS0 CS1 A2 A1 A0 */
#define REG_DATA           0b01000
#define REG_ERROR_FEATURES 0b01001
#define REG_SECTOR_COUNT   0b01010
#define REG_CYL_LOW        0b01100
#define REG_CYL_HIGH       0b01101
#define REG_HEAD_DEVICE    0b01110
#define REG_STATUS_COMMAND 0b01111

#define BYTE_COUNT_LOW     0b10100
#define BYTE_COUNT_HIGH    0b10101

#define REG_CS0_MASK 0b00010000
#define REG_CS1_MASK 0b00001000
#define REG_DA2_MASK 0b00000100
#define REG_DA1_MASK 0b00000010
#define REG_DA0_MASK 0b00000001

static void ide_set_bus_mode(uint32_t mode) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /*Configure GPIO pins : IDE_DD4_Pin IDE_DD5_Pin IDE_DD6_Pin IDE_DD7_Pin
	                           IDE_DD8_Pin IDE_DD9_Pin IDE_DD10_Pin IDE_DD11_Pin
	                           IDE_DD12_Pin IDE_DD13_Pin IDE_DD14_Pin IDE_DD15_Pin */
	GPIO_InitStruct.Pin = IDE_DD4_Pin|IDE_DD5_Pin|IDE_DD6_Pin|IDE_DD7_Pin
	                          |IDE_DD8_Pin|IDE_DD9_Pin|IDE_DD10_Pin|IDE_DD11_Pin
	                          |IDE_DD12_Pin|IDE_DD13_Pin|IDE_DD14_Pin|IDE_DD15_Pin;
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : IDE_DD0_Pin IDE_DD1_Pin IDE_DD2_Pin IDE_DD3_Pin */
	GPIO_InitStruct.Pin = IDE_DD0_Pin|IDE_DD1_Pin|IDE_DD2_Pin|IDE_DD3_Pin;
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

#define PORTD_BUS_IDR_MASK 0b00000000000000000000000000001111
#define PORTE_BUS_IDR_MASK 0b00000000000000001111111111110000
#define PORTD_BUS_BSRR_MASK 0b0000000000001111
#define PORTE_BUS_BSRR_MASK 0b1111111111110000

/*static uint16_t ide_bus_read() {
	ide_set_bus_mode(GPIO_MODE_INPUT); // TODO consider removing, should not be needed as it's the default state
	HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_RESET); // flash read strobe (active low)
	HAL_Delay(50);
	uint16_t result = (uint16_t)((GPIOD->IDR & PORTD_BUS_IDR_MASK) | (GPIOE->IDR & PORTE_BUS_IDR_MASK));
	HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_SET); // release read strobe
	return result;
}

static void ide_bus_write(uint16_t word) {
	ide_set_bus_mode(GPIO_MODE_OUTPUT_PP);
	GPIOD->BSRR = ((~word & PORTD_BUS_BSRR_MASK) << 16) | (word & PORTD_BUS_BSRR_MASK);
	GPIOE->BSRR = ((~word & PORTE_BUS_BSRR_MASK) << 16) | (word & PORTE_BUS_BSRR_MASK);
	HAL_GPIO_WritePin(IDE_DIOW_GPIO_Port, IDE_DIOW_Pin, GPIO_PIN_RESET); // flash write strobe (active low)
	HAL_Delay(1);
	HAL_GPIO_WritePin(IDE_DIOW_GPIO_Port, IDE_DIOW_Pin, GPIO_PIN_SET); // release write strobe
	ide_set_bus_mode(GPIO_MODE_INPUT);
}*/

static inline void ide_ndelay(int ns) {
	int cycles = ns / 6 + 1;
	for (int i = 0; i < cycles; i++);
	//HAL_Delay(1); // TODO
}

static void ide_select_register(uint8_t reg) {
	HAL_GPIO_WritePin(IDE_CS0_GPIO_Port, IDE_CS0_Pin, reg & REG_CS0_MASK ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IDE_CS1_GPIO_Port, IDE_CS1_Pin, reg & REG_CS1_MASK ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IDE_DA0_GPIO_Port, IDE_DA0_Pin, reg & REG_DA0_MASK ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IDE_DA1_GPIO_Port, IDE_DA1_Pin, reg & REG_DA1_MASK ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IDE_DA2_GPIO_Port, IDE_DA2_Pin, reg & REG_DA2_MASK ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint16_t ide_register_read(uint8_t reg) {
	ide_set_bus_mode(GPIO_MODE_INPUT); // TODO consider removing, should not be needed as it's the default state
	ide_select_register(reg);
	HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_RESET); // flash read strobe (active low)
	ide_ndelay(600);
	uint16_t result = (uint16_t)((GPIOD->IDR & PORTD_BUS_IDR_MASK) | (GPIOE->IDR & PORTE_BUS_IDR_MASK));
	HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_SET); // release read strobe
	ide_ndelay(600);
	return result;
}

static void ide_register_write(uint8_t reg, uint16_t word) {
	ide_set_bus_mode(GPIO_MODE_OUTPUT_PP);
	ide_select_register(reg);
	GPIOD->BSRR = ((~word & PORTD_BUS_BSRR_MASK) << 16) | (word & PORTD_BUS_BSRR_MASK);
	GPIOE->BSRR = ((~word & PORTE_BUS_BSRR_MASK) << 16) | (word & PORTE_BUS_BSRR_MASK);
	HAL_GPIO_WritePin(IDE_DIOW_GPIO_Port, IDE_DIOW_Pin, GPIO_PIN_RESET); // flash write strobe (active low)
	ide_ndelay(600);
	HAL_GPIO_WritePin(IDE_DIOW_GPIO_Port, IDE_DIOW_Pin, GPIO_PIN_SET); // release write strobe
	ide_set_bus_mode(GPIO_MODE_INPUT);
	ide_ndelay(600);
}

static int ide_ready() {
	uint16_t status = ide_register_read(REG_STATUS_COMMAND);
	int ready = status & 0b0000000001000000;
	int busy = status & 0b0000000010000000;
	return ready && !busy;
}

static int ide_drq() {
	uint16_t status = ide_register_read(REG_STATUS_COMMAND);
	return status & 0b0000000000001000;
}

void ide_init() {
	// never drive the bus unless when actually writing
	ide_set_bus_mode(GPIO_MODE_INPUT);
	// read/write strobes are active low -> normally keep them pulled high
	HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IDE_DIOW_GPIO_Port, IDE_DIOW_Pin, GPIO_PIN_SET);
	// now enable the level shifters, connecting the MCU and the IDE device
	HAL_GPIO_WritePin(TXS0108E_OE_GPIO_Port, TXS0108E_OE_Pin, GPIO_PIN_SET);
	// start device reset
	HAL_GPIO_WritePin(IDE_RESET_GPIO_Port, IDE_RESET_Pin, GPIO_PIN_RESET);
	// wait for the device to reset
	HAL_Delay(1);
	HAL_GPIO_WritePin(IDE_RESET_GPIO_Port, IDE_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(4);
	//HAL_GPIO_WritePin(IDE_RESET_GPIO_Port, IDE_RESET_Pin, GPIO_PIN_SET);
	//ide_register_write(REG_HEAD_DEVICE, 0b11100000); // select master device
	//while(!ide_ready());
}

void ide_main_loop() {
	/*int r1 = ide_register_read(REG_DATA);
	int r2 = ide_register_read(REG_ERROR_FEATURES);
	int r3 = ide_register_read(REG_SECTOR_COUNT);
	int r4 = ide_register_read(REG_LBA_LOW);
	int r5 = ide_register_read(REG_LBA_MID);
	int r6 = ide_register_read(REG_LBA_HIGH);
	int r7 = ide_register_read(REG_HEAD_DEVICE);
	int r8 = ide_register_read(REG_STATUS_COMMAND);*/

	HAL_GPIO_WritePin(IDE_RESET_GPIO_Port, IDE_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	ide_register_write(REG_HEAD_DEVICE, 0b11100000); // select master device

	while(!ide_ready());

	// set PIO mode 1 without IORDY
	ide_register_write(REG_SECTOR_COUNT, 0x01);
	ide_register_write(REG_ERROR_FEATURES, 0x03);
	ide_register_write(REG_STATUS_COMMAND, 0xEF);
	while(!ide_ready());

	uint16_t status;// = ide_register_read(REG_STATUS_COMMAND);

	ide_register_write(REG_CYL_LOW, 0);
	ide_register_write(REG_CYL_HIGH, 0);
	ide_register_write(REG_SECTOR_COUNT, 1);
	ide_register_write(REG_STATUS_COMMAND, 0x20);
	//ide_register_write(REG_STATUS_COMMAND, 0xEC);

	while(!ide_drq());
	uint8_t buf[512];
	for (int i = 0; i < 256; i++) {
		//while(!ide_drq());
		uint16_t data = ide_register_read(REG_DATA);
		/*HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_RESET); // flash read strobe (active low)
		uint16_t data = (uint16_t)((GPIOD->IDR & PORTD_BUS_IDR_MASK) | (GPIOE->IDR & PORTE_BUS_IDR_MASK));
		HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_SET); // release read strobe*/

		buf[i * 2] = (uint8_t) (data & 0x00FF);
		buf[i * 2 + 1] = (uint8_t) ((data & 0xFF00) >> 8);

		/*if (data == 0xff80 || data == 0x80ff || data == 0x00ff || data == 0xff00) {
			status = ide_register_read(REG_STATUS_COMMAND);
			int error = (status & 0b0000000000000001) ? 1 : 0;
			int pulse = (status & 0b0000000000000010) ? 1 : 0;
			int ecc   = (status & 0b0000000000000100) ? 1 : 0;
			int drq   = (status & 0b0000000000001000) ? 1 : 0;
			int skc   = (status & 0b0000000000010000) ? 1 : 0;
			int wft   = (status & 0b0000000000100000) ? 1 : 0;
			int ready = (status & 0b0000000001000000) ? 1 : 0;
			int busy  = (status & 0b0000000010000000) ? 1 : 0;
			HAL_Delay(1);
		}*/
	}


	status = ide_register_read(REG_STATUS_COMMAND);
	int error = (status & 0b0000000000000001) ? 1 : 0;
	int pulse = (status & 0b0000000000000010) ? 1 : 0;
	int ecc   = (status & 0b0000000000000100) ? 1 : 0;
	int drq   = (status & 0b0000000000001000) ? 1 : 0;
	int skc   = (status & 0b0000000000010000) ? 1 : 0;
	int wft   = (status & 0b0000000000100000) ? 1 : 0;
	int ready = (status & 0b0000000001000000) ? 1 : 0;
	int busy  = (status & 0b0000000010000000) ? 1 : 0;

	HAL_GPIO_WritePin(IDE_RESET_GPIO_Port, IDE_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
}
