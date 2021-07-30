#include "main.h"
#include "ide.h"

/* register addresses are in format CS0 CS1 A2 A1 A0 */
#define REG_DATA           0b01000
#define REG_ERROR_FEATURES 0b01001
#define REG_SECTOR_COUNT   0b01010
#define REG_SECTOR         0b01011
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

static uint32_t ide_current_bus_mode = GPIO_MODE_INPUT;

static inline void ide_set_bus_mode(uint32_t mode) {
	if (ide_current_bus_mode == mode) {
		return; // already in requested mode
	}
	ide_current_bus_mode = mode;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /*Configure GPIO pins : IDE_DD4_Pin IDE_DD5_Pin IDE_DD6_Pin IDE_DD7_Pin
	                           IDE_DD8_Pin IDE_DD9_Pin IDE_DD10_Pin IDE_DD11_Pin
	                           IDE_DD12_Pin IDE_DD13_Pin IDE_DD14_Pin IDE_DD15_Pin */
	GPIO_InitStruct.Pin = IDE_DD4_Pin|IDE_DD5_Pin|IDE_DD6_Pin|IDE_DD7_Pin
	                          |IDE_DD8_Pin|IDE_DD9_Pin|IDE_DD10_Pin|IDE_DD11_Pin
	                          |IDE_DD12_Pin|IDE_DD13_Pin|IDE_DD14_Pin|IDE_DD15_Pin;
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : IDE_DD0_Pin IDE_DD1_Pin IDE_DD2_Pin IDE_DD3_Pin */
	GPIO_InitStruct.Pin = IDE_DD0_Pin|IDE_DD1_Pin|IDE_DD2_Pin|IDE_DD3_Pin;
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
	int cycles = (ns / 6 + 1);
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

static uint16_t ide_register_read_once(uint8_t reg) {
	ide_set_bus_mode(GPIO_MODE_INPUT);
	ide_select_register(reg);
	HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_RESET); // flash read strobe (active low)
	ide_ndelay(600);
	uint16_t result = (uint16_t)((GPIOD->IDR & PORTD_BUS_IDR_MASK) | (GPIOE->IDR & PORTE_BUS_IDR_MASK));
	HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_SET); // release read strobe
	return result;
}

// buf size must be 2 * count
static void ide_register_read_multi(uint8_t reg, uint8_t* buf, int count) {
	ide_set_bus_mode(GPIO_MODE_INPUT);
	ide_select_register(reg);
	ide_ndelay(600);
	for (int i = 0; i < count; i++) {
		HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_RESET); // flash read strobe (active low)
		ide_ndelay(600);
		uint16_t data = (uint16_t)((GPIOD->IDR & PORTD_BUS_IDR_MASK) | (GPIOE->IDR & PORTE_BUS_IDR_MASK));
		HAL_GPIO_WritePin(IDE_DIOR_GPIO_Port, IDE_DIOR_Pin, GPIO_PIN_SET); // release read strobe
		buf[i * 2] = (uint8_t) (data & 0x00FF);
		buf[i * 2 + 1] = (uint8_t) ((data & 0xFF00) >> 8);
		ide_ndelay(600);
	}
}

static void ide_register_write_once(uint8_t reg, uint16_t word) {
	ide_set_bus_mode(GPIO_MODE_OUTPUT_PP);
	ide_select_register(reg);
	GPIOD->BSRR = ((~word & PORTD_BUS_BSRR_MASK) << 16) | (word & PORTD_BUS_BSRR_MASK);
	GPIOE->BSRR = ((~word & PORTE_BUS_BSRR_MASK) << 16) | (word & PORTE_BUS_BSRR_MASK);
	ide_ndelay(600);
	HAL_GPIO_WritePin(IDE_DIOW_GPIO_Port, IDE_DIOW_Pin, GPIO_PIN_RESET); // flash write strobe (active low)
	ide_ndelay(600);
	HAL_GPIO_WritePin(IDE_DIOW_GPIO_Port, IDE_DIOW_Pin, GPIO_PIN_SET); // release write strobe
	ide_ndelay(600);
	ide_set_bus_mode(GPIO_MODE_INPUT);
}

static void ide_error() {
	uint16_t error = ide_register_read_once(REG_ERROR_FEATURES);
	int amnf  = error & 0b00000001;
	int tk0nf = error & 0b00000010;
	int abrt  = error & 0b00000100;
	int mcr   = error & 0b00001000;
	int idnf  = error & 0b00010000;
	int mc    = error & 0b00100000;
	int unc   = error & 0b01000000;

	HAL_GPIO_WritePin(IDE_RESET_GPIO_Port, IDE_RESET_Pin, GPIO_PIN_RESET);
	while(1) {
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		HAL_Delay(100);
	}
}

int ide_ready() {
	uint16_t status = ide_register_read_once(REG_STATUS_COMMAND);
	if (status & 1) ide_error();
	int ready = status & 0b0000000001000000;
	int busy = status & 0b0000000010000000;
	return ready && !busy;
}

static int ide_drq() {
	uint16_t status = ide_register_read_once(REG_STATUS_COMMAND);
	if (status & 1) ide_error();
	return status & 0b0000000000001000;
}

static void ide_set_lba(uint32_t lba, uint16_t sector_count) { // 28 bit lba
	ide_register_write_once(REG_HEAD_DEVICE, (uint8_t)((lba & 0x0F000000) >> 24) | 0b11100000); // master device, LBA mode, lba most significant 4 bits
	ide_register_write_once(REG_CYL_HIGH,    (uint8_t)((lba & 0x00FF0000) >> 16));
	ide_register_write_once(REG_CYL_LOW,     (uint8_t)((lba & 0x0000FF00) >> 8));
	ide_register_write_once(REG_SECTOR,      (uint8_t)((lba & 0x000000FF)));
	ide_register_write_once(REG_SECTOR_COUNT, sector_count);
}

static void ide_reset() {
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
	ide_register_write_once(REG_HEAD_DEVICE, 0b11100000); // select master device and LBA mode
	// set PIO mode 1 without IORDY
	/*ide_register_write(REG_SECTOR_COUNT, 0x01);
	ide_register_write(REG_ERROR_FEATURES, 0x03);
	ide_register_write(REG_STATUS_COMMAND, 0xEF);*/
	while(!ide_ready());
}

static void ide_identify_device(uint16_t* buf) {
	ide_reset();
	while(!ide_ready());
	ide_register_write_once(REG_STATUS_COMMAND, 0xEC);
	while(!ide_drq());
	for (int i = 0; i < 256; i++) {
		buf[i]= ide_register_read_once(REG_DATA);
	}
}

void ide_init() {
	ide_reset();
}

int ide_get_num_sectors() {
	uint16_t buf[256];
	ide_identify_device(buf);
	return ((uint32_t)(buf[61]) << 16) | ((uint32_t)(buf[60]));
}

void ide_read_sectors(uint32_t lba, uint8_t* buf, uint16_t num_sectors) {
	while(!ide_ready());
	ide_set_lba(lba, num_sectors);
	ide_register_write_once(REG_STATUS_COMMAND, 0x20);
	while(!ide_drq());
	/*for (int i = 0; i < 256 * num_sectors; i++) {
		uint16_t data = ide_register_read_once(REG_DATA);
		buf[i * 2] = (uint8_t) (data & 0x00FF);
		buf[i * 2 + 1] = (uint8_t) ((data & 0xFF00) >> 8);
	}*/
	ide_register_read_multi(REG_DATA, buf, 256 * num_sectors);
}

void ide_main_loop() {
	/*uint8_t buf[512];
	ide_read_sectors(0, buf, 1);
	uint16_t status = ide_register_read(REG_STATUS_COMMAND);
	int error = (status & 0b0000000000000001) ? 1 : 0;
	int pulse = (status & 0b0000000000000010) ? 1 : 0;
	int ecc   = (status & 0b0000000000000100) ? 1 : 0;
	int drq   = (status & 0b0000000000001000) ? 1 : 0;
	int skc   = (status & 0b0000000000010000) ? 1 : 0;
	int wft   = (status & 0b0000000000100000) ? 1 : 0;
	int ready = (status & 0b0000000001000000) ? 1 : 0;
	int busy  = (status & 0b0000000010000000) ? 1 : 0;
	HAL_Delay(1000);*/

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	ide_ndelay(6000);
	return;
}
