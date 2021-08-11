#include <string.h> // for memcpy
#include "main.h"
#include "cmsis_os.h"

#include "ide.h"
#include "ide_async.h"

#define CACHE_NUM_SECTORS 32
#define CACHE_SIZE (512 * CACHE_NUM_SECTORS)

enum {
	STATE_NOT_INITIALIZED,
	STATE_READY,
	STATE_READING,
	STATE_WRITING
} state = STATE_NOT_INITIALIZED;

static uint32_t capacity_in_sectors;


static uint32_t requested_lba = 0;
//static uint32_t max_sectors;
static uint8_t cache[CACHE_SIZE];
static uint32_t cache_len = 0;
static osMutexId_t requestParamLock;
static osSemaphoreId_t requestSem;


static inline uint32_t min(uint32_t a, uint32_t b) {return a < b ? a : b;}

uint32_t ide_async_get_num_sectors() {
	while (state == STATE_NOT_INITIALIZED) {
		osThreadYield();
	}
	return capacity_in_sectors;
}

int32_t ide_async_read(uint32_t lba, uint32_t offset, uint8_t* buf, uint32_t buf_size) {
	if ((state == STATE_READING || (state == STATE_READY && cache_len > 0)) && lba >= requested_lba) {
		uint32_t delta_lba = lba - requested_lba;
		if (delta_lba < CACHE_NUM_SECTORS) {
			uint32_t delta_bytes = delta_lba * 512;
			if ((delta_bytes + offset) < cache_len) {
				uint32_t len = min(cache_len - delta_bytes - offset, buf_size);
				memcpy(buf, cache + delta_bytes + offset, len);
				return len;
			}
			osThreadYield();
			return 0;
		}
	}

	osMutexAcquire(requestParamLock, osWaitForever);
	requested_lba = lba;
	//max_sectors = buf_size / 512;
	cache_len = 0;
	state = STATE_READING;
	osMutexRelease(requestParamLock);
	while(osOK != osSemaphoreRelease(requestSem));
	return 0;
}

int32_t ide_async_write(uint32_t lba, uint32_t offset, uint8_t* buf, uint32_t buf_size) {
	if (offset != 0) return 0;
	if (state == STATE_READY) {
		osMutexAcquire(requestParamLock, osWaitForever);
		memcpy(cache, buf, buf_size);
		requested_lba = lba;
		cache_len = buf_size;
		state = STATE_WRITING;
		osMutexRelease(requestParamLock);
		while(osOK != osSemaphoreRelease(requestSem));
		return buf_size;
	}

	osThreadYield();
	return 0;
}

void ide_async_init() {
	requestParamLock = osMutexNew(NULL);
	requestSem = osSemaphoreNew(1, 0, NULL);
	ide_init();
	capacity_in_sectors = ide_get_num_sectors();
	state = STATE_READY;
}

void ide_async_main_loop_step() {
	osSemaphoreAcquire(requestSem, osWaitForever);
	osMutexAcquire(requestParamLock, osWaitForever);
	if (state == STATE_READING) {
		uint16_t num_sectors = min(CACHE_NUM_SECTORS, capacity_in_sectors - requested_lba);
		//uint16_t num_sectors = min(CACHE_NUM_SECTORS, max_sectors);
		ide_begin_read_sectors(requested_lba, num_sectors);
		for (int i = 0; i < num_sectors; i++) {
			ide_read_next_sector(cache + cache_len);
			cache_len += 512;
			osThreadYield();
		}
		state = STATE_READY;
	} else if (state == STATE_WRITING) {
		uint16_t num_sectors = cache_len / 512;
		ide_begin_write_sectors(requested_lba, num_sectors);
		for (int i = 0; i < num_sectors; i++) {
			ide_write_next_sector(cache + (i * 512));
		}
		state = STATE_READY;
	}
	osMutexRelease(requestParamLock);
}
