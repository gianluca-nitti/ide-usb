#ifndef _IDE_ASYNC_H
#define _IDE_ASYNC_H

// Client API
uint32_t ide_async_get_num_sectors();
int32_t ide_async_read(uint32_t lba, uint32_t offset, uint8_t* buf, uint32_t buf_size);
int32_t ide_async_write(uint32_t lba, uint32_t offset, uint8_t* buf, uint32_t buf_size);

// "OS" API
void ide_async_init();
void ide_async_main_loop_step();

#endif
