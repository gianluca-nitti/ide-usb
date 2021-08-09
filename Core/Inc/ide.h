#ifndef _IDE_H
#define _IDE_H

#include <stdint.h>

void ide_init();
int ide_ready();
int ide_get_num_sectors();
void ide_begin_read_sectors(uint32_t lba, uint16_t num_sectors);
void ide_read_next_sector(uint8_t* buf);
void ide_begin_write_sectors(uint32_t lba, uint16_t num_sectors);
void ide_write_next_sector(uint8_t* buf);

#endif
