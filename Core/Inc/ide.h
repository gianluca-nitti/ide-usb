#ifndef _IDE_H
#define _IDE_H

#include <stdint.h>

void ide_init();
int ide_ready();
int ide_get_num_sectors();
void ide_read_sectors(uint32_t lba, uint8_t* buf, uint16_t num_sectors);

void ide_main_loop(); // TODO remove

#endif
