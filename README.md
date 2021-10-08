Implementation of an USB-IDE/PATA converter on a STM32F4DISCOVERY dev board
(based on STM32F407VG microcontroller); converts an old IDE hard drive in a
standard USB mass storage device, thus no drivers are required on the PC
side. USB is limited to 1.1, top transfer speed reached is about 1.1MB/s.

Based on FreeRTOS and [TinyUSB](https://github.com/hathach/tinyusb);
the IDE/PATA driver is written from scratch using GPIOs, inspired by similar
projects on the [8051](https://www.pjrc.com/tech/8051/ide/) and
[Raspberry PI](https://github.com/Manawyrm/pata-gpio).

The board is mounted on a custom PCB which can be found
[here](https://github.com/gianluca-nitti/ide-usb-pcb).

Developed as exam project for the [Progettazione di Sistemi Operativi](https://labonline.ctu.unimi.it/course/info.php?id=215)
course @ Università degli Studi di Milano.

See `report.pdf` and `slides.pdf` for documentation (Italian).
