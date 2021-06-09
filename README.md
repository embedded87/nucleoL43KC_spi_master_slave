# nucleoL43KC_spi_master_slave
STM32L4(Master) communicating with another STM32L4(Slave) over SPI. Master take commands over serial for read/write data on the slave. Refer to document for further details.
1. Project Overview
Master will take commands over serial and Transfer it to Slave over SPI 
The slave processes the commands and replies back to the master. Slaves have two sensors led and temperature. The project has a provision of extending to various sensors on slave and master. 
 Peripheral used 

Frame structure

USART
SYNC_B1
SYNC_B2
CMD
SENSOR
DATA
CRC_H
CRC_l

*crc for future development 
SPI
SLAVE_ADD
R
CMD
SENSOR
DATA
DATE_2
R



2. Software
Prerequisite
STM32 CubeIDE. STM32CubeMX, ARM GCC compiler. STM32L4 libraries.
