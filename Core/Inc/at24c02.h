#ifndef __AT24C02_H__
#define __AT24C02_H__

#include "main.h"
#include "i2c.h"

// 星火开发板 AT24C02 I2C 硬件地址 (A0/A1/A2接地，7位地址0x50，写地址0xA0，读地址0xA1)
#define AT24C02_DEV_ADDR   0xA0

uint8_t AT24C02_Write_Byte(uint8_t mem_addr, uint8_t data);
uint8_t AT24C02_Read_Byte(uint8_t mem_addr, uint8_t *data);

#endif
