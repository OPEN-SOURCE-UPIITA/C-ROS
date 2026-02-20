#ifndef __IIC_H_
#define __IIC_H_

#include <stdint.h>


void IIC_init();
int IIC_WriteToMem(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t length);
int IIC_Write(uint8_t address, uint8_t *data, uint8_t length);
int IIC_ReadFromMem(uint8_t address, uint8_t reg_addr, uint8_t *buf, uint8_t length);

#endif

