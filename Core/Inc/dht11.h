#ifndef __DHT11_H__
#define __DHT11_H__

#include "main.h"

// 星火开发板 DHT11 引脚连接在 PC13
#define DHT11_GPIO_PORT       GPIOC
#define DHT11_PIN             GPIO_PIN_13

// 极速单周期寄存器操作宏 (避免函数调用开销，确保微秒时序绝对精准)
#define DHT11_DQ_OUT_HIGH()   (GPIOC->BSRR = GPIO_PIN_13)
#define DHT11_DQ_OUT_LOW()    (GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16)
#define DHT11_DQ_IN()         ((GPIOC->IDR & GPIO_PIN_13) != 0)

void delay_us(uint32_t nus);
void DHT11_IO_OUT(void);
void DHT11_IO_IN(void);
uint8_t DHT11_Init(void);
void DHT11_Rst(void);
uint8_t DHT11_Check(void);
uint8_t DHT11_Read_Bit(void);
uint8_t DHT11_Read_Byte(void);
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi);

#endif
