#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "stm32f4xx.h"

#define GPIOBEN					(1U<<1)
#define I2C1EN					(1U<<21)

#define I2C_100KHZ					80 //0B 0101 0000 = Decimal = 80
#define I2C_400KHZ					26
#define I2C_FAST_DUTY			((1U << 14) | (1U << 15))
#define SD_MODE_MAX_RISE_TIME		17
#define FS_MODE_MAX_RISE_TIME		5
#define CR1_PE					(1U<<0)

#define SR2_BUSY				(1U<<1)
#define CR1_START				(1U<<8)
#define	SR1_SB					(1U<<0)
#define	SR1_ADDR				(1U<<1)
#define	SR1_TXE					(1U<<7)
#define	CR1_ACK					(1U<<10)
#define CR1_STOP				(1U<<9)
#define	SR1_RXNE				(1U<<6)
#define	SR1_BTF					(1U<<2)
#define SR1_AF					(1U<<10)

void I2C1_Init(void);
void I2C1_byteRead(char saddr, char maddr, char* data);
void I2C1_burstRead(char saddr, char maddr, int n, char* data);
void I2C1_burstWrite(char saddr, char maddr, int n, char* data);

#endif /* I2C_H_ */
