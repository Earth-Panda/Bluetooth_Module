#include "i2c.h"

//#define GPIOBEN					(1U<<1)
//#define I2C1EN					(1U<<21)
//
//#define I2C_100KHZ					80 //0B 0101 0000 = Decimal = 80
//#define SD_MODE_MAX_RISE_TIME		17
//#define CR1_PE					(1U<<0)
//
//#define SR2_BUSY				(1U<<1)
//#define CR1_START				(1U<<8)
//#define	SR1_SB					(1U<<0)
//#define	SR1_ADDR				(1U<<1)
//#define	SR1_TXE					(1U<<7)
//#define	CR1_ACK					(1U<<10)
//#define CR1_STOP				(1U<<9)
//#define	SR1_RXNE				(1U<<6)
//#define	SR1_BTF					(1U<<2)
//#define SR1_AF					(1U<<10)


void I2C1_Init(void)
{
	/*Enable clock access to GPIOB*/
	RCC->AHB1ENR |= GPIOBEN;

	/*Set PB8 and PB9 mode to alternate function*/
	GPIOB->MODER &= ~(1U<<16);
	GPIOB->MODER |= (1U<<17);

	GPIOB->MODER &= ~(1U<<18);
	GPIOB->MODER |= (1U<<19);

	/*Set PB8 and PB9 output type to open drain*/
	GPIOB->OTYPER |= (1U<<8);
	GPIOB->OTYPER |= (1U<<9);

	/*Enable Pull-up for PB6 and PB7*/
	GPIOB->PUPDR |= (1U<<16);
	GPIOB->PUPDR &= ~(1U<<17);

	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &= ~(1U<<19);

	/*Set PB8 and PB9 alternate function type to I2C (AF4)
	 * PB8 --> SCL
	 * PB9 --> SDA*/
	GPIOB->AFR[1] &= ~(1U<<0);
	GPIOB->AFR[1] &= ~(1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &= ~(1U<<3);

	GPIOB->AFR[1] &= ~(1U<<4);
	GPIOB->AFR[1] &= ~(1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &= ~(1U<<7);

	/***Configuring I2C1***/

	/*Enable clock access to I2C1*/
	RCC->APB1ENR |= I2C1EN;

	/*Enter reset mode  */
	I2C1->CR1 |= (1U<<15);

	/*Come out of reset mode  */
	I2C1->CR1 &= ~(1U<<15);

	/*Set Peripheral clock frequency*/
	I2C1->CR2 = (1U<<4);   //16 Mhz

//	/*Set I2C to standard mode, 100kHz clock */
//	I2C1->CCR = I2C_100KHZ; //CCR = PCLK/(2*12C_FREQ) = 16MHz/(2*100KHz) = 80
//
//	/*Set rise time */
//	I2C1->TRISE = SD_MODE_MAX_RISE_TIME; //(1000ns/(1/16MHz)+1 = 17

	/*Set I2C to fast mode, 400kHz clock */
	I2C1->CCR = 1; //Refer to ST reference manual for derivation
	I2C1->CCR = I2C_FAST_DUTY; //Set to Fast mode and duty cycle of 16/9

	/*Set rise time */
	I2C1->TRISE = 6;//FS_MODE_MAX_RISE_TIME; //(300ns/(1/16MHz)+1 = 5.8

	/*Enable I2C1 module */
	I2C1->CR1 |= CR1_PE;
}

void I2C1_byteRead(char saddr, char maddr, char* data) {

	  volatile int tmp;

	  /* Wait until bus not busy */
	  while (I2C1->SR2 & (SR2_BUSY)){}

	  /* Generate start condition */
	  I2C1->CR1 |= CR1_START;

	  /* Wait until start condition flag is set, note the negation */
	  /* Stay in loop if start flag is not set */
	  while (!(I2C1->SR1 & (SR1_SB))){}

	  /* Transmit slave address + Write (0) */
	  /* This slave address is sent to each slave device along with an indication to establish either read or write communication */
	  /* Yo device A, I want to write to one of your memory registers */
	  I2C1->DR = saddr << 1; // shifting the address left makes room for the R/W bit, which is 0 for write

	  /* Wait until address flag is set */
	  /* Each slave device compares the address from master to its own address and sends an ACK if matched */
	  /* I'm device A and of course you can write to one of my registers */
	  while (!(I2C1->SR1 & (SR1_ADDR))){}

	  /* Clear address flag by reading SR2 register */
	  tmp = I2C1->SR2;

	  /*Wait until transmitter empty */
	  while (!(I2C1->SR1 & SR1_TXE)){}

	  /* Send register address */
	  /* Aight, here is the address of the register I'm want to write to read from*/
	  I2C1->DR = maddr;

	  /*Wait until transmitter empty */
	  /* No probs */
	  while (!(I2C1->SR1 & SR1_TXE)){}

	 /* Begin actual reading process once slave device and register of interest are understood between master and slave*/
	 /*Generate restart */
	 I2C1->CR1 |= CR1_START;

	 /* Wait until start flag is set */
	 while (!(I2C1->SR1 & SR1_SB)){}

	 /* Transmit slave address + Read */
	 I2C1->DR = saddr << 1 | 1;

	 /* Wait until addr flag is set */
	 while (!(I2C1->SR1 & (SR1_ADDR))){}

	 /* Clear addr flag */
	 tmp = I2C1->SR2;

	 /* Disable Acknowledge */
	 I2C1->CR1 &= ~CR1_ACK;

	 /* Generate stop after data received */
	 I2C1->CR1 |= CR1_STOP;

	 /* Wait until RXNE flag is set
	  * Wait until receiver is not empty (has contents to read)*/
	 while (!(I2C1->SR1 & SR1_RXNE)){}

	 /* Read data from DR */
	 *data++ = I2C1->DR;
}

void I2C1_burstRead(char saddr, char maddr, int n, char* data) {

	  volatile int tmp;

	  /* Wait until bus not busy */
	  while (I2C1->SR2 & (SR2_BUSY)){}

	  /* Generate start condition */
	  I2C1->CR1 |= CR1_START;

	  /* Wait until start condition flag is set, note the negation */
	  /* Stay in loop if start flag is not set */
	  while (!(I2C1->SR1 & (SR1_SB))){}

	  /* Transmit slave address + Write (0) */
	  /* This slave address is sent to each slave device along with an indication to establish either read or write communication */
	  I2C1->DR = saddr << 1; // shifting the address left makes room for the R/W bit, which is 0 for write

	  /* Wait until address flag is set */
	  /* Each slave device compares the address from master to its own address and sends an ACK if matched */
	  while (!(I2C1->SR1 & (SR1_ADDR))){}

	  /* Clear address flag by reading SR2 register */
	  tmp = I2C1->SR2;

	  /*Wait until transmitter empty */
	  while (!(I2C1->SR1 & SR1_TXE)){}

	  /* Send register address */
	  I2C1->DR = maddr;

	  /*Wait until transmitter empty */
	  while (!(I2C1->SR1 & SR1_TXE)){}

	 /* Begin actual reading process once slave device and register of interest are understood between master and slave*/
	 /*Generate restart */
	 I2C1->CR1 |= CR1_START;

	 /* Wait until start flag is set */
	 while (!(I2C1->SR1 & SR1_SB)){}

	 /* Transmit slave address + Read */
	 I2C1->DR = saddr << 1 | 1;

	 /* Wait until addr flag is set */
	 while (!(I2C1->SR1 & (SR1_ADDR))){}

	 /* Clear addr flag */
	 tmp = I2C1->SR2;

	 /* Enable Acknowledge */
	 I2C1->CR1 |= CR1_ACK;

	 while(n > 0U)
	 {
		 /*if one byte*/
		 if(n == 1U)
		 {
			 /* Disable Acknowledge */
			 I2C1->CR1 &= ~CR1_ACK;

			 /* Generate Stop */
			 I2C1->CR1 |= CR1_STOP;

			 /* Wait for RXNE flag set */
			 while (!(I2C1->SR1 & SR1_RXNE)){}

			 /* Read data from DR */
			 *data++ = I2C1->DR;
			 break;
		 }
		 else
		 {
			 /* Wait until RXNE flag is set */
			 while (!(I2C1->SR1 & SR1_RXNE)){}

			 /* Read data from DR */
			 (*data++) = I2C1->DR;

			 n--;
		 }
	 }

}

void I2C1_burstWrite(char saddr, char maddr, int n, char* data) {

	/* Temporary variable to read SR2*/
	volatile int tmp;

	/* Wait until bus not busy */
	while (I2C1->SR2 & (SR2_BUSY)){}

	/* Generate start condition */
	I2C1->CR1 |= CR1_START;

	/* Wait until start condition flag is set, note the negation */
	/* Stay in loop if start flag is not set*/
	while (!(I2C1->SR1 & (SR1_SB))){}

	/* Transmit slave address + Write (0) */
	/* This slave address is sent to each slave device along with an indication to establish either read or write communication*/
	/* Yo device A, I want to write to one of your memory registers */
	I2C1->DR = saddr << 1; // shifting the address left makes room for the R/W bit, which is 0 for write

	/* Wait until address flag is set */
	/* Each slave device compares the address from master to its own address and sends an ACK if mathced*/
	/* I'm device A and of course you can write to one of my registers */
	while (!(I2C1->SR1 & (SR1_ADDR))){}

	/* Clear address flag by reading SR2 register */
	tmp = I2C1->SR2;

	/*Wait until data register empty */
	while (!(I2C1->SR1 & SR1_TXE)){}

	/* Send register address */
	/* aight here is the register I'm writing to write to and the value I want written */
	I2C1->DR = maddr;

	for (int i = 0; i < n; i++) {

		/*Wait until data register empty */
		while (!(I2C1->SR1 & SR1_TXE)){}

		/* Transmit memory address */
		I2C1->DR = *data++;
	}
	/* Wait until transfer finished */
	while (!(I2C1->SR1 & (SR1_BTF))){}

	/* Generate stop */
	I2C1->CR1 |= CR1_STOP;
}
