#include "mux.h"


//Returns TRUE if the device sends an ACK indicating successful acknowledgment of the address byte
bool muxConnected()
{
	uint32_t timeout = 1000; //Timeout to connect set to 1000ms
	unsigned long startTime = getMillis();

	while ((getMillis() - startTime) < timeout)
	{
		volatile int tmp;

		/* Wait until bus not busy */
		while (I2C1->SR2 & (SR2_BUSY)){}

		/* Generate start condition */
		I2C1->CR1 |= CR1_START;

		/* Wait until start condition flag is set */
		while (!(I2C1->SR1 & (SR1_SB))){}

		/* Transmit slave address + Write (0) */
		I2C1->DR = MUX_WRITE_ADDR;

		/* Wait until address flag is set */
		while (!(I2C1->SR1 & (SR1_ADDR))){}

		/* Clear address flag by reading SR2 register */
		tmp = I2C1->SR2;

		if (I2C1->SR1 & SR1_AF)
		{
			//No ACK received, address not acknowledged
			I2C1->CR1 |= CR1_STOP;  // Send STOP condition
			continue;
		}

		//Address acknowledged, device is connected
		I2C1->CR1 |= CR1_STOP; // Send STOP condition
		return true;
	}
	//Timeout to connect has expired, hence device is not connected
	return false;
}

//If using reconfigured address selector pins from default
void selectMux_and_control(uint8_t mux_address, uint8_t control_byte)
{
	uint8_t bufferSize = 1;
	char data[bufferSize];

	data[0] = control_byte;
	I2C1_burstWrite(mux_address, 0, bufferSize, data);
}

//Default Mux address of 0x70
void enableChannel(uint8_t channel)
{
	disableChannels();
	selectMux_and_control(MUX_ADDR, channel);
}

//Disables all channels
void disableChannels()
{
	selectMux_and_control(MUX_ADDR, RESET);
}

