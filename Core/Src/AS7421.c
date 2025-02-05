#include <AS7421.h>

uint8_t config_values[8] = {0x21, 0x21, 0x21, 0x21, 0x43, 0x43, 0x43, 0x43};

static uint8_t AS7421_readRegister(uint8_t addr);
static void AS7421_readRegisters(uint8_t addr, uint8_t bufferSize, uint8_t *data);
static void AS7421_writeRegister(uint8_t addr, uint8_t val);
static void AS7421_writeRegisters(uint8_t startAddr, uint8_t bufferSize, uint8_t *val);
static uint16_t byteSwap16(uint16_t value);

//Reads from a given location from the AS7421
static uint8_t AS7421_readRegister(uint8_t addr)
{
	char data = 0;
	I2C1_byteRead(AS7421_ADDR, addr, &data);
	return data;
}

//Reads from consecutive register locations on the AS7421
static void AS7421_readRegisters(uint8_t addr, uint8_t bufferSize, uint8_t *data)
{
	I2C1_burstRead(AS7421_ADDR, addr, bufferSize, (char *)data);
}

//Write a value to a given location on the AS7421
static void AS7421_writeRegister(uint8_t addr, uint8_t val)
{
	uint8_t bufferSize = 1;
	char data[bufferSize];

	data[0] = val;
	I2C1_burstWrite(AS7421_ADDR, addr, bufferSize, data);
}

//Write values to consecutive register locations on the AS7421
static void AS7421_writeRegisters(uint8_t startAddr, uint8_t bufferSize, uint8_t *val)
{
//	char data[bufferSize];
//
//	for (int i = 0; i < bufferSize; i++)
//	{
//		data[i] = val[i];
//	}
	I2C1_burstWrite(AS7421_ADDR, startAddr, bufferSize, (char *)val);
}

static uint16_t byteSwap16(uint16_t value) {
    return (value >> 8) | (value << 8);
}

// Enable FPU
void fpu_enable()
{
	/*Enable Floating Point Unit: Enable CP10 and CP11 full access*/
	SCB->CPACR |= (1U<<20);
	SCB->CPACR |= (1U<<21);
	SCB->CPACR |= (1U<<22);
	SCB->CPACR |= (1U<<23);
}

//Returns TRUE if the device sends an ACK indicating it is connected
bool isConnected()
{
	uint32_t timeout = 1000; //Timeout to connect set to 1000ms
	unsigned long startTime = HAL_GetTick();

	while ((HAL_GetTick() - startTime) < timeout)
	{
		volatile int tmp;

		/* Wait until bus not busy */
		while (I2C1->SR2 & (SR2_BUSY)){}

		/* Generate start condition */
		I2C1->CR1 |= CR1_START;

		/* Wait until start condition flag is set */
		while (!(I2C1->SR1 & (SR1_SB))){}

		/* Transmit slave address + Write (0) */
		I2C1->DR = AS7421_WRITE_ADDR;

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

//Initializes the sensor with basic settings
//Returns false if sensor is not detected
bool initialize()
{
	uart2_rxtx_init();
	I2C1_Init();
	fpu_enable();

	if (isConnected() == false)
	{
		return false; //Check for sensor presence
	}

	//Add initialization functions
	configueLEDWait(false); //False is to enable LED wait time between cycles
	configureWaitCycle(true);
	setInterLED(2);
	setLTF_CCOUNT(1023);

	configureLEDAuto(false);
	setWaitTime(10);
	setIntegrationTime(20);
	numMeasurements(CONTINUOUS);
	setIntegrationMode(INTEGRATIONCYLE_ABCD);
	configureAutozero(true, AZ_WTIME_128US, 0, 1);
//	configueLEDWait(false);

	return true;
}

void startup()
{
	bool works = initialize();
	if (works)
	{
		setIntegrationTime(65.5); //65.5
		setWaitTime(5);

		sleep();
		powerup();
		configureSMUX();
		configureGain(8); //Sets gain for all ADCs // 8 max
		configureLEDs(true, ALL_LEDS, LED_CURRENT_LIMIT_75MA); //75 max
	}
	else
	{
		printf("Sensor failed to respond \n\r");
		return;
	}
}

// Perform Measurements
void performMeasurements(uint16_t arrSpectra[CHANNELSIZE], uint16_t arrTemp[TEMPSIZE])
{
	unsigned long startTime = HAL_GetTick();
	while (getMeasurementStatus(ADATA) == 0){} //End of measurement, new measurement data can be read if true

	unsigned long endTime = HAL_GetTick();
//	printf("\nTIme to get data: %ld \n\r", endTime - startTime);

	getAllSpectralData(arrSpectra); //Reading spectral data channels and passing organized values into arrSpectra
	getAllTemperatureData(arrTemp); //Reading temperatures of integration cycles A to D
}

//Configure LED_WAIT_OFF or Disable LED_WAIT_OFF to modify waiting time between integration cycle A to D
void configueLEDWait(bool setting)
{
	uint8_t value = AS7421_readRegister(CFG_MISC); //Read existing state
	if (setting)
	{
		value |= (1U << 2); //Set LED_WAIT_OFF bit (bit 2)
	}
	else
	{
		value &= ~(1U << 2); //Reset LED_WAIT_OFF bit (bit 2)
	}

	AS7421_writeRegister(CFG_MISC, value); //Write value to CFG_MISC register to configure the LED_WAIT_OFF bit
}

// Set Wait time (LED_WAIT) between switching on the LED and begin of integration/modulation. Wait time = 1024us * waitTime
void setInterLED(uint8_t waitTime)
{
	if (waitTime > 255)
	{
        waitTime = 255; // Set to maximum value if out of range
	}

	AS7421_writeRegister(LED_WAIT, waitTime);
}

//Configure the waiting time between integration cycle A to D (programmable with LTF_WTIME)
void configureWaitCycle(bool setting)
{
	uint8_t value = AS7421_readRegister(CFG_MISC); //Read existing state
	if (setting)
	{
		value |= (1U << 1); //Set WAIT_CYCLE_ON bit (bit 1)
	}
	else
	{
		value &= ~(1U << 1); //Reset WAIT_CYCLE_ON bit (bit 1)
	}

	AS7421_writeRegister(CFG_MISC, value); //Write value to CFG_MISC register to configure the WAIT_CYCLE_ON bit
}

//Programs the wait time (WTIME) in ms between two consecutive spectral measurements
void setWaitTime(uint32_t waitTime)
{
	uint32_t waitCounts = ((waitTime * F_CLKMOD) / 1000) - 1;

	uint8_t data[3] = {0};
	data[0] = waitCounts & 0xFF; //low byte
	data[1] = (waitCounts >> 8) & 0xFF; //mid byte
	data[2] = (waitCounts >> 16) & 0xFF; //high byte

	AS7421_writeRegisters(LTF_WTIME, 3, data);
}

void setLTF_CCOUNT(uint16_t ccount_value)
{
    // Validate the input value
    if (ccount_value > 0xFFFF) {
        ccount_value = 0xFFFF;  // Cap to maximum 16-bit value
    }
	uint8_t data[2] = {0};
	ccount_value = byteSwap16(ccount_value);

	data[0] = ccount_value & 0xFF; //low byte
	data[1] = (ccount_value >> 8) & 0xFF; //high byte

    // Write the value to the LTF_CCOUNT register
    AS7421_writeRegisters(LTF_CCOUNT, 2, data);
}

//Controls NIR light source during spectral measurement
void configureLEDAuto(bool mode)
{
	uint8_t value = AS7421_readRegister(ENABLE); //Read existing state
	if (mode)
	{
		value |= (1U << 4); //Set LED_AUTO bit (bit 4 and 5)
		value |= (1U << 5);
	}
	else
	{
		value &= ~(1U << 4); //Reset LED_AUTO bit (bit 4 and 5)
		value &= ~(1U << 5);
	}

	AS7421_writeRegister(ENABLE, value);
}

//Programs the integration time (ITIME) in ms of the LTF converter
void setIntegrationTime(uint32_t intTime)
{
//	if (intTime > 256)
//	{
//		intTime = 256;
//	}
	uint32_t intCounts = ((intTime * F_CLKMOD) / 1000) - 1;

	uint8_t data[3] = {0};
	data[0] = intCounts & 0xFF;
	data[1] = (intCounts >> 8) & 0xFF;
	data[2] = (intCounts >> 16) & 0xFF;

	AS7421_writeRegisters(LTF_ITIME, 3, data);
}

void numMeasurements(uint8_t counts) //specifying number of measurements
{
	if (counts > 255)
	{
		counts = 255; // Set to maximum value if out of range
	}

	AS7421_writeRegister(LTF_ICOUNT, counts);
}

//16 channels (A), 32 channels (AB), 48 channels (ABC), 64 channels (ABCD)
void setIntegrationMode(uint8_t mode)
{
	if (mode > INTEGRATIONCYLE_ABCD)
	{
		mode = INTEGRATIONCYLE_ABCD; //Limit mode to 2 bits
	}

	uint8_t value = AS7421_readRegister(CFG_LTF); //Read existing state
	value &= 0b11100111; //Clear LTF_CYCLE bits
	value |= (mode << 3); //Set LTF_CYCLE bits with user's choice

	AS7421_writeRegister(CFG_LTF, value);
}


void configureAutozero(bool enable, uint8_t az_waitTime, uint8_t iteration, uint8_t cycle)
{
	uint8_t value = AS7421_readRegister(CFG_AZ);

	value |= (1U << 7); //Enable AZ_ON (bit 7)
	if (az_waitTime > 0b11)
	{
		az_waitTime = 0b11;
	}
    value &= ~(0b11 << 5);  // Clear the AZ_WTIME bits (bits 5 and 6)
	value |= (az_waitTime << 5); //Set AZ_WTIME (bit 5 and 6)

    value &= ~(1U << 4);  // Clear the AZ_EN bit
	value |= (enable << 4); //Set AZ_EN (bit 4)

	value &= ~(1U << 3);  // Clear the AZ_CYCLE bit
	value |= (cycle << 3); //Set AZ_CYCLE (bit 3)

	if (iteration > 0b111)
	{
		iteration = 0b111;
	}
    value &= ~0b111;  // Clear the iteration bits (bits 0, 1, and 2)
	value |= iteration;

    AS7421_writeRegister(CFG_AZ, value);
}

// Internal oscillator enabled, potentially write 0x44 to register 0x6F, 0x20 to register 0x6E, 0x00 to register 0x6F, sensor is in idle state
void powerup()
{
	uint8_t value = AS7421_readRegister(ENABLE); //Read existing state
	value |= (1U << 0); //Set PON (bit 0)
    AS7421_writeRegister(ENABLE, value);

    //After power on reset the following commands have to be written prior accessing other registers
    AS7421_writeRegister(0x6F, 0x44);
    AS7421_writeRegister(0x6E, 0x20);
    AS7421_writeRegister(0x6F, 0x00);
}

// Reset
void reset()
{
	uint8_t value = AS7421_readRegister(CFG_MISC); //Reading current state
	value |= (1U << 0); //Setting SW_RST (bit 0)
	AS7421_writeRegister(CFG_MISC, value);
}

// Internal oscillator disabled, sensor is in sleep state
void sleep()
{
	uint8_t value = AS7421_readRegister(ENABLE); //Read existing state
	value &= ~(1U << 0); //Reset PON (bit 0)
    AS7421_writeRegister(ENABLE, value);
}

void writeRAMData(uint8_t *smuxData, uint8_t offset)
{
	if (offset == INTA_OFFSET)
	{
		AS7421_writeRegister(CFG_RAM_0, smuxData[0]);
		AS7421_writeRegister(CFG_RAM_1, smuxData[1]);
		AS7421_writeRegister(CFG_RAM_2, smuxData[2]);
		AS7421_writeRegister(CFG_RAM_3, smuxData[3]);
		AS7421_writeRegister(CFG_RAM_4, smuxData[4]);
		AS7421_writeRegister(CFG_RAM_5, smuxData[5]);
		AS7421_writeRegister(CFG_RAM_6, smuxData[6]);
		AS7421_writeRegister(CFG_RAM_7, smuxData[7]);
	}

	if (offset == INTB_OFFSET)
	{
		AS7421_writeRegister(CFG_RAM_8, smuxData[0]);
		AS7421_writeRegister(CFG_RAM_9, smuxData[1]);
		AS7421_writeRegister(CFG_RAM_10, smuxData[2]);
		AS7421_writeRegister(CFG_RAM_11, smuxData[3]);
		AS7421_writeRegister(CFG_RAM_12, smuxData[4]);
		AS7421_writeRegister(CFG_RAM_13, smuxData[5]);
		AS7421_writeRegister(CFG_RAM_14, smuxData[6]);
		AS7421_writeRegister(CFG_RAM_15, smuxData[7]);
	}

	if (offset == INTC_OFFSET)
	{
		AS7421_writeRegister(CFG_RAM_16, smuxData[0]);
		AS7421_writeRegister(CFG_RAM_17, smuxData[1]);
		AS7421_writeRegister(CFG_RAM_18, smuxData[2]);
		AS7421_writeRegister(CFG_RAM_19, smuxData[3]);
		AS7421_writeRegister(CFG_RAM_20, smuxData[4]);
		AS7421_writeRegister(CFG_RAM_21, smuxData[5]);
		AS7421_writeRegister(CFG_RAM_22, smuxData[6]);
		AS7421_writeRegister(CFG_RAM_23, smuxData[7]);
	}

	if (offset == INTD_OFFSET)
	{
		AS7421_writeRegister(CFG_RAM_24, smuxData[0]);
		AS7421_writeRegister(CFG_RAM_25, smuxData[1]);
		AS7421_writeRegister(CFG_RAM_26, smuxData[2]);
		AS7421_writeRegister(CFG_RAM_27, smuxData[3]);
		AS7421_writeRegister(CFG_RAM_28, smuxData[4]);
		AS7421_writeRegister(CFG_RAM_29, smuxData[5]);
		AS7421_writeRegister(CFG_RAM_30, smuxData[6]);
		AS7421_writeRegister(CFG_RAM_31, smuxData[7]);
	}
}

// Clear RAM registers with SMUX
void zeroSMUX()
{
	uint8_t zeros[8] = {0};
	for (int i = SMUX_A_ADDR; i <= SMUX_D_ADDR; i++)
	{
		AS7421_writeRegister(CFG_RAM, i);

		writeRAMData(zeros, 0);
		writeRAMData(zeros, 1);
		writeRAMData(zeros, 2);
		writeRAMData(zeros, 3);
	}
}

// Set SMUX region (A,B,C, or D) with ramOffsetAddr
void setSMUX(uint8_t ramOffsetAddr, uint8_t offset, uint8_t* configvalues)
{
	AS7421_writeRegister(CFG_RAM, ramOffsetAddr); //Writing the ram offset(SMUX addresses) for programming the configuration into RAM
	writeRAMData(configvalues, offset); //Writing to respective ram registers after setting RAM offset
}

// Set SMUX for integration cycle A and write to ram registers
void setSMUX_A(uint8_t* configvalues)
{
	setSMUX(SMUX_A_ADDR, INTA_OFFSET, configvalues);
}

// Set SMUX for integration cycle B and write to ram registers
void setSMUX_B(uint8_t* configvalues)
{
	setSMUX(SMUX_B_ADDR, INTB_OFFSET, configvalues);
}

// Set SMUX for integration cycle C and write to ram registers
void setSMUX_C(uint8_t* configvalues)
{
	setSMUX(SMUX_C_ADDR, INTC_OFFSET, configvalues);
}

// Set SMUX for integration cycle D and write to ram registers
void setSMUX_D(uint8_t* configvalues)
{
	setSMUX(SMUX_D_ADDR, INTD_OFFSET, configvalues);
}

// Configure all SMUX registers either with a specified default array of bytes or the config_values
void configureSMUX()
{
    zeroSMUX();

    // Configure SMUX registers
	setSMUX_A(config_values);
	setSMUX_B(config_values);
	setSMUX_C(config_values);
	setSMUX_D(config_values);
}

//2^x gain, i.e. gain of 6 = 2^6 = 256x
void configureGain(uint8_t gain)
{
	if (gain > 8)
	{
		gain = 8;
	}

	uint8_t data[8] = {gain};

	AS7421_writeRegister(CFG_RAM, ASETUP_AB);

	writeRAMData(data, 0);
	writeRAMData(data, 1);
	writeRAMData(data, 2);
	writeRAMData(data, 3);

	AS7421_writeRegister(CFG_RAM, ASETUP_CD);

	writeRAMData(data, 0);
	writeRAMData(data, 1);
	writeRAMData(data, 2);
	writeRAMData(data, 3);
}

// Configure LED register
void configureLEDs(bool enable, uint8_t led, uint8_t current)
{
	//Clearing LED config register to default
	AS7421_writeRegister(CFG_LED, 0);

	uint8_t value = AS7421_readRegister(CFG_LED);

	value |= (enable << 7); //Configure SET_LED_ON (bit 7)

	for (int i = LED_MULT_0; i <= LED_MULT_3; i++)
	{
		value |= (i << 4); //Enable LED_OFFSET (bits 4 and 5)
		AS7421_writeRegister(CFG_LED, value); // Setting Offset address for programming the values for LED_MULT
		AS7421_writeRegister(CFG_LED_MULT, led);
	}

	if (current > LED_CURRENT_LIMIT_75MA)
	{
		current = LED_CURRENT_LIMIT_75MA;
	}
	value |= (current << 0);

	AS7421_writeRegister(CFG_LED, value);
}

// Start spectral measurement
void startMeasurements(bool withLED)
{
	//Turn on LEDs
	configureLEDAuto(withLED);

	uint8_t value = AS7421_readRegister(ENABLE); //Read existing state

	// Power on
	value |= (1U << 0); //Set PON (bit 0)

    //Spectral measurement enabled
    value |= (1U << 1); //Set LTF_EN (bit 1)

    //Automatic power down by temperature measurement
    value |= (1U << 2); //Set TSD_EN (bit 2)

    AS7421_writeRegister(ENABLE, value);
}

void stopMeasurements()
{
	uint8_t value = AS7421_readRegister(ENABLE); //Read existing state

    //Spectral measurement enabled
    value &= ~(1U << 1); //Reset LTF_EN (bit 1)

    //Automatic power down by temperature measurement
    value &= ~(1U << 2); //Reset TSD_EN (bit 2)

    AS7421_writeRegister(ENABLE, value);

	configureLEDAuto(false);
}

//Measurement is active. New measurement cannot be started
bool measurementActive()
{
	uint8_t value = AS7421_readRegister(STATUS_6);
    bool status = (value & (1U << 4)) != 0; // Isolate bit 4 (LTF_BUSY) and check if it's set
    return status;
}

//Measurement is finished. New measurement can be started
bool measurementReady()
{
	uint8_t value = AS7421_readRegister(STATUS_6);
    bool status = (value & (1U << 5)) != 0; // Isolate bit 5 (LTF_READY) and check if it's set
    return status;
}

/* Status 7 bits
 * Bit[7:6] = I2C_DATA_PTR
 * Bit5 = DLOST
 * Bit4 = DSAT
 * Bit3 = ASAT
 * Bit2 = TSD
 * Bit1 = AZ
 * Bit0 = ADATA
 */
bool getMeasurementStatus(uint8_t bit)
{
	uint8_t status7 = AS7421_readRegister(STATUS_7);
	if (bit > 7)
	{
		printf("Bit is not within range of 0-7 of STATUS_7 register \n\r");
		return 0;
	}
    bool status = (status7 & (1U << bit)) != 0; // Isolate a bit and check its status
	return status;
}

// Helper function to record channel data
uint16_t recordChannelData(uint8_t addr)
{
	uint8_t data[2] = {0};
	AS7421_readRegisters(addr, 2, data);

	// Combine the two bytes into a 16-bit value
	return ((uint16_t)data[1] << 8) | data[0];
}

/* Integration Cycle A */
uint16_t getChannel1()
{
	return recordChannelData(CH1_DATA);
}
uint16_t getChannel48()
{
	return recordChannelData(CH48_DATA);
}
uint16_t getChannel2()
{
	return recordChannelData(CH2_DATA);
}
uint16_t getChannel34()
{
	return recordChannelData(CH34_DATA);
}
uint16_t getChannel16()
{
	return recordChannelData(CH16_DATA);

}
uint16_t getChannel32()
{
	return recordChannelData(CH32_DATA);
}
uint16_t getChannel18()
{
	return recordChannelData(CH18_DATA);
}
uint16_t getChannel51()
{
	return recordChannelData(CH51_DATA);
}

//Next 8 PDs
uint16_t getChannel4()
{
	return recordChannelData(CH4_DATA);
}
uint16_t getChannel49()
{
	return recordChannelData(CH49_DATA);
}
uint16_t getChannel3()
{
	return recordChannelData(CH3_DATA);
}
uint16_t getChannel35()
{
	return recordChannelData(CH35_DATA);

}
uint16_t getChannel17()
{
	return recordChannelData(CH17_DATA);
}
uint16_t getChannel33()
{
	return recordChannelData(CH33_DATA);
}
uint16_t getChannel19()
{
	return recordChannelData(CH19_DATA);
}
uint16_t getChannel54()
{
	return recordChannelData(CH54_DATA);
}

/* Integration Cycle B */
uint16_t getChannel0()
{
	return recordChannelData(CH0_DATA);
}
uint16_t getChannel13()
{
	return recordChannelData(CH13_DATA);
}
uint16_t getChannel50()
{
	return recordChannelData(CH50_DATA);
}
uint16_t getChannel63()
{
	return recordChannelData(CH63_DATA);
}
uint16_t getChannel52()
{
	return recordChannelData(CH52_DATA);
}
uint16_t getChannel6()
{
	return recordChannelData(CH6_DATA);
}
uint16_t getChannel38()
{
	return recordChannelData(CH38_DATA);
}
uint16_t getChannel20()
{
	return recordChannelData(CH20_DATA);
}

//Next 8 PDs
uint16_t getChannel36()
{
	return recordChannelData(CH36_DATA);
}
uint16_t getChannel22()
{
	return recordChannelData(CH22_DATA);
}
uint16_t getChannel55()
{
	return recordChannelData(CH55_DATA);
}
uint16_t getChannel5()
{
	return recordChannelData(CH5_DATA);
}
uint16_t getChannel53()
{
	return recordChannelData(CH53_DATA);
}
uint16_t getChannel7()
{
	return recordChannelData(CH7_DATA);
}
uint16_t getChannel39()
{
	return recordChannelData(CH39_DATA);
}
uint16_t getChannel21()
{
	return recordChannelData(CH21_DATA);
}

/* Integration Cycle C */
uint16_t getChannel37()
{
	return recordChannelData(CH37_DATA);
}
uint16_t getChannel23()
{
	return recordChannelData(CH23_DATA);
}
uint16_t getChannel40()
{
	return recordChannelData(CH40_DATA);
}
uint16_t getChannel26()
{
	return recordChannelData(CH26_DATA);
}
uint16_t getChannel42()
{
	return recordChannelData(CH42_DATA);
}
uint16_t getChannel24()
{
	return recordChannelData(CH24_DATA);
}
uint16_t getChannel56()
{
	return recordChannelData(CH56_DATA);
}
uint16_t getChannel10()
{
	return recordChannelData(CH10_DATA);
}

//Next 8 PDs
uint16_t getChannel58()
{
	return recordChannelData(CH58_DATA);
}
uint16_t getChannel8()
{
	return recordChannelData(CH8_DATA);
}
uint16_t getChannel41()
{
	return recordChannelData(CH41_DATA);
}
uint16_t getChannel27()
{
	return recordChannelData(CH27_DATA);
}
uint16_t getChannel43()
{
	return recordChannelData(CH43_DATA);
}
uint16_t getChannel25()
{
	return recordChannelData(CH25_DATA);
}
uint16_t getChannel57()
{
	return recordChannelData(CH57_DATA);
}
uint16_t getChannel11()
{
	return recordChannelData(CH11_DATA);
}

/* Integration Cycle D */
uint16_t getChannel59()
{
	return recordChannelData(CH59_DATA);
}
uint16_t getChannel9()
{
	return recordChannelData(CH9_DATA);
}
uint16_t getChannel44()
{
	return recordChannelData(CH44_DATA);
}
uint16_t getChannel30()
{
	return recordChannelData(CH30_DATA);
}
uint16_t getChannel46()
{
	return recordChannelData(CH46_DATA);
}
uint16_t getChannel28()
{
	return recordChannelData(CH28_DATA);
}
uint16_t getChannel60()
{
	return recordChannelData(CH60_DATA);
}
uint16_t getChannel14()
{
	return recordChannelData(CH14_DATA);
}

//Next 8 PDs
uint16_t getChannel62()
{
	return recordChannelData(CH62_DATA);
}
uint16_t getChannel12()
{
	return recordChannelData(CH12_DATA);
}
uint16_t getChannel45()
{
	return recordChannelData(CH45_DATA);
}
uint16_t getChannel31()
{
	return recordChannelData(CH31_DATA);
}
uint16_t getChannel47()
{
	return recordChannelData(CH47_DATA);
}
uint16_t getChannel29()
{
	return recordChannelData(CH29_DATA);
}
uint16_t getChannel61()
{
	return recordChannelData(CH61_DATA);
}
uint16_t getChannel15()
{
	return recordChannelData(CH15_DATA);
}


void getAllSpectralData(uint16_t arr[CHANNELSIZE])
{
	/* Integration Cycle A */
	arr[0] = getChannel1();
	arr[1] = getChannel48();
	arr[2] = getChannel2();
	arr[3] = getChannel34();
	arr[4] = getChannel16();
	arr[5] = getChannel32();
	arr[6] = getChannel18();
	arr[7] = getChannel51();

	arr[8] = getChannel4();
	arr[9] = getChannel49();
	arr[10] = getChannel3();
	arr[11] = getChannel35();
	arr[12] = getChannel17();
	arr[13] = getChannel33();
	arr[14] = getChannel19();
	arr[15] = getChannel54();

	/* Integration Cycle B */
	arr[16] = getChannel0();
	arr[17] = getChannel13();
	arr[18] = getChannel50();
	arr[19] = getChannel63();
	arr[20] = getChannel52();
	arr[21] = getChannel6();
	arr[22] = getChannel38();
	arr[23] = getChannel20();

	arr[24] = getChannel36();
	arr[25] = getChannel22();
	arr[26] = getChannel55();
	arr[27] = getChannel5();
	arr[28] = getChannel53();
	arr[29] = getChannel7();
	arr[30] = getChannel39();
	arr[31] = getChannel21();

	/* Integration Cycle C */
	arr[32] = getChannel37();
	arr[33] = getChannel23();
	arr[34] = getChannel40();
	arr[35] = getChannel26();
	arr[36] = getChannel42();
	arr[37] = getChannel24();
	arr[38] = getChannel56();
	arr[39] = getChannel10();

	arr[40] = getChannel58();
	arr[41] = getChannel8();
	arr[42] = getChannel41();
	arr[43] = getChannel27();
	arr[44] = getChannel43();
	arr[45] = getChannel25();
	arr[46] = getChannel57();
	arr[47] = getChannel11();

	/* Integration Cycle D */
	arr[48] = getChannel59();
	arr[49] = getChannel9();
	arr[50] = getChannel44();
	arr[51] = getChannel30();
	arr[52] = getChannel46();
	arr[53] = getChannel28();
	arr[54] = getChannel60();
	arr[55] = getChannel14();

	arr[56] = getChannel62();
	arr[57] = getChannel12();
	arr[58] = getChannel45();
	arr[59] = getChannel31();
	arr[60] = getChannel47();
	arr[61] = getChannel29();
	arr[62] = getChannel61();
	arr[63] = getChannel15();
}

//void getAllSpectralData(uint16_t arr[CHANNELSIZE])
//{
//	//Ordered PDs
//	/* Integration Cycle A */
//	arr[0] = getChannel1() / 431;
//	arr[1] = getChannel48() / 375;
//	arr[2] = getChannel2() / 409;
//	arr[3] = getChannel34() / 401;
//	arr[4] = getChannel16() / 398;
//	arr[5] = getChannel32() / 401;
//	arr[6] = getChannel18() / 390;
//	arr[7] = getChannel51() / 320;
//
//	arr[8] = getChannel4() / 190;
//	arr[9] = getChannel49() / 401;
//	arr[10] = getChannel3() / 296;
//	arr[11] = getChannel35() / 370;
//	arr[12] = getChannel17() / 416;
//	arr[13] = getChannel33() / 430;
//	arr[14] = getChannel19() / 367;
//	arr[15] = getChannel54() / 169;
//
//	/* Integration Cycle B */
//	arr[16] = getChannel0() / 406;
//	arr[17] = getChannel13() / 151;
//	arr[18] = getChannel50() / 387;
//	arr[19] = getChannel63() / 426;
//	arr[20] = getChannel52() / 239;
//	arr[21] = getChannel6() / 105;
//	arr[22] = getChannel38() / 225;
//	arr[23] = getChannel20() / 253;
//
//	arr[24] = getChannel36() / 304;
//	arr[25] = getChannel22() / 180;
//	arr[26] = getChannel55() / 74;
//	arr[27] = getChannel5() / 112;
//	arr[28] = getChannel53() / 163;
//	arr[29] = getChannel7() / 385;
//	arr[30] = getChannel39() / 104;
//	arr[31] = getChannel21() / 166;
//
//	/* Integration Cycle C */
//	arr[32] = getChannel37() / 209;
//	arr[33] = getChannel23() / 74;
//	arr[34] = getChannel40() / 397;
//	arr[35] = getChannel26() / 405;
//	arr[36] = getChannel42() / 371;
//	arr[37] = getChannel24() / 398;
//	arr[38] = getChannel56() / 403;
//	arr[39] = getChannel10() / 385;
//
//	arr[40] = getChannel58() / 327;
//	arr[41] = getChannel8() / 394;
//	arr[42] = getChannel41() / 418;
//	arr[43] = getChannel27() / 363;
//	arr[44] = getChannel43() / 354;
//	arr[45] = getChannel25() / 411;
//	arr[46] = getChannel57() / 413;
//	arr[47] = getChannel11() / 344;
//
//	/* Integration Cycle D */
//	arr[48] = getChannel59() / 214;
//	arr[49] = getChannel9() / 422;
//	arr[50] = getChannel44() / 269;
//	arr[51] = getChannel30() / 206;
//	arr[52] = getChannel46() / 203;
//	arr[53] = getChannel28() / 286;
//	arr[54] = getChannel60() / 214;
//	arr[55] = getChannel14() / 148;
//
//	arr[56] = getChannel62() / 134;
//	arr[57] = getChannel12() / 235;
//	arr[58] = getChannel45() / 177;
//	arr[59] = getChannel31() / 92;
//	arr[60] = getChannel47() / 90;
//	arr[61] = getChannel29() / 189;
//	arr[62] = getChannel61() / 143;
//	arr[63] = getChannel15() / 61;
//}

//void getAllSpectralData(float arr[CHANNELSIZE])
//{
//	//Ordered PDs
//	/* Integration Cycle A */
//	arr[0] = getChannel1() / 406;
//	arr[1] = getChannel48() / 431;
//	arr[2] = getChannel2() / 409;
//	arr[3] = getChannel34() / 296;
//	arr[4] = getChannel16() / 190;
//	arr[5] = getChannel32() / 112;
//	arr[6] = getChannel18() / 105;
//	arr[7] = getChannel51() / 385;
//
//	arr[8] = getChannel4() / 394;
//	arr[9] = getChannel49() / 422;
//	arr[10] = getChannel3() / 385;
//	arr[11] = getChannel35() / 344;
//	arr[12] = getChannel17() / 235;
//	arr[13] = getChannel33() / 151;
//	arr[14] = getChannel19() / 148;
//	arr[15] = getChannel54() / 61;
//
//	/* Integration Cycle B */
//	arr[16] = getChannel0() / 398;
//	arr[17] = getChannel13() / 416;
//	arr[18] = getChannel50() / 390;
//	arr[19] = getChannel63() / 367;
//	arr[20] = getChannel52() / 253;
//	arr[21] = getChannel6() / 166;
//	arr[22] = getChannel38() / 180;
//	arr[23] = getChannel20() / 74;
//
//	arr[24] = getChannel36() / 398;
//	arr[25] = getChannel22() / 411;
//	arr[26] = getChannel55() / 405;
//	arr[27] = getChannel5() / 363;
//	arr[28] = getChannel53() / 286;
//	arr[29] = getChannel7() / 189;
//	arr[30] = getChannel39() / 206;
//	arr[31] = getChannel21() / 92;
//
//	/* Integration Cycle C */
//	arr[32] = getChannel37() / 401;
//	arr[33] = getChannel23() / 430;
//	arr[34] = getChannel40() / 401;
//	arr[35] = getChannel26() / 370;
//	arr[36] = getChannel42() / 304;
//	arr[37] = getChannel24() / 209;
//	arr[38] = getChannel56() / 225;
//	arr[39] = getChannel10() / 104;
//
//	arr[40] = getChannel58() / 397;
//	arr[41] = getChannel8() / 418;
//	arr[42] = getChannel41() / 371;
//	arr[43] = getChannel27() / 354;
//	arr[44] = getChannel43() / 269;
//	arr[45] = getChannel25() / 177;
//	arr[46] = getChannel57() / 203;
//	arr[47] = getChannel11() / 90;
//
//	/* Integration Cycle D */
//	arr[48] = getChannel59() / 375;
//	arr[49] = getChannel9() / 401;
//	arr[50] = getChannel44() / 387;
//	arr[51] = getChannel30() / 320;
//	arr[52] = getChannel46() / 239;
//	arr[53] = getChannel28() / 163;
//	arr[54] = getChannel60() / 169;
//	arr[55] = getChannel14() / 74;
//
//	arr[56] = getChannel62() / 403;
//	arr[57] = getChannel12() / 413;
//	arr[58] = getChannel45() / 327;
//	arr[59] = getChannel31() / 214;
//	arr[60] = getChannel47() / 214;
//	arr[61] = getChannel29() / 143;
//	arr[62] = getChannel61() / 134;
//	arr[63] = getChannel15() / 426;
//}


// Helper function to record temperature data
uint16_t recordTemperatures(uint8_t addr)
{
	uint8_t data[2] = {0};
	AS7421_readRegisters(addr, 2, data);

	// Combine the two bytes into a 16-bit value
	return ((uint16_t)data[1] << 8) | data[0];
}

uint16_t getTemp_IntA()
{
	return recordTemperatures(TEMPA);
}
uint16_t getTemp_IntB()
{
	return recordTemperatures(TEMPB);
}
uint16_t getTemp_IntC()
{
	return recordTemperatures(TEMPC);
}
uint16_t getTemp_IntD()
{
	return recordTemperatures(TEMPD);
}

void getAllTemperatureData(uint16_t arr[TEMPSIZE])
{
	arr[0] = getTemp_IntA();
	arr[1] = getTemp_IntB();
	arr[2] = getTemp_IntC();
	arr[3] = getTemp_IntD();
}
