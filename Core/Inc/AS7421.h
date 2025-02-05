#ifndef AS7421_H_
#define AS7421_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "uart.h"
#include "i2c.h"
#include "systick.h"
#include "stm32f4xx.h"


/*I2C Slave Device Address*/
#define AS7421_ADDR							0x64
#define AS7421_READ_ADDR					0xC9
#define AS7421_WRITE_ADDR					0xC8

/*Configuration Registers */
#define CFG_MISC							0x38
#define CFG_LED_MULT						0x39
#define LTF_CCOUNT							0x3A
#define TEMP_COMPDAC						0x3C
#define LED_WAIT							0x3D
#define CFG_PINMAP							0x3E

/* Ram Registers */
#define CFG_RAM_0							0x40
#define CFG_RAM_1							0x41
#define CFG_RAM_2							0x42
#define CFG_RAM_3							0x43
#define CFG_RAM_4							0x44
#define CFG_RAM_5							0x45
#define CFG_RAM_6							0x46
#define CFG_RAM_7							0x47

#define CFG_RAM_8							0x48
#define CFG_RAM_9							0x49
#define CFG_RAM_10							0x4A
#define CFG_RAM_11							0x4B
#define CFG_RAM_12							0x4C
#define CFG_RAM_13							0x4D
#define CFG_RAM_14							0x4E
#define CFG_RAM_15							0x4F

#define CFG_RAM_16							0x50
#define CFG_RAM_17							0x51
#define CFG_RAM_18							0x52
#define CFG_RAM_19							0x53
#define CFG_RAM_20							0x54
#define CFG_RAM_21							0x55
#define CFG_RAM_22							0x56
#define CFG_RAM_23							0x57

#define CFG_RAM_24							0x58
#define CFG_RAM_25							0x59
#define CFG_RAM_26							0x5A
#define CFG_RAM_27							0x5B
#define CFG_RAM_28							0x5C
#define CFG_RAM_29							0x5D
#define CFG_RAM_30							0x5E
#define CFG_RAM_31							0x5F

/* Enable Register */
#define ENABLE								0x60

/* Configuration Registers */
#define LTF_ITIME							0x61 //Starting addresses
#define LTF_WTIME							0x64 //Starting addresses
#define CFG_LTF								0x67
#define CFG_LED								0x68
#define LTF_ICOUNT							0x69
#define CFG_RAM								0x6A
#define CFG_GPIO							0x6B
#define INT_ENABLE							0x6C
#define CFG_AZ								0x6D

/* Status Registers */
#define STATUS_0							0x70
#define STATUS_1							0x71
#define STATUS_2							0x72
#define STATUS_3							0x73
#define STATUS_6							0x76
#define STATUS_7							0x77

/* Temperature Registers (2-byte low starting addresses) */
#define TEMPA								0x78
#define TEMPB								0x7A
#define TEMPC								0x7C
#define TEMPD								0x7E

/* Spectral Channel Output Registers (2-byte low starting addresses) */
#define CH0_DATA							0x80
#define CH1_DATA							0x82
#define CH2_DATA							0x84
#define CH3_DATA							0x86
#define CH4_DATA							0x88
#define CH5_DATA							0x8A
#define CH6_DATA							0x8C
#define CH7_DATA							0x8E

#define CH8_DATA							0x90
#define CH9_DATA							0x92
#define CH10_DATA							0x94
#define CH11_DATA							0x96
#define CH12_DATA							0x98
#define CH13_DATA							0x9A
#define CH14_DATA							0x9C
#define CH15_DATA							0x9E

#define CH16_DATA							0xA0
#define CH17_DATA							0xA2
#define CH18_DATA							0xA4
#define CH19_DATA							0xA6
#define CH20_DATA							0xA8
#define CH21_DATA							0xAA
#define CH22_DATA							0xAC
#define CH23_DATA							0xAE

#define CH24_DATA							0xB0
#define CH25_DATA							0xB2
#define CH26_DATA							0xB4
#define CH27_DATA							0xB6
#define CH28_DATA							0xB8
#define CH29_DATA							0xBA
#define CH30_DATA							0xBC
#define CH31_DATA							0xBE

#define CH32_DATA							0xC0
#define CH33_DATA							0xC2
#define CH34_DATA							0xC4
#define CH35_DATA							0xC6
#define CH36_DATA							0xC8
#define CH37_DATA							0xCA
#define CH38_DATA							0xCC
#define CH39_DATA							0xCE

#define CH40_DATA							0xD0
#define CH41_DATA							0xD2
#define CH42_DATA							0xD4
#define CH43_DATA							0xD6
#define CH44_DATA							0xD8
#define CH45_DATA							0xDA
#define CH46_DATA							0xDC
#define CH47_DATA							0xDE

#define CH48_DATA							0xE0
#define CH49_DATA							0xE2
#define CH50_DATA							0xE4
#define CH51_DATA							0xE6
#define CH52_DATA							0xE8
#define CH53_DATA							0xEA
#define CH54_DATA							0xEC
#define CH55_DATA							0xEE

#define CH56_DATA							0xF0
#define CH57_DATA							0xF2
#define CH58_DATA							0xF4
#define CH59_DATA							0xF6
#define CH60_DATA							0xF8
#define CH61_DATA							0xFA
#define CH62_DATA							0xFC
#define CH63_DATA							0xFE

//Settings
#define F_CLKMOD							1000000 //Frequency of integration clock
#define INTEGRATIONCYLE_A			 		0b00
#define INTEGRATIONCYLE_AB			 		0b01
#define INTEGRATIONCYLE_ABC			 		0b10
#define INTEGRATIONCYLE_ABCD		 		0b11

#define SMUX_A_ADDR							0x0C
#define SMUX_B_ADDR							0x0D
#define SMUX_C_ADDR							0x0E
#define SMUX_D_ADDR							0x0F

#define ASETUP_AB							0x10
#define ASETUP_CD							0x11

#define CONTINUOUS							255
#define AZ_WTIME_32US						0b00
#define AZ_WTIME_64US						0b01
#define AZ_WTIME_128US						0b10
#define AZ_WTIME_256US						0b11

#define INTA_OFFSET							0
#define INTB_OFFSET							8
#define INTC_OFFSET							16
#define INTD_OFFSET							24

#define LED_MULT_0							0b00
#define LED_MULT_1							0b01
#define LED_MULT_2							0b10
#define LED_MULT_3							0b11

#define LED_1								0x01
#define LED_2								0x02
#define LED_3								0x04
#define LED_4								0x08
#define ALL_LEDS							0x1F

#define LED_CURRENT_LIMIT_50MA				0b000
#define LED_CURRENT_LIMIT_75MA				0b001

#define ADATA								0b0
#define CHANNELSIZE							64
#define TEMPSIZE							4


/* Initializing functions*/
void fpu_enable(); //Enable floating point unit
bool initialize();
bool isConnected(); //Checks if sensor ACK the I2C request
void startup(); //Setting Integration time with SMUX and gain settings

/* Device Information and status*/
uint8_t getDeviceID();
uint8_t getRevID();
uint8_t getAnalogSatMod_0_7(); //Analog saturation happens when the signal level exceeds the sensor’s range, which can distort measurements or lead to inaccurate results for that specific modulator.
uint8_t getAnalogSatMod_8_15(); //Status 3

//Status 6
bool tempSaturated(); //Analog saturation of temperature
bool measurementReady(); //Measurement is finished. New measurement can be started
bool measurementActive(); //Measurement is active. New measurement cannot be started

// Status 7 register settings
/* Bit[7:6] = I2C_DATA_PTR
 * Bit5 = DLOST
 * Bit4 = DSAT
 * Bit3 = ASAT
 * Bit2 = TSD
 * Bit1 = AZ
 * Bit0 = ADATA
 */
bool getMeasurementStatus(uint8_t bit);


/* Temperature */
uint16_t recordTemperatures(uint8_t addr);
void getAllTemperatureData(uint16_t arr[TEMPSIZE]);

uint16_t getTemp_IntA();
uint16_t getTemp_IntB();
uint16_t getTemp_IntC();
uint16_t getTemp_IntD();

/* LED control */
void enableLED(uint8_t device);
void disableLED(uint8_t device);
void configureLEDs(bool enable, uint8_t led, uint8_t current);
void enableLEDs();
void disableLEDs();

void configueLEDWait(bool setting); //Enable and set LED_WAIT or Disable the LED wait time between integration cycles
void setInterLED(uint8_t waitTime); //Wait time (LED_WAIT) between switching on the LED and begin of integration/modulation
void configureLEDAuto(bool mode); //Controls NIR light source during spectral measurement
void configureAutozero(bool enable, uint8_t az_waitTime, uint8_t iteration, uint8_t cycle);

/* Measurement control, mode */
void setIntegrationTime(uint32_t intTime); //Register 0x61, 0x62 and 0x63 program the integration time of the LTF converter
void configureWaitCycle(bool setting); //Enable the waiting time between integration cycle A to D (programmable with LTF_WTIME)
void setWaitTime(uint32_t waitTime); //Time in between consecutive measurements using LTF_WTIME
void setLTF_CCOUNT(uint16_t ccount_value);
void setIntegrationMode(uint8_t mode); //16 channels (A), 32 channels (AB), 48 channels (ABC), 64 channels (ABCD)
bool measurementReady();

/* Gain, integration cycles */
void powerup(); // Internal oscillator enabled, potentially write 0x44 to register 0x6F, 0x20 to register 0x6E, 0x00 to register 0x6F
void sleep();
void printRAMData();
void writeRAMData(uint8_t *smuxData, uint8_t offset);

void configureGain(uint8_t gain); //2^x gain, i.e. gain of 5 = 2^5 = 128x
void setSMUX(uint8_t ramOffsetAddr, uint8_t offset, uint8_t* configvalues);
void setSMUX_A(uint8_t* configvalues);
void setSMUX_B(uint8_t* configvalues);
void setSMUX_C(uint8_t* configvalues);
void setSMUX_D(uint8_t* configvalues);
void configureSMUX();
void zeroSMUX();

void performMeasurements(uint16_t arrSpectra[CHANNELSIZE], uint16_t arrTemp[TEMPSIZE]);
void startMeasurements(bool withLED);
void stopMeasurements();
void numMeasurements(uint8_t counts); //specifying number of measurements
uint8_t endOfMeasurements();

/* Reset */
void reset();


/* Reading 64 channels
 * Note: order of the 64 channels is not intuitively organized
 * Order of declaration is required to obtain continuous organized spectra
 * Before reading the data of the last */
uint16_t recordChannelData(uint8_t addr);
void getAllSpectralData(uint16_t arr[CHANNELSIZE]);

/* Integration Cycle A */
uint16_t getChannel1();
uint16_t getChannel48();
uint16_t getChannel2();
uint16_t getChannel34();
uint16_t getChannel16();
uint16_t getChannel32();
uint16_t getChannel18();
uint16_t getChannel51();

uint16_t getChannel4();
uint16_t getChannel49();
uint16_t getChannel3();
uint16_t getChannel35();
uint16_t getChannel17();
uint16_t getChannel33();
uint16_t getChannel19();
uint16_t getChannel54();

/* Integration Cycle B */
uint16_t getChannel0();
uint16_t getChannel13();
uint16_t getChannel50();
uint16_t getChannel63();
uint16_t getChannel52();
uint16_t getChannel6();
uint16_t getChannel38();
uint16_t getChannel20();

uint16_t getChannel36();
uint16_t getChannel22();
uint16_t getChannel55();
uint16_t getChannel5();
uint16_t getChannel53();
uint16_t getChannel7();
uint16_t getChannel39();
uint16_t getChannel21();

/* Integration Cycle C */
uint16_t getChannel37();
uint16_t getChannel23();
uint16_t getChannel40();
uint16_t getChannel26();
uint16_t getChannel42();
uint16_t getChannel24();
uint16_t getChannel56();
uint16_t getChannel10();

uint16_t getChannel58();
uint16_t getChannel8();
uint16_t getChannel41();
uint16_t getChannel27();
uint16_t getChannel43();
uint16_t getChannel25();
uint16_t getChannel57();
uint16_t getChannel11();

/* Integration Cycle D */
uint16_t getChannel59();
uint16_t getChannel9();
uint16_t getChannel44();
uint16_t getChannel30();
uint16_t getChannel46();
uint16_t getChannel28();
uint16_t getChannel60();
uint16_t getChannel14();

uint16_t getChannel62();
uint16_t getChannel12();
uint16_t getChannel45();
uint16_t getChannel31();
uint16_t getChannel47();
uint16_t getChannel29();
uint16_t getChannel61();
uint16_t getChannel15();


#ifdef __cplusplus
}
#endif

#endif /* AS7421X_H_ */
