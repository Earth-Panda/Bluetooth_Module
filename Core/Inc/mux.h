#ifndef MUX_H_
#define MUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "systick.h"
#include "i2c.h"

#define MUX_ADDR							0x70
#define MUX_READ_ADDR						0xE1
#define MUX_WRITE_ADDR						0xE0

#define CHANNEL_0							(1U<<0)
#define CHANNEL_1							(1U<<1)
#define CHANNEL_2							(1U<<2)
#define CHANNEL_3							(1U<<3)
#define CHANNEL_4							(1U<<4)
#define CHANNEL_5							(1U<<5)
#define CHANNEL_6							(1U<<6)
#define CHANNEL_7							(1U<<7)
#define RESET								0x00


bool muxConnected();
void selectMux_and_control(uint8_t mux_address, uint8_t control_byte);
void enableChannel(uint8_t channel);
void disableChannels();


#ifdef __cplusplus
}
#endif

#endif /* MUX_H_ */
