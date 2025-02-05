#ifndef SYSTICK_H_
#define SYSTICK_H_

#include <stdint.h>

void SysTick_Init(void);
void SysTick_Handler(void);
uint32_t getMillis(void);
void delayMillis(uint32_t delay);

//void systickDelayMs(int delay);



#endif /* SYSTICK_H_ */
