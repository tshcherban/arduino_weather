#ifndef PRESCALER_INC
#define PRESCALER_INC

#include "Arduino.h"

#define CLOCK_PRESCALER_1   (0x0)
#define CLOCK_PRESCALER_2   (0x1)
#define CLOCK_PRESCALER_4   (0x2)
#define CLOCK_PRESCALER_8   (0x3)
#define CLOCK_PRESCALER_16  (0x4)
#define CLOCK_PRESCALER_32  (0x5)
#define CLOCK_PRESCALER_64  (0x6)
#define CLOCK_PRESCALER_128 (0x7)
#define CLOCK_PRESCALER_256 (0x8)

void setClockPrescaler(uint8_t clockPrescaler);

uint8_t getClockPrescaler();

uint16_t getClockDivisionFactor();

inline unsigned long trueMillis();

inline unsigned long trueMicros();

void trueDelay(unsigned long ms);

void trueDelayMicros(unsigned long ms);

unsigned long rescaleDuration(unsigned long d);

unsigned long rescaleTime(unsigned long t);

#endif
