#include "Arduino.h"
#include "Prescaler.h"

// Initialize global variable.
static uint8_t __clock_prescaler = (CLKPR & (_BV(CLKPS0) | _BV(CLKPS1) | _BV(CLKPS2) | _BV(CLKPS3)));

void setClockPrescaler(uint8_t clockPrescaler) {
  if (clockPrescaler <= CLOCK_PRESCALER_256) {
    // Disable interrupts.
    uint8_t oldSREG = SREG;
    cli();

    // Enable change.
    CLKPR = _BV(CLKPCE); // write the CLKPCE bit to one and all the other to zero

    // Change clock division.
    CLKPR = clockPrescaler; // write the CLKPS0..3 bits while writing the CLKPE bit to zero

    // Copy for fast access.
    __clock_prescaler = clockPrescaler;

    // Recopy interrupt register.
    SREG = oldSREG;    
  }
}

uint8_t getClockPrescaler() {
  return (__clock_prescaler);
}

uint16_t getClockDivisionFactor() {
  return ((uint16_t)(1 << __clock_prescaler));
}

unsigned long trueMillis()
{
  return millis() * getClockDivisionFactor();
}

void trueDelay(unsigned long ms)
{
  unsigned long start = trueMillis();
  while (trueMillis() - start < ms);
}

unsigned long rescaleDuration(unsigned long d) {
  return (d / getClockDivisionFactor());
}

unsigned long rescaleTime(unsigned long t) {
  return (t * getClockDivisionFactor());
}
