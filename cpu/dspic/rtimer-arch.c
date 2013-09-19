#include <xc.h>
#include <stdint.h>

#include "sys/energest.h"
#include "sys/rtimer.h"
#include "sys/etimer.h"
#include "rtimer-arch.h"
#include "contiki-conf.h"
#include "sys/clock.h"

volatile uint64_t rcounter = 0;
volatile uint64_t ecounter = 0;
static volatile clock_time_t count;     // milliseconds

/* code for Timer2 ISR - one millisecond interrupt */
void __attribute__ ((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
  _T2IF = 0;                    // Clear Timer1 Interrupt Flag

  count++;

  if(rcounter <= 1) {
    rtimer_run_next();
  } else if(rcounter > 0) {
    rcounter--;
  }

  if(++ecounter >= 10) {
    ecounter = 0;
    if(etimer_pending()) {
      etimer_request_poll();
    }
  }
}

void
clock_init(void)
{
  count = 0;
}

unsigned long
clock_seconds(void)
{
  return clock_time() / CLOCK_CONF_SECOND;
}

clock_time_t
clock_time(void)
{
  _T2IE = 0;
  clock_time_t time = count;

  _T2IE = 1;
  return time;
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
  _T2IE = 0;
  rcounter = t;
  _T2IE = 1;
}

/**
 * Blocking delay for a multiple of milliseconds
 */
void
clock_delay(unsigned int i)
{
  uint64_t waitfor = clock_time() + i;

  while(waitfor < clock_time()) {
    for(int16_t i = 0; i < 1000; i++) {
      // make sure the timer interrupt is enabled long enough to run
    }
  }
}

void
rtimer_arch_init(void)
{
  rcounter = 0;
  // Fosc = 70Mhz, prescale of 1:256 = Fin = 273438Hz = 3.7uSec
  //   reload register => 273x3.7uSec => interupt rate of 998.4 uSec ~ 1ms
  T2CONbits.TON = 0;            // Disable Timer
  T2CONbits.TCKPS = 0b11;      // Select 1:256 Prescaler
  /*
     #if RTIMER_ARCH_PRESCALER==256
     T2CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
     #elif RTIMER_ARCH_PRESCALER==64
     T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
     #elif RTIMER_ARCH_PRESCALER==8
     T2CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
     #elif RTIMER_ARCH_PRESCALER==1
     T2CONbits.TCKPS = 0b00; // Select 1:1 prescale value
     #else
     #error Timer2 RTIMER_ARCH_PRESCALER factor not supported.
     #endif
   */

  TMR2 = 0x00;                  // Clear timer register
  PR2 = 273;                    // Load the period value
  _T2IP = 7;                    // Set Timer2 Interrupt Priority Level
  _T2IF = 0;                    // Clear Timer2 Interrupt Flag
  _T2IE = 1;                    // Enable Timer2 interrupt
  T2CONbits.TON = 1;            // Start Timer
}
