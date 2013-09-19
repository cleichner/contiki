/* 
 * File:   pins.h
 * Author: cleichner
 *
 * Created on September 16, 2013, 10:51 AM
 */

#ifndef PINS_H
#define	PINS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>

// These allow atomic bitwise manipulation of SFRs and are therefore suitable
// to use in the presence of interrupts. The awkward double defines are to make
// sure that the arguments are macro expanded before being stringified.

// Set the bit on pin 0-15 on sfr LAT, latx. Runs in one assembly instruction
// on word-alligned SFRs.
#define sfr_bset(latx, pin) _sfr_bset(latx, pin)
#define _sfr_bset(latx, pin) __asm__ volatile("bset " #latx ", #" #pin)

// Clear the bit on pin 0-15 on sfr LAT, latx. Runs in one assembly instruction
// on word-alligned SFRs.
#define sfr_bclr(latx, pin) _sfr_bclr(latx, pin)
#define _sfr_bclr(latx, pin) __asm__ volatile("bclr " #latx ", #" #pin)

// Toggle the bit on pin 0-15 on sfr LAT, latx. Runs in one assembly instruction
// on word-alligned SFRs.
#define sfr_btg(latx, pin) _sfr_btg(latx, pin)
#define _sfr_btg(latx, pin) __asm__ volatile("btg " #latx ", #" #pin)

#define OUTPUT 0
#define INPUT 1

#define DIGITAL 0
#define ANALOG 1

#define MRF24J40_TRIS_CSn _TRISD7
#define MRF24J40_CSn_LAT LATD
#define MRF24J40_CSn_PIN 7

#define MRF24J40_TRIS_RESETn _TRISF13
#define MRF24J40_RESETn_LAT LATF
#define MRF24J40_RESETn_PIN 13

#define MRF24J40_TRIS_WAKE _TRISA5
#define MRF24J40_WAKE_LAT LATA
#define MRF24J40_WAKE_PIN 5

#define MRF24J40_TRIS_INT _TRISB10
#define MRF24J40_RP_INT 42      // RB10

#define SCLK_RP_R _RP118R       // R66
#define MOSI_RP_R _RP67R        // RD3
#define MISO_RP   52            // RC4

#define SPI1_MOSI_CODE 0b000101
#define SPI1_SCLK_CODE 0b000110

  int8_t dspic_spi1_write(const uint8_t * data, uint32_t len);
  int8_t dspic_spi1_read(uint8_t * data, uint32_t len);

#ifdef	__cplusplus
}
#endif
#endif                          /* PINS_H */
