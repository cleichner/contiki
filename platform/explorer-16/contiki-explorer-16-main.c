#if defined(__XC16__)
#include <xc.h>
#elif defined(__C30__)
#if defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#endif
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mrf24j40.h"
#include "pins.h"

#include "contiki.h"
#include "dev/leds.h"
#include "dev/serial-line.h"

// FGS
#pragma config GWRP = OFF       // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF        // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF       // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = PRIPLL   // Initial Oscillator Source Selection Bits (Primary Oscillator (XT, HS, EC) with PLL)
#pragma config IESO = OFF       // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT      // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF   // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON     // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD   // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler Bits (1:32,768)
#pragma config WDTPRE = PR128   // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON      // PLL Lock Wait Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF     // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF     // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128   // Power-on Reset Timer Value Select bits (128ms)
#pragma config BOREN = ON       // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF    // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)
#pragma config ALTI2C2 = OFF    // Alternate I2C pins for I2C2 (SDA2/SCK2 pins are selected as the I/O pins for I2C2)

// FICD
#pragma config ICS = PGD1       // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config RSTPRI = PF      // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF     // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF       // Auxiliary Segment Write-protect bit (Auxiliary program memory is not write-protected)
#pragma config APL = OFF        // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF       // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)

int
write(int handle, void *buffer, unsigned int len)
{
  switch (handle) {
  case 0:                      // handle 0 corresponds to stdout
  case 1:                      // handle 1 corresponds to stdin
  case 2:                      // handle 2 corresponds to stderr
  default:
    for(unsigned int i = 0; i < len; i++) {
      while(U2STAbits.UTXBF);   // Wait for space in UART2 Tx buffer
      U2TXREG = *(char *)buffer++;      // Write character to UART2
    }
  }
  return (len);
}

/* Microcontroller MIPs (FCY) */
// 140 MHz clock -> 70 MIPS
#define SYS_FREQ        140000000ULL
#define FCY             SYS_FREQ/2

// Waits for a signal to go high, with a given timeout. If it times out before
// receiving the signal, it returns false. If all goes well, it returns true.

bool
wait_for(volatile bool signal, uint32_t wait_time)
{
  uint32_t timer = 0;

  while(!signal) {
    timer++;
    if(timer > wait_time) {
      return false;
    }
  }
  return true;
}

// Synchronous Delay

void
delay32(uint32_t delay)
{
  for(uint32_t i = 0; i < delay; i++) {
    Nop();
  }
}

// Synchronous delay for the given number of microseconds.
// Not particularly precise.
void
clock_delay_usec(uint16_t dt)
{
  for(uint32_t i = 0; i < dt; i++) {
    delay32(FCY / 20000000ULL);
  }
}

void
uart2_init(void)
{
  _TRISF5 = OUTPUT;
  _RP101R = 0b000011;          // U2 Output

  _TRISF4 = INPUT;
  _U2RXR = 100;                 // RP 100

  U2MODEbits.BRGH = 1;          // select high speed baud rate generator
  U2BRG = 76;                   // 230400baud
  U2MODEbits.UARTEN = 1;        // enable the UART
  U2STAbits.UTXEN = 1;          // enable TX

  _U2RXIF = 0;                  // clear recieve interrupt flag
  _U2RXIE = 1;                  // enable reception interrupts
}

void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{
  _U2RXIF = 0;
  serial_line_input_byte(U2RXREG);
}

void
radio_int_init(void)
{
  _INT1R = MRF24J40_RP_INT;

  _INT1EP = 1;                  // 1 is negative edge
  _INT1IF = 0;
  _INT1IE = 1;
}

#define SPI_NO_ERRORS               0

int8_t
dspic_spi1_write(const uint8_t * data, uint32_t len)
{
  uint32_t dummy;

  for(uint32_t i = 0; i < len; i++) {
    while(SPI1STATbits.SPITBF);
    SPI1BUF = data[i];
    while(!SPI1STATbits.SPIRBF);
    dummy = SPI1BUF;
  }
  return SPI_NO_ERRORS;
}

int8_t
dspic_spi1_read(uint8_t * data, uint32_t len)
{
  for(uint32_t i = 0; i < len; i++) {
    while(SPI1STATbits.SPITBF);
    SPI1BUF = 0;
    while(!SPI1STATbits.SPIRBF);
    data[i] = SPI1BUF;
  }
  return SPI_NO_ERRORS;
}

void
spi1_init(void)
{
  MOSI_RP_R = SPI1_MOSI_CODE;
  SCLK_RP_R = SPI1_SCLK_CODE;
  _SDI1R = MISO_RP;

  // SPI1CON1 Register Settings
  SPI1CON1bits.MODE16 = 0;      // Communication is byte-wide (8 bits)
  SPI1CON1bits.CKE = 1;         // Serial output data changes on transition from
  // Idle clock state to active clock state
  SPI1CON1bits.CKP = 0;         // Active state for clock is a high-level;
  // Idle state is a low-level
  SPI1CON1bits.MSTEN = 1;       // Master mode enabled

  // 10MHz SPI Bus rate
  SPI1CON1bits.SPRE = 0b001;   // secondary prescalar is 7:1
  SPI1CON1bits.PPRE = 0b10;    // primary prescalar is 1:1
  SPI1STATbits.SPIEN = 1;       // Enable SPI module
}

void
osc_init(void)
{
  // Fsys = 2 * Fsys = 280 MHz
  // Fosc = 2 * Fcy = 140 MHz
  // Fcy = 70 MHz

  // Configure PLL prescaler, PLL postscaler, PLL divisor
  PLLFBD = 68;                  // M=70
  CLKDIVbits.PLLPOST = 0;       // N2=2
  CLKDIVbits.PLLPRE = 0;        // N1=2

  // Wait for PLL to lock
  while(OSCCONbits.LOCK != 1);
}

void
ports_init(void)
{
  ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = DIGITAL;
  TRISA = TRISC = TRISD = TRISE = TRISF = TRISG = OUTPUT;
  LATA = LATC = LATD = LATE = LATF = LATG = 0;
}

void
net_init(void)
{
  mrf24j40_set_panid(0xFFFF);

#ifdef SENDING
  mrf24j40_set_short_mac_addr(0xABCD);
#else
  mrf24j40_set_short_mac_addr(0xDCBA);
#endif

}

uint8_t header_len = 9;         // bytes
uint8_t data_len = 28;          // bytes
uint8_t seq_number = 0;
uint8_t offset = 13;

PROCESS(led_control_process, "LED Control");
PROCESS(command_reading_process, "Command Reading");

PROCESS_THREAD(command_reading_process, ev, data)
{
  uint8_t msg[MAX_PACKET_LEN] = { header_len,
    header_len + data_len,
    0x22, 0x22,                 // headermem
    0x00,                       // seq number
    0xff, 0xff,                 // broadcast PAN addr
    0xab, 0xcd,                 // dest addr
    0xdc, 0xba,                 // source addr
  };

  PROCESS_BEGIN();
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data);

    char command[] = "xxxxxxxxxxxxx\n";

    strncpy(command, data, 8);
    printf("%s", command);

    msg[4] = seq_number++;
    memcpy(msg + header_len + 2 + offset, command, data_len - offset);
    //write(0, msg, header_len + data_len + 2);
    if(mrf24j40_write(msg, MAX_PACKET_LEN) != RADIO_TX_OK) {
      printf("ERROR!\n");
    }
  }
  PROCESS_END();
}

PROCESS_THREAD(led_control_process, ev, data)
{
  int16_t off = 24;

  PROCESS_BEGIN();
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    printf("Received!\n");
    for(int16_t i = 0; i < 8; i++) {
      if(com[i + off] == '0') {
        LATA &= ~(1 << (7 - i));
      } else if(com[i + off] == '1') {
        LATA |= (1 << (7 - i));
      } else if(com[i + off] != 'x') {
        printf("Invalid command character: %c\n", com[i]);
      }
    }
  }
  PROCESS_END();
}

int
main()
{
  osc_init();
  ports_init();
  uart2_init();
  spi1_init();
  radio_int_init();
  mrf24j40_init();
  net_init();

  clock_init();
  rtimer_init();
  process_init();
  process_start(&etimer_process, NULL);

  serial_line_init();

  ctimer_init();
  autostart_start(autostart_processes);
  //leds_on(LEDS_RED);

  process_start(&command_reading_process, NULL);
  process_start(&led_control_process, NULL);
  while(1) {
    if(data_valid) {
      _INT1IE = 0;
      write(0, (void *)com + header_len + 2 + offset, data_len - offset - 1);
      data_valid = false;
      _INT1IE = 1;
    }
    process_run();
  }
  return 0;
}
