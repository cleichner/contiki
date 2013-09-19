#ifndef __CONTIKI_CONF_H__EXPLORER16__
#define __CONTIKI_CONF_H__EXPLORER16__

#include <stdint.h>

#define CCIF
#define CLIF

#define PROCESS_CONF_NO_PROCESS_NAMES 1

#define SYS_FREQ        140000000ULL
#define F_CPU           SYS_FREQ/2

#define RTIMER_ARCH_PRESCALER 256
#define CLOCK_CONF_SECOND 1000

typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;

typedef uint64_t clock_time_t;
typedef unsigned int uip_stats_t;

// webserver log requests to serial port
#define LOG_CONF_ENABLED 0

/* uIP configuration */
#define UIP_CONF_LLH_LEN 0
#define UIP_CONF_BROADCAST 0
#define UIP_CONF_LOGGING 0

#define UIP_CONF_BUFFER_SIZE     0

// my cellphone advertises a large TCP MSS of 1460 which causes uIP to emit 
// frames of 1500 bytes (including Ethernet 2 header) which are dropped
// by either the ENC28J60 chip OR my switch. Not sure which, but using 
// this config to ensure the TCP MSS is negotiated down to something that
// gets off the board OK. 
// There are some "send large frame" config bits in the ethernet chip - to investigate
// if this increments a counter when triggered
#define UIP_CONF_TCP_MSS     0

// we don't bother with the packet queue, zero the size
#define QUEUEBUF_CONF_NUM        0
#define PACKETBUF_CONF_SIZE      0
#define PACKETBUF_CONF_HDR_SIZE  0

#define UIP_CONF_MAX_CONNECTIONS 0
#define UIP_CONF_MAX_LISTENPORTS 0
#define UIP_CONF_UDP_CONNS       0
// we don't forward any packets, so zero this
#define UIP_CONF_FWCACHE_SIZE    0
#define UIP_CONF_BROADCAST       0
#define UIP_CONF_UDP_CHECKSUMS   0
#define UIP_PINGNOADDRESSDROP    0
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_TCP_FORWARD     0
#define UIP_CONF_IPV6_QUEUE_PKT  0

#endif /* __CONTIKI_CONF_H__EXPLORER16__ */
