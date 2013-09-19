/*
 * Contiki SeedEye Platform project
 *
 * Copyright (c) 2012,
 *  Scuola Superiore Sant'Anna (http://www.sssup.it) and
 *  Consorzio Nazionale Interuniversitario per le Telecomunicazioni
 *  (http://www.cnit.it).
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \addtogroup mrf24j40 MRF24J40 Driver
 *
 * @{
 */

/**
 * \file   mrf24j40_arch.h
 * \brief  MRF24J40 Specific Arch Conf
 * \author Giovanni Pellerano <giovanni.pellerano@evilaliv3.org>
 * \date   2012-03-21
 */

#ifndef __MRF24J40_ARCH_H__
#define __MRF24J40_ARCH_H__

#if 0
#include "p32xxxx.h"
#endif

#include "pins.h"

#if 0
#include "dev/radio.h"
#endif

/* RESET low/high */
#define MRF24J40_HARDRESET_LOW()  sfr_bclr(MRF24J40_RESETn_LAT, MRF24J40_RESETn_PIN)
#define MRF24J40_HARDRESET_HIGH() sfr_bset(MRF24J40_RESETn_LAT, MRF24J40_RESETn_PIN)
#define MRF24J40_CSn_LOW()        sfr_bclr(MRF24J40_CSn_LAT, MRF24J40_CSn_PIN)
#define MRF24J40_CSn_HIGH()       sfr_bset(MRF24J40_CSn_LAT, MRF24J40_CSn_PIN)

/* Spi port Mapping */
#define MRF24J40_SPI_PORT_INIT  init_spi1
#define MRF24J40_SPI_PORT_WRITE dspic_spi1_write
#define MRF24J40_SPI_PORT_READ  dspic_spi1_read

/* IRC Configuration */
#define MRF24J40_ISR()  void __attribute__((interrupt, auto_psv)) _INT1Interrupt(void)
#define MRF24J40_INTERRUPT_FLAG_SET()       _INT1IF = 1
#define MRF24J40_INTERRUPT_FLAG_CLR()       _INT1IF = 0
#define MRF24J40_INTERRUPT_ENABLE_SET()     _INT1IE = 1
#define MRF24J40_INTERRUPT_ENABLE_CLR()     _INT1IE = 0
#define MRF24J40_INTERRUPT_ENABLE_STAT()    _INT1IE

#define MRF24J40_PINDIRECTION_INIT()        \
do {                                        \
    MRF24J40_TRIS_RESETn = 0;               \
    MRF24J40_TRIS_INT = 1;                  \
    MRF24J40_TRIS_CSn = 0;                  \
    MRF24J40_TRIS_WAKE = 0;                 \
} while(0)

#define MRF24J40_INTERRUPT_INIT(p, s)

#endif /* __MRF24J40_ARCH_H__ */

/** @} */
