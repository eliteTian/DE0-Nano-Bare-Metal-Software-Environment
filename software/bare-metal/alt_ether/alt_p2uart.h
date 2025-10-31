/******************************************************************************
*
* Copyright 2013-2017 Altera Corporation. All Rights Reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
#ifndef ___ALT_P2UART_H___
#define ___ALT_P2UART_H___

#include <stdint.h>
#include "alt_16550_uart.h"
#include "alt_printf.h"
#include "socal/hps.h"
#include "socal/socal.h"
#include "socal/alt_uart.h"

#ifndef BAUD_RATE
#define BAUD_RATE      (115200)
#endif

typedef struct UART_INFO_s
{
  void (*putc_function)(char pchar, FILE * info);
  int    init_done;
  ALT_16550_DEVICE_t uart_ID;
  ALT_16550_HANDLE_t mUart;
} UART_INFO_t;



void uart_putc(char pchar, FILE *info);

ALT_STATUS_CODE init_uart(UART_INFO_t *uartInfo);


void alt_log_done(FILE *op);

#endif

