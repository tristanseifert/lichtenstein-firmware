/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#if defined(TRACE)

#include "cmsis_device.h"
#include "diag/Trace.h"

/* USER CODE BEGIN 0 */
#define SWO_FREQ 115200
#define HCLK_FREQ SystemCoreClock

/* Pelican TPIU
 * cut down TPIU implemented in STM32F7 series
 * see en.DM00224583.pdf page 1882
 */
#define TPIU_CURRENT_PORT_SIZE *((volatile unsigned *)(0xE0040004))
#define TPIU_ASYNC_CLOCK_PRESCALER *((volatile unsigned *)(0xE0040010))
#define TPIU_SELECTED_PIN_PROTOCOL *((volatile unsigned *)(0xE00400F0))
#define TPIU_FORMATTER_AND_FLUSH_CONTROL *((volatile unsigned *)(0xE0040304))

// ----------------------------------------------------------------------------

// Forward definitions.
void SWO_PrintChar(char c, uint8_t portNo);

// ----------------------------------------------------------------------------

void trace_initialize(void) {
	  /* USER CODE BEGIN 1 */
	  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;
	  DBGMCU->CR = DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY | DBGMCU_CR_TRACE_IOEN;

	  TPIU_CURRENT_PORT_SIZE = 1; /* port size = 1 bit */
	  TPIU_SELECTED_PIN_PROTOCOL = 1; /* trace port protocol = Manchester */
	  TPIU_ASYNC_CLOCK_PRESCALER = (HCLK_FREQ / SWO_FREQ) - 1;
	  TPIU_FORMATTER_AND_FLUSH_CONTROL = 0x100; /* turn off formatter (0x02 bit) */

	  ITM->LAR = 0xC5ACCE55;
	  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk;
	  ITM->TPR = ITM_TPR_PRIVMASK_Msk; /* all ports accessible unprivileged */
	  ITM->TER = 1; /* enable stimulus channel 0, used with ITM_SendChar() */

	  /* this apparently turns off sync packets, see SYNCTAP in DDI0403D pdf: */
	  DWT->CTRL = 0x400003FE;
	  /* USER CODE END 1 */
}

/*!
 * \brief Sends a character over the SWO channel
 * \param c Character to be sent
 * \param portNo SWO channel number, value in the range of 0 to 31
 */
void SWO_PrintChar(char c, uint8_t portNo) {
	volatile int timeout;

	/* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
	if ((ITM->TCR&ITM_TCR_ITMENA_Msk) == 0) { /* check Trace Control Register if ITM trace is enabled*/
		return; /* not enabled? */
	}
	/* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
	if ((ITM->TER & (1ul<<portNo))==0) { /* check Trace Enable Register if requested port is enabled */
		return; /* requested port not enabled? */
	}

	timeout = 5000; /* arbitrary timeout value */
	while (ITM->PORT[0].u32 == 0) {
		/* Wait until STIMx is ready, then send data */
		timeout--;
		if (timeout==0) {
			return;
		}
	}

	ITM->PORT[0].u16 = (uint16_t) (0x08 | (c<<8));
}

// ----------------------------------------------------------------------------

// This function is called from _write() for fd==1 or fd==2 and from some
// of the trace_* functions.

ssize_t
trace_write (const char* buf, size_t nbyte) {
	for(size_t i = 0; i < nbyte; i++) {
		SWO_PrintChar(*buf++, 0);
	}

	return (ssize_t) nbyte;
}

#endif // TRACE

// ----------------------------------------------------------------------------

