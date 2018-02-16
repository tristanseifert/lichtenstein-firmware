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

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "cmsis_device.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board/Board.h"
#include "clock/Clock.h"
#include "net/Network.h"

#include "ledout/Output.h"

/**
 * Stack overflow handler
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
	trace_printf("Stack overflow in task %s!", pcTaskName);

	while(1);
}

/**
 * Define heap sections. We want to set this up so that the heap spans the
 * entirety of the unused BSS.
 */
extern uint32_t __ram_size__[];
extern uint32_t _Heap_Begin[];
extern uint32_t _Heap_Size[];

const HeapRegion_t xHeapRegions[] = {
    { (uint8_t *) _Heap_Begin, (size_t) _Heap_Size},
    { NULL, 0 } /* Terminates the array. */
};

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[]) {
	// set the priority levels in the NVIC (required for FreeRTOS)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// initialize debug SWO output
	trace_initialize();
	trace_printf("lichtenstein %s\nSystemCoreClock = %uHz, %u bytes of RAM\n", GIT_REV, SystemCoreClock, __ram_size__);

	// set up FreeRTOS heap
	trace_printf("Allocating heap region at 0x%x sized %u bytes\n", xHeapRegions[0].pucStartAddress, xHeapRegions[0].xSizeInBytes);
	vPortDefineHeapRegions(xHeapRegions);

	// set up hardware
	Board::init();
	Network::init();

	Clock::init();

	// start tasks and other higher-level facilities
	Output::init();

	// some debugging to diagnose memory usage
	trace_printf("heap usage: %u bytes free, %u bytes total\n",
				 xPortGetFreeHeapSize(), _Heap_Size);

	// start scheduler (this should never return!)
    vTaskStartScheduler();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
