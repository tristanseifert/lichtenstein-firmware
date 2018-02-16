/*
 * OutputTask.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "OutputTask.h"

#include <Errors.h>

// configuration
#define LEDOUT_TASK_STACK_SZ 	configMINIMAL_STACK_SIZE
#define LEDOUT_TASK_PRIORITY		(3)

namespace ledout {

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _OutputTaskTrampoline(void *ctx) {
	(static_cast<OutputTask *>(ctx))->taskEntry();
}

/**
 * Sets up the LED output task.
 *
 * @note If the task couldn't be created, an exception is thrown.
 */
OutputTask::OutputTask() {
	BaseType_t ok;

	ok = xTaskCreate(_OutputTaskTrampoline, "LEDOut", LEDOUT_TASK_STACK_SZ,
					 this, LEDOUT_TASK_PRIORITY, &this->handle);

	if(ok != pdPASS) {
		trace_puts("Couldn't create LEDOut task!");
	}
}

/**
 * If the task is still running, kill it.
 */
OutputTask::~OutputTask() {
	// TODO: do this a bit nicer so the task can deallocate its resources
	vTaskDelete(this->handle);
}

/**
 * Entry point for the task.
 */
void OutputTask::taskEntry(void) noexcept {
	int i = 0;

	while(1) {
//		trace_printf("LED out task iteration %u\n", i++);
		vTaskDelay(1000);
	}
}

} /* namespace ledout */
