/*
 * Logger.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */
#define LOG_MODULE "LOG"

#include "Logger.h"
#include "LoggerPrivate.h"

#include "LichtensteinApp.h"

#include "mini-printf/mini-printf.h"
#include "diag/Trace.h"

#include <stdarg.h>
#include <cstring>

// shared instance of the logger
static Logger *gLogger = nullptr;

// static SHARED buffer
static const int gPrintBufferSz = 200;
static char gPrintBuffer[gPrintBufferSz];

// buffer for output to the trace port
static const int gTraceBufferSz = 256;
static char gTraceBuffer[gTraceBufferSz];

// when set, the message handler task is running
static bool taskRunning = false;

SemaphoreHandle_t gBufferMutex;

/**
 * Entry point for the logger task.
 */
void _LoggerTaskTrampoline(void *ctx) {
	(static_cast<Logger *>(ctx))->taskEntry();
}


/**
 * Returns the shared logger instance.
 */
Logger *Logger::sharedInstance() noexcept {
	// allocate the logger if needed
	taskENTER_CRITICAL();

	if(!gLogger) {
		gLogger = new Logger();
	}

	taskEXIT_CRITICAL();

	// return it
	return gLogger;
}


/**
 * Initializes the logger.
 */
Logger::Logger() {
	// create mutex
	gBufferMutex = xSemaphoreCreateMutex();

	// initialize the trace facility
	trace_initialize();

	trace_putchar('\n');
	trace_putchar('\n');
	trace_putchar('\n');

	// create logger task
	BaseType_t ok;

	ok = xTaskCreate(_LoggerTaskTrampoline, "Logger", 200, this, 4, &this->task);

	if(ok != pdPASS) {
		LOG(S_FATAL, "Couldn't create logger task!");
	}

	// create logger queue
	this->messageQueue = xQueueCreate(10, sizeof(logger_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create logging queue");
	}
}

/**
 * Kills the task and deletes the message queue.
 */
Logger::~Logger() {
	// have the log function go straight to the trace output
	taskRunning = false;

	// delete task and queue
	vTaskDelete(this->task);
	vQueueDelete(this->messageQueue);
}

/**
 * Entry point for the logging task.
 */
void Logger::taskEntry(void) {
	int ret;
	logger_message_t msg;
	BaseType_t dequeued;

	// task is running
	taskRunning = true;

	// enter a forever loop
	while(1) {
		// dequeue a message at a time
		dequeued = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		if(!dequeued) {
			LOG(S_ERROR, "Returned from xQueueReceive with no item returned");
			continue;
		}

		// handle the message
		if(this->printToTrace) {
			const char *module = (msg.module == nullptr) ? "???" : msg.module;

			ret = mini_snprintf(gTraceBuffer, gTraceBufferSz,
								"[%u %s %s:%u]: %s\n", msg.sev, module,
								msg.file, msg.line, msg.buffer);
			trace_write(gTraceBuffer, ret);
		}

		// call any additional handlers

		// de-allocate the buffer
		vPortFree(msg.buffer);
	}
}



/**
 * Performs actual logging. Returns the length of the debug message if sent.
 */
int Logger::log(bool fromISR, logger_severity_t severity, const char *module, const char *file, int line, const char *format, va_list ap) {
	int ret;
	char *bufferCopy = nullptr;

	// lock the buffer
	xSemaphoreTake(gBufferMutex, portMAX_DELAY);

	// Print to the local buffer
	ret = mini_vsnprintf(gPrintBuffer, gPrintBufferSz, format, ap);

	if(ret) {
		// create a copy of the buffer
		bufferCopy = (char *) pvPortMalloc((ret + 1));
		memset(bufferCopy, 0, (ret + 1));
		memcpy(bufferCopy, gPrintBuffer, ret);
	} else {
		// if the printf didn't evaluate to anything, exit
		xSemaphoreGive(gBufferMutex);

		goto done;
	}

	// if the FreeRTOS scheduler is running, push it to the log task
	if(taskRunning && fromISR == false) {
		BaseType_t queued;

		// release the semaphore to the global buffer
		xSemaphoreGive(gBufferMutex);

		// create the message
		logger_message_t msg;

		msg.buffer = bufferCopy;
		msg.file = file;
		msg.line = line;
		msg.module = module;
		msg.sev = severity;

		// queue it, dropping the message if not queued
		queued = xQueueSendToBack(this->messageQueue, &msg, portMAX_DELAY);

		if(queued != pdTRUE) {
			vPortFree(bufferCopy);
		}
	}
	// otherwise, just send it via the trace output
	else {
		ret = mini_snprintf(gPrintBuffer, gPrintBufferSz, "{%u %s:%u}: %s\n", severity, file, line, bufferCopy);

		trace_write(gPrintBuffer, ret);
		vPortFree(bufferCopy);

		// release the semaphore to the global buffer
		xSemaphoreGive(gBufferMutex);
	}

	done: ;
	return ret;
}

extern "C" int _LoggerDoLog(bool fromISR, logger_severity_t severity, const char *module, const char *file, int line, const char *format, ...) {
	int ret;

	// set up the variadic arguments and call into the logger
	va_list ap;
	va_start(ap, format);

	ret = Logger::sharedInstance()->log(fromISR, severity, module, file, line, format, ap);

	va_end(ap);
	return ret;
}
