/*
 * Logger.h
 *
 * Handles logging for the entire app. Logs are tagged with the source file and
 * line they occurred in, as well as an optional module name.
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */

#ifndef SYS_LOGGER_H_
#define SYS_LOGGER_H_

#include <LichtensteinApp.h>

#include "LoggerGlobal.h"

#include <cstddef>
#include <cstdarg>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

// declare linkeage of the ISR
extern "C" void DMA1_Channel7_IRQHandler(void);

class Logger {
	public:
		static Logger *sharedInstance(void) noexcept;

	private:
		Logger();
		virtual ~Logger();

	private:
		void initUART(void);
		void initUARTDMA(void);

		void printMessage(char *, size_t, bool fromISR = false);

	private:
		friend void DMA1_Channel7_IRQHandler(void);

		SemaphoreHandle_t uartDMALock = nullptr;

	private:
		// how many pending log messages can there be?
		static const size_t logQueueLength = 16;
		// priority of the logger task
		static const int TaskPriority = 1;
		// stack size for the logger task
		static const size_t TaskStackSize = 200;

		bool printToTrace = true;

		TaskHandle_t task = nullptr;
		QueueHandle_t messageQueue = nullptr;

		friend void _LoggerTaskTrampoline(void *);

		void taskEntry(void);

	private:
		friend int _LoggerDoLog(bool, logger_severity_t, const char *, const char *, int, const char *, ...);

		int log(bool fromISR, logger_severity_t severity, const char *module, const char *file, int line, const char *format, va_list ap);
};

#pragma GCC diagnostic pop

#endif /* SYS_LOGGER_H_ */
