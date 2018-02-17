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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "LoggerGlobal.h"

#include <stdarg.h>

class Logger {
	public:
		static Logger *sharedInstance(void) noexcept;

	private:
		Logger();
		virtual ~Logger();

	private:
		bool printToTrace = true;

		TaskHandle_t task;
		QueueHandle_t messageQueue;

		friend void _LoggerTaskTrampoline(void *);

		void taskEntry(void);

	private:
		friend int _LoggerDoLog(logger_severity_t, const char *, const char *, int, const char *, ...);

		int log(logger_severity_t severity, const char *module, const char *file, int line, const char *format, va_list ap);
};

#endif /* SYS_LOGGER_H_ */
