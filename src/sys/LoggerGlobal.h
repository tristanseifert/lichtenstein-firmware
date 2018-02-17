/*
 * LoggerGlobal.h
 *
 * Global includes for logging.
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */

#ifndef SYS_LOGGERGLOBAL_H_
#define SYS_LOGGERGLOBAL_H_

#include <stdint.h>
#include <unistd.h>

/**
 * Logging severity. Messages with a severity lower than what the logger is
 * configured for will be discarded.
 */
typedef enum {
	kSeverityFatal	= (1 << 5),
	kSeverityError	= (1 << 4),
	kSeverityWarning	= (1 << 3),
	kSeverityInfo	= (1 << 2),
	kSeverityDebug	= (1 << 1),
	kSeverityVerbose	= (1 << 0)
} logger_severity_t;


/**
 * Define the logging function: this is printf-like after the first few
 * arguments to do with tracking the module and file.
 */
extern "C" int _LoggerDoLog(logger_severity_t severity, const char *module, const char *file, int line, const char *format, ...);

/**
 * Logging macro.
 */
#ifdef LOG_MODULE
	#define LOG(sev, format...)_LoggerDoLog(sev, LOG_MODULE, __FILE__, __LINE__, format)
#else
	#define LOG(sev, format...)_LoggerDoLog(sev, 0, __FILE__, __LINE__, format)
#endif

#define S_FATAL								kSeverityFatal
#define S_ERROR								kSeverityError
#define S_WARN								kSeverityWarning
#define S_INFO								kSeverityInfo
#define S_DEBUG								kSeverityDebug
#define S_VERBOSE							kSeverityVerbose

#endif /* SYS_LOGGERGLOBAL_H_ */
