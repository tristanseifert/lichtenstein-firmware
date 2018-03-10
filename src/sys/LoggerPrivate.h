/*
 * LoggerPrivate.h
 *
 * Private types used by the logger.
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */

#ifndef SYS_LOGGERPRIVATE_H_
#define SYS_LOGGERPRIVATE_H_

#include "LoggerGlobal.h"

/**
 * A log message as sent to the logger task.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	/// severity of the message
	logger_severity_t sev;

	/// module name
	const char *module;
	/// file from which the log was triggered, if available
	const char *file;
	/// line in the file from which the log originated
	int line;

	// pointer to a buffer of log message (we need to free this)
	char *buffer;
} logger_message_t;

#pragma GCC diagnostic pop



#endif /* SYS_LOGGERPRIVATE_H_ */
