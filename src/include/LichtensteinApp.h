/*
 * LichtensteinApp.h
 *
 * Bundles together several includes for global features and other things that
 * are used throughout the app code.
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */
#ifndef INCLUDE_LICHTENSTEINAPP_H_
#define INCLUDE_LICHTENSTEINAPP_H_

// logging
#include "LoggerGlobal.h"

// FreeRTOS
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#pragma GCC diagnostic pop

// CMSIS
#include "cmsis_device.h"

// snprintf-type functions
#include <mini-printf/mini-printf.h>

// useful includes
#ifdef __cplusplus

#endif

#endif /* INCLUDE_LICHTENSTEINAPP_H_ */
