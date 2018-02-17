/*
 * System.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: tristan
 */
#include "System.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "cmsis_device.h"

#include "../fs/Filesystem.h"

static System *gSystem = nullptr;

using namespace sys;

/**
 * Callback for the system CPU usage output timer.
 */
static void SystemCPUOutputTimerCallback(TimerHandle_t timer) {
	trace_printf("CPU load: %u, idle = %u\n", System::sharedInstance()->getCPULoad(), System::sharedInstance()->getCPUIdleTime());
}


/**
 * Allocates the shared system handler.
 */
void System::init(void) {
	taskENTER_CRITICAL();

	if(!gSystem) {
		gSystem = new System();
	}

	taskEXIT_CRITICAL();
}

/**
 * Returns the shared system handler instance.
 */
System *System::sharedInstance() noexcept {
	return gSystem;
}

/**
 * Initializes the system, allocating the various profiling mechanisms.
 */
System::System() {
	// allocate the CPU time profiler
	this->cpuLoad = new CPULoadProfiler();

#if 1
	this->cpuLoadOutputTimer = xTimerCreate("CPULoad", pdMS_TO_TICKS(1000),
										   pdTRUE, nullptr,
										   SystemCPUOutputTimerCallback);
//	xTimerStart(this->cpuLoadOutputTimer, 0);
#endif
	// initialize the filesystem too
	Filesystem::init();
}

/**
 * Deletes the stuff we allocated earlier.
 */
System::~System() {
	delete this->cpuLoad;
}

