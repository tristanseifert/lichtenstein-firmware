/*
 * System.cpp
 *
 *  Created on: Feb 16, 2018
 *      Author: tristan
 */
#define LOG_MODULE "SYS"

#include "System.h"

#include "LichtensteinApp.h"

#include "../fs/Filesystem.h"

static System *gSystem = nullptr;

using namespace sys;

/**
 * Callback for the system CPU usage output timer.
 */
static void SystemCPUOutputTimerCallback(TimerHandle_t timer) {
//	LOG(S_VERBOSE, "CPU load: %u", System::sharedInstance()->getCPULoad());
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
	this->cpuLoadOutputTimer = xTimerCreate("CPULoad", pdMS_TO_TICKS(10000),
										   pdTRUE, nullptr,
										   SystemCPUOutputTimerCallback);
	xTimerStart(this->cpuLoadOutputTimer, 0);
#endif
	// initialize the filesystem too
	Filesystem::init();
}

/**
 * Deletes the stuff we allocated earlier.
 */
System::~System() {
	// kill the timer
	xTimerStop(this->cpuLoadOutputTimer, 0);

	// delete profiler for CPU load
	delete this->cpuLoad;
}

