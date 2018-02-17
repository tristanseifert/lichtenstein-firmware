/*
 * System.h
 *
 *  Created on: Feb 16, 2018
 *      Author: tristan
 */

#ifndef SYS_SYSTEM_H_
#define SYS_SYSTEM_H_

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "timers.h"

#include "CPULoadProfiler.h"

class System {
	friend class sys::CPULoadProfiler;

	public:
		static void init(void);
		static System *sharedInstance(void) noexcept;

	private:
		System();
		virtual ~System();

	private:
		friend void sys::_CPULoadProfilerContextSwitch();
		friend void sys::_CPULoadProfilerTimerISR();
		sys::CPULoadProfiler *cpuLoad;

#if 1
		TimerHandle_t cpuLoadOutputTimer;
#endif

	public:
		int getCPULoad(void) const {
			return cpuLoad->getCPULoad();
		}

		int getCPUIdleTime(void) const {
			return cpuLoad->getCPUIdleTime();
		}
};

#endif /* SYS_SYSTEM_H_ */
