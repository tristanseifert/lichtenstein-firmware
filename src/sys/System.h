/*
 * System.h
 *
 *  Created on: Feb 16, 2018
 *      Author: tristan
 */

#ifndef SYS_SYSTEM_H_
#define SYS_SYSTEM_H_

#include <cstddef>

#include <LichtensteinApp.h>

#include "CPULoadProfiler.h"
#include "Random.h"

// declare friend function types
namespace sys {
	void _CPULoadProfilerContextSwitch();
	void _CPULoadProfilerTimerISR();
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class System {
	friend class sys::CPULoadProfiler;

	public:
		static void init(void);
		static System *sharedInstance(void) noexcept;

	private:
		System();
		virtual ~System();

	private:
		// random number generator
		sys::Random prng;

	public:
		uint32_t random(void) {
			return this->prng.getRandom();
		}

	private:
		friend void sys::_CPULoadProfilerContextSwitch();
		friend void sys::_CPULoadProfilerTimerISR();
		sys::CPULoadProfiler cpuLoad;

#if 1
		TimerHandle_t cpuLoadOutputTimer;
#endif

	public:
		int getCPULoad(void) const {
			return this->cpuLoad.getCPULoad();
		}

		int getCPUIdleTime(void) const {
			return this->cpuLoad.getCPUIdleTime();
		}
};

#pragma GCC diagnostic pop

#endif /* SYS_SYSTEM_H_ */
