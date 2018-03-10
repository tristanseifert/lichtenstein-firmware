/*
 * CPULoadProfiler.h
 *
 * Sets up a timer that fires an interrupt every second, and retrieves the
 * value of another timer that's started when the idle task begins to run,
 * and stopped when another task runs. This way, we measure the percentage of
 * time that the processor is idle, and by negating that, can get the CPU load.
 *
 *  Created on: Feb 16, 2018
 *      Author: tristan
 */

#ifndef SYS_CPULOADPROFILER_H_
#define SYS_CPULOADPROFILER_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace sys {
	class CPULoadProfiler {
		public:
			CPULoadProfiler();
			virtual ~CPULoadProfiler();

		public:
			/**
			 * Returns the percentage of time that the CPU was executing user
			 * tasks over the past second: this is the complement of the idle
			 * time.
			 */
			int getCPULoad(void) const {
				return 100 - this->getCPUIdleTime();
			}

			/**
			 * Gets the percentage of time that the CPU was idle over the past
			 * second.
			 */
			int getCPUIdleTime(void) const {
				return this->lastIdleTime;
			}

		private:
			friend void _CPULoadProfilerTimerISR(void);

			// percentage of time the CPU is idle, [0, 100]
			int lastIdleTime = 0;
			// timer ticks that we spent in the idle task
			int idleTime = 0;

			// last time counter value when the idle task was entered
			int lastTimerValueOnIdleEnter = 0;
			// set when we enter the idle task, cleared on the next switch
			bool isInIdleTask = false;

		private:
			friend void _CPULoadProfilerContextSwitch();

			void handleContextSwitchFromKernel();
	};

} /* namespace sys */

#pragma GCC diagnostic pop

#endif /* SYS_CPULOADPROFILER_H_ */
