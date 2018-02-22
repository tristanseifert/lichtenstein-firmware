/*
 * OutputTask.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_OUTPUTTASK_H_
#define LEDOUT_OUTPUTTASK_H_

#include <cstddef>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "RGBPixel.h"

namespace ledout {
	class OutputTask {
		friend class Output;

		public:
			OutputTask();
			virtual ~OutputTask() noexcept;

			int getFpsForChannel(int channel) const {
				return this->fps[channel];
			}

		private:
			friend void _OutputTaskTrampoline(void *);

			void allocBuffers(void);

			void taskEntry(void) noexcept;

		private:
			void convertBuffer(int index, int pixels = 300, void *bufferPtr = nullptr);

		private:
			// max number of output buffers we can store
			static const int maxOutputBuffers = 4;

			unsigned int ledsPerBuffer[maxOutputBuffers];

			rgbw_pixel_t *rgbwBuffer[maxOutputBuffers];

			uint8_t *outputBuffer[maxOutputBuffers];
			unsigned int outputBufferBytesWritten[maxOutputBuffers];

			unsigned int fpsCounter[maxOutputBuffers];
			unsigned int fps[maxOutputBuffers];

		private:
			friend void OutputFPSTimerCallback(TimerHandle_t);

			// size of the stack
			static const size_t TaskStackSize = 200;
			// output task priority
			static const size_t TaskPriority = 3;

			TaskHandle_t handle;
			TimerHandle_t fpsTimer;
	};

} /* namespace ledout */

#endif /* LEDOUT_OUTPUTTASK_H_ */
