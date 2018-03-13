/*
 * OutputTask.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_OUTPUTTASK_H_
#define LEDOUT_OUTPUTTASK_H_

#include <LichtensteinApp.h>

#include <cstddef>

#include "RGBPixel.h"

// forward-declare message type
#ifndef OUTPUTTASK_PRIVATE
typedef void output_message_t;
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

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

			void clearBuffer(size_t, size_t);

			void taskEntry(void) noexcept;

			void taskConvertBuffer(output_message_t *);
			void taskSendBuffer(output_message_t *);

		public:
			int sendMessage(output_message_t *msg, int timeout = portMAX_DELAY);

		private:
			void convertRGBW(int index, int numPixels, void *bufferPtr);

		public:
			// max number of output buffers we support
			static const int maxOutputBuffers = 2;

		private:
			// number of LEDs in each buffer
			unsigned int ledsPerBuffer[maxOutputBuffers];

			// output buffer (sent via SPI)
			uint8_t *outputBuffer[maxOutputBuffers];
			unsigned int outputBufferBytesWritten[maxOutputBuffers];

			// frame counters
			unsigned int fpsCounter[maxOutputBuffers];
			unsigned int fps[maxOutputBuffers];

		private:
			friend void OutputFPSTimerCallback(TimerHandle_t);

			// size of the stack
			static const size_t TaskStackSize = 200;
			// output task priority
			static const size_t TaskPriority = 3;
			// how many messages can be pending at a time in the message queue
			static const size_t MessageQueueDepth = 4;

			TaskHandle_t handle = nullptr;
			TimerHandle_t fpsTimer = nullptr;

			QueueHandle_t messageQueue = nullptr;
	};
} /* namespace ledout */

#pragma GCC diagnostic pop

#endif /* LEDOUT_OUTPUTTASK_H_ */
