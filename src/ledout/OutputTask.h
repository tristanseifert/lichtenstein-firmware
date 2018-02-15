/*
 * OutputTask.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_OUTPUTTASK_H_
#define LEDOUT_OUTPUTTASK_H_

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

namespace ledout {
	class OutputTask {
		friend class Output;

		public:
			OutputTask();
			virtual ~OutputTask() noexcept;

		private:
			friend void _OutputTaskTrampoline(void *);

			void taskEntry(void) noexcept;

		private:
			TaskHandle_t handle;
	};

} /* namespace ledout */

#endif /* LEDOUT_OUTPUTTASK_H_ */
