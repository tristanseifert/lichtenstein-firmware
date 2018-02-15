/*
 * Output.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_OUTPUT_H_
#define LEDOUT_OUTPUT_H_

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"

#include "OutputTask.h"

class Output {
	friend class ledout::OutputTask;

	public:
		static void init(void);
		static Output *sharedInstance(void) noexcept;

	private:
		Output();

	private:
		void initOutputGPIOs(void);
		void initOutputDMA(void);

		void setOutputEnable(bool enable);

		int outputData(int channel, void *data, size_t length);

	private:
		static const int NumOutputChannels = 2;

		SemaphoreHandle_t dmaSemaphores[NumOutputChannels];

	private:
		virtual ~Output();

	private:
		ledout::OutputTask *task;
};

#endif /* LEDOUT_OUTPUT_H_ */
