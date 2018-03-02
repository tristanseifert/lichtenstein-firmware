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

// declare ISR linkeage
extern "C" void DMA1_Channel5_IRQHandler(void);
extern "C" void DMA2_Channel2_IRQHandler(void);

class Output {
	friend class ledout::OutputTask;

	friend void DMA1_Channel5_IRQHandler(void);
	friend void DMA2_Channel2_IRQHandler(void);

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
		// locks used to sync DMA
		SemaphoreHandle_t dmaSemaphores[NumOutputChannels];
		// number of writes to output channels
		volatile unsigned int activeOutputs = 0;

	private:
		virtual ~Output();

	private:
		ledout::OutputTask *task;
};

#endif /* LEDOUT_OUTPUT_H_ */
