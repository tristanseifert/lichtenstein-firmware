/*
 * Output.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_OUTPUT_H_
#define LEDOUT_OUTPUT_H_

#include "OutputTask.h"

class Output {
	public:
		static void init(void);
		static Output *sharedInstance(void) noexcept;

	private:
		Output();
		virtual ~Output();

	private:
		ledout::OutputTask *task;
};

#endif /* LEDOUT_OUTPUT_H_ */
