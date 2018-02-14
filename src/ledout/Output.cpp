/*
 * Output.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "Output.h"

#include "cmsis_device.h"

static Output *gOutput = nullptr;

using namespace ledout;

/**
 * Allocates the shared output handler.
 */
void Output::init(void) {
	if(!gOutput) {
		gOutput = new Output();
	}
}

/**
 * Returns the shared output handler instance.
 */
Output *Output::sharedInstance() noexcept {
	return gOutput;
}

/**
 * Instantiates the output handler. This will set up a LED output task and the
 * peripherals needed to drive it.
 */
Output::Output() {
	this->task = new OutputTask();
}

Output::~Output() {
	delete this->task;
}

