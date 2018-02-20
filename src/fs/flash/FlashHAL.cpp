/*
 * FlashHAL.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */
#define LOG_MODULE "FHAL"

#include "FlashHAL.h"

#include "../FlashCommands.h"

#include <LichtensteinApp.h>

namespace fs {

/**
 * Initializes the flash HAL.
 */
FlashHAL::FlashHAL(Filesystem *_fs) : fs(_fs) {


}

/**
 * We don't do anything here.
 */
FlashHAL::~FlashHAL() {

}



/**
 * Reads the JDEC ID from the flash. We _assume_ that this will almost always
 * be the same across chips from different vendors, so we provide a common
 * implementation here. If that's not the case, it can obviously be
 * overwritten in a subclass.
 */
uint32_t FlashHAL::getJDECID(void) {
	int err;
	uint32_t id = 0;

	// start a flash transaction
	if(this->fs->startFlashTransaction() != 0) {
		LOG(S_ERROR, "Couldn't start flash transaction for JDEC ID read");

		return 0;
	}

	// send the JDEC ID command
	err = this->fs->flashCommand(FlashHAL::cmdJDECID);

	if(err != 0) {
		// end transaction
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send JDEC ID command");
		return 0;
	}

	// read the manufacturer byte
	id |= (this->fs->spiRead() << 16);
	// read the memory type byte
	id |= (this->fs->spiRead() << 8);
	// read the memory size byte
	id |= this->fs->spiRead();

	// end the flash transaction we started earlier
	this->fs->endFlashTransaction();

	return id;
}

} /* namespace fs */
