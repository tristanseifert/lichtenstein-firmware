/*
 * SST25VF016.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */
#define LOG_MODULE "FHAL"

#include "SST25VF016.h"

#include <LichtensteinApp.h>

// writes larger than this will use DMA
#define DMA_THRESHOLD			(1024*1024*4)

// set this variable to log invocations to the IO functions
#define LOG_IO					0

namespace fs {

/**
 * Initializes the chip.
 *
 * On initialization, read the status register, check for write protection,
 * and if enabled, disable it.
 */
SST25VF016::SST25VF016(Filesystem *_fs) : FlashHAL(_fs) {
	// make the flash writeable
	this->checkWriteProtectState();
}

/**
 * Cleans up resources we've allocated.
 */
SST25VF016::~SST25VF016() {

}



/**
 * Waits the amount of time required to write a byte: this is approximately
 * 10µS.
 */
void SST25VF016::flashWaitTBP(void) {
	// enter critical section
	taskENTER_CRITICAL();

	volatile int timeout = 150;

	while(timeout-- != 0) {

	}

	// exit critical section
	taskEXIT_CRITICAL();
}

/**
 * Waits for a sector erase to complete: this is approximately 25ms.
 *
 * TODO: use the hardware method to check this
 */
void SST25VF016::flashWaitSectorErase(void) {
	// TODO: use a timer peripheral with an interrupt instead
//	vTaskDelay((25 / portTICK_PERIOD_MS));
	vTaskDelay(4);
}



/**
 * Reads data from the flash, starting at the given address.
 */
int SST25VF016::read(uint32_t address, size_t size, void *dst) {
	int err;

#if LOG_IO
	LOG(S_DEBUG, "Reading %u bytes from 0x%06x, buffer at 0x%x", size, address, dst);
#endif

	// DMA can do a maximum of 64K so limit to that
	if(size >= 0xFFFF) {
		LOG(S_ERROR, "Couldn't start transaction for read");

		return -1;
	}


	// start a flash transaction
	if(this->fs->startFlashTransaction() != 0) {
		return -1;
	}

	// send the read command and address
	// TODO: implement fast reading support
	err = this->fs->flashCommandWithAddress(SST25VF016::cmdReadSlow, address);

	if(err != 0) {
		// end transaction
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send read command with address 0x%06x", address);

		return err;
	}

	// if we have more than this threshold, use DMA; otherwise, read in a loop
	if(size > DMA_THRESHOLD) {
		// TODO: implement DMA
	} else {
		uint8_t *outBuf = reinterpret_cast<uint8_t *>(dst);

		for(unsigned int i = 0; i < size; i++) {
			// read a byte
			outBuf[i] = this->fs->spiRead();
		}
	}

	// end the flash transaction we started earlier
	this->fs->endFlashTransaction();

	// if we get down here, everything should be good
	return 0;
}

/**
 * Writes to the flash. The address _must_ be even.
 */
int SST25VF016::write(uint32_t address, size_t size, void *src) {
	int err;

#if LOG_IO
	LOG(S_DEBUG, "Writing %u bytes to 0x%06x, buffer at 0x%x", size, address, src);
#endif

	// DMA can do a maximum of 64K so limit to that
	if(size >= 0xFFFF) {
		return -1;
	}

	// the address _must_ be even
	if((address & 1)) {
		LOG(S_ERROR, "Attempt to write to odd address 0x%08x", address);
		return -1;
	}


	// start a flash transaction
	if(this->fs->startFlashTransaction() != 0) {
		LOG(S_ERROR, "Couldn't start transaction for write");

		return -1;
	}

	// we need to enable writing so send the write enable command
	err = this->fs->flashCommand(SST25VF016::cmdWriteEnable);

	if(err != 0) {
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send write enable");
		return err;
	}

	this->fs->spiPulseCS();

	// send the write command and address
	err = this->fs->flashCommandWithAddress(SST25VF016::cmdWriteAIW, address);

	if(err != 0) {
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send write command with address 0x%06x", address);
		return err;
	}

	// write two bytes at a time
	uint8_t *outBuf = reinterpret_cast<uint8_t *>(src);
	unsigned int bytesWritten = 0, endOfCommand = 0;

	for(unsigned int i = 0; i < (size & 0xFFFFFFFE); i++) {
		this->fs->spiWrite(outBuf[i]);
		bytesWritten++; endOfCommand++;

		// have two bytes been written?
		if(endOfCommand == 2) {
			// wait for the data to be sent and the peripheral to be idle
			this->fs->spiWaitIdle();

			// de-assert CS, wait for the write operation to complete
			this->fs->setFlashCS(false);
			this->flashWaitTBP();

			// was this the last byte to write?
			if(bytesWritten != size) {
				// re-assert /CS, then send another write command
				this->fs->setFlashCS(true);

				err = this->fs->flashCommand(SST25VF016::cmdWriteAIW);

				if(err != 0) {
					this->fs->endFlashTransaction();

					LOG(S_ERROR, "Couldn't send write command with address 0x%06x, wrote %u bytes", address, bytesWritten);
					return err;
				}
			}

			// reset the counter
			endOfCommand = 0;
		}
	}

	// pulse /CS so the write command begins, then disable writing
	this->fs->setFlashCS(false);
	this->flashWaitTBP();

	// are there any risidual bytes to write?
	if(bytesWritten < size) {
		// re-assert /CS and send write command
		this->fs->setFlashCS(true);

		address += bytesWritten;
		err = this->fs->flashCommandWithAddress(SST25VF016::cmdWriteByte, address);

		if(err != 0) {
			this->fs->endFlashTransaction();

			LOG(S_ERROR, "Couldn't send write command with address 0x%06x", address);
			return err;
		}

		// send the data byte
		this->fs->spiWrite(outBuf[bytesWritten++]);
		bytesWritten++;

		// wait for the data to be sent and the peripheral to be idle
		this->fs->spiWaitIdle();

		// de-assert CS, wait for the write operation to complete
		this->fs->setFlashCS(false);
		this->flashWaitTBP();
	}

	LOG(S_DEBUG, "Wrote %u bytes, expected %u", bytesWritten, size);

	// put the flash back in write disable state
	this->fs->setFlashCS(true);
	err = this->fs->flashCommand(SST25VF016::cmdWriteDisable);

	if(err != 0) {
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send write disable");
		return err;
	}

	// end the flash transaction we started earlier
	this->fs->endFlashTransaction();

	// if we get down here, everything should be good
	return 0;
}

/**
 * Performs an erase operation over a block.
 */
int SST25VF016::erase(uint32_t address, size_t size) {
	int err;

#if LOG_IO
	LOG(S_DEBUG, "Erasing %u bytes from 0x%06x", size, address);
#endif

	// TODO: implement hangling bigger cases than a 4K page
	if(size != 0x1000) {
		LOG(S_ERROR, "Erasing sizes other than 4K is unsupported");
		return -1;
	}


	// start a flash transaction
	if(this->fs->startFlashTransaction() != 0) {
		LOG(S_ERROR, "Couldn't start transaction for erase");

		return -1;
	}

	// send write enable, then pulse CS to start the command
	err = this->fs->flashCommand(SST25VF016::cmdWriteEnable);

	if(err != 0) {
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send write enable");
		return err;
	}

	this->fs->spiPulseCS();

	// send the write command and address
	err = this->fs->flashCommandWithAddress(SST25VF016::cmdErase4kBlock, address);

	if(err != 0) {
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send erase command with address 0x%06x", address);
		return err;
	}

	// wait for the erase to complete
	this->fs->setFlashCS(false);
	this->flashWaitSectorErase();

	// disable writing again
	this->fs->setFlashCS(true);

	err = this->fs->flashCommand(SST25VF016::cmdWriteDisable);

	if(err != 0) {
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send write disable");
		return err;
	}

	// end the flash transaction we started earlier
	this->fs->endFlashTransaction();

	// if we get down here, everything should be good
	return 0;
}



/**
 * Reads the status register.
 */
uint8_t SST25VF016::getStatus(void) {
	int err;
	uint8_t statusReg;

	// start a flash transaction
	if(this->fs->startFlashTransaction() != 0) {
		LOG(S_ERROR, "Couldn't start transaction for status register read");

		return 0;
	}

	// send the status register read command
	err = this->fs->flashCommand(SST25VF016::cmdReadStatus);

	if(err != 0) {
		// end transaction
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send RDSR command");
		return 0;
	}

	// read the status register
	statusReg = this->fs->spiRead();

	// end the flash transaction we started earlier
	this->fs->endFlashTransaction();

	// return the value we read
	return statusReg;
}

/**
 * Writes the status register.
 */
void SST25VF016::setStatus(uint8_t newStatus) {
	int err;

	// start a flash transaction
	if(this->fs->startFlashTransaction() != 0) {
		LOG(S_ERROR, "Couldn't start transaction for status register write");

		return;
	}

	// send the "enable status register write" command
	err = this->fs->flashCommand(SST25VF016::cmdEnableStatusWrite);

	if(err != 0) {
		// end transaction
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send EWSR command");
		return;
	}

	// pulse CS, then write the status register
	this->fs->spiPulseCS();

	err = this->fs->flashCommand(SST25VF016::cmdWriteStatus);

	if(err != 0) {
		// end transaction
		this->fs->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send WRSR command");
		return;
	}

	this->fs->spiWrite(newStatus);

	// end the flash transaction we started earlier
	this->fs->endFlashTransaction();
}



/**
 * Checks whether the flash has the write protect bits set in the status
 * register; and if so, attempts to clear them.
 */
void SST25VF016::checkWriteProtectState(void) {
	uint8_t status = this->getStatus();

	if(status & SST25VF016::statusBPMask) {
		LOG(S_INFO, "Flash is write protected (0x%02x)", status);

		this->unWriteProtect();
	}
}

/**
 * Removes write protection by clearing the BPL bits.
 */
void SST25VF016::unWriteProtect(void) {
	uint8_t status = this->getStatus();

	/*
	 * To clear the write protection, we need to mask out the BP bits in
	 * the status register, but also clear the BPL bit.
	 */
	status &= ~SST25VF016::statusBPMask;
	status &= ~SST25VF016::statusBPL;

	this->setStatus(status);

	// read the status reg and check if write protection is gone
	status = this->getStatus();

	if(status & SST25VF016::statusBPMask) {
		LOG(S_FATAL, "Couldn't clear BP bits; check /WP");
	}
}

} /* namespace fs */
