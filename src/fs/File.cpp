/*
 * File.cpp
 *
 *  Created on: Mar 23, 2018
 *      Author: tristan
 */
#define LOG_MODULE "FS"

#define FILE_PRIVATE

#include <fs/File.h>

#include "Filesystem.h"
#include "FSPrivate.h"

#include <LichtensteinApp.h>

#include <cstring>
#include <cstdint>
#include <cstddef>

namespace fs {

/**
 * Allocates a new file struct, given a particular file descriptor.
 */
File::File(fs_descriptor_t _descriptor) : descriptor(_descriptor) {
	// the file is open now
	this->isOpen = true;
}

/**
 * Deallocates the file, closing it if it's still open.
 */
File::~File() {
	if(this->isOpen) {
		this->close();
	}
}



/**
 * Closes the file.
 *
 * @return 0 if successful, error code otherwise.
 */
int File::close(void) {
	int err;

	// check file is open
	if(!this->isOpen) {
		LOG(S_ERROR, "Attempted to close an already closed file 0x%08x", this);
		return -1;
	}

	// structures for the message
	fs_message_t msg;

	bool success = false;
	uint32_t returnValue = 0;

	// set up message
	err = this->prepareMessage(&msg, kFSMessageCloseFile);

	if(err != 0) {
		return err;
	}

	msg.successPtr = &success;
	msg.returnValuePtr = &returnValue;

	// attempt to post message
	err = Filesystem::sharedInstance()->postFSRequest(&msg);

	if(err != 0) {
		vSemaphoreDelete(msg.completion);
		return err;
	}

	// wait for completion
	err = this->waitOnMessage(&msg);
	if(err != 0) return err;

	// return status code
	if(!success) {
		LOG(S_ERROR, "Couldn't close: %d", returnValue);
	} else {
		// mark file as closed
		this->isOpen = false;
	}

	return returnValue;
}

/**
 * Reads from the file.
 *
 * @param buffer Buffer into which data is read
 * @param length Length of buffer
 * @return Number of bytes read, or a negative error code.
 */
int File::read(void *buffer, size_t length) {
	int err;

	// structures for the message
	fs_message_t msg;

	bool success = false;
	uint32_t returnValue = 0;

	// set up message
	err = this->prepareMessage(&msg, kFSMessageRead);

	if(err != 0) {
		return err;
	}

	msg.successPtr = &success;
	msg.returnValuePtr = &returnValue;

	msg.payload.read.buffer = buffer;
	msg.payload.read.length = length;

	// attempt to post message
	err = Filesystem::sharedInstance()->postFSRequest(&msg);

	if(err != 0) {
		vSemaphoreDelete(msg.completion);
		return err;
	}

	// wait for completion
	err = this->waitOnMessage(&msg);
	if(err != 0) return err;

	// return status code
	if(!success) {
		LOG(S_ERROR, "Couldn't read: %d", returnValue);
	}

	return returnValue;
}

/**
 * Writes to the file.
 *
 * @param buffer Buffer containing data to write
 * @param length Length of buffer
 * @return Number of bytes written, or a negative error code.
 */
int File::write(void *buffer, size_t length) {
	int err;

	// structures for the message
	fs_message_t msg;

	bool success = false;
	uint32_t returnValue = 0;

	// set up message
	err = this->prepareMessage(&msg, kFSMessageWrite);

	if(err != 0) {
		return err;
	}

	msg.successPtr = &success;
	msg.returnValuePtr = &returnValue;

	msg.payload.write.buffer = buffer;
	msg.payload.write.length = length;

	// attempt to post message
	err = Filesystem::sharedInstance()->postFSRequest(&msg);

	if(err != 0) {
		vSemaphoreDelete(msg.completion);
		return err;
	}

	// wait for completion
	err = this->waitOnMessage(&msg);
	if(err != 0) return err;

	// return status code
	if(!success) {
		LOG(S_ERROR, "Couldn't write: %d", returnValue);
	}

	return returnValue;
}

/**
 * Gets the current position in the file.
 *
 * @return Offset into the file, negative if error.
 */
int File::tell(void) {
	return this->seek(File::CURRENT, 0);
}

/**
 * Seeks to the specified position in the file.
 *
 * @param mode One of END, SET, or CURRENT
 * @param offset Signed offset
 *
 * @return New position if success, negative if error.
 */
int File::seek(int mode, ssize_t offset) {
	int err;

	// structures for the message
	fs_message_t msg;

	bool success = false;
	uint32_t returnValue = 0;

	// set up message
	err = this->prepareMessage(&msg, kFSMessageSeek);

	if(err != 0) {
		return err;
	}

	msg.successPtr = &success;
	msg.returnValuePtr = &returnValue;

	if(mode == File::END) {
		msg.payload.seek.mode = SPIFFS_SEEK_END;
	} else if(mode == File::SET) {
		msg.payload.seek.mode = SPIFFS_SEEK_SET;
	} else if(mode == File::CURRENT) {
		msg.payload.seek.mode = SPIFFS_SEEK_CUR;
	} else {
		LOG(S_ERROR, "Invalid mode: %d", mode);

		// clean up and return error
		vSemaphoreDelete(msg.completion);
		return -1;
	}

	msg.payload.seek.offset = offset;


	// attempt to post message
	err = Filesystem::sharedInstance()->postFSRequest(&msg);

	if(err != 0) {
		vSemaphoreDelete(msg.completion);
		return err;
	}

	// wait for completion
	err = this->waitOnMessage(&msg);
	if(err != 0) return err;

	// return status code
	if(!success) {
		LOG(S_ERROR, "Couldn't seek: %d", returnValue);
	}

	return returnValue;
}



/**
 * Prepares a message struct and allocates a completion semaphore for it.
 *
 * @param msg Message to prepare
 * @return 0 if success, error code otherwise.
 */
int File::prepareMessage(fs_message_t *msg, fs_message_type_t type) {
	// allocate the semaphore
	SemaphoreHandle_t completion = xSemaphoreCreateBinary();

	if(completion == nullptr) {
		LOG(S_ERROR, "Can't allocate completion semaphore");
		return -1;
	}

	// set up message
	memset(msg, 0, sizeof(fs_message_t));

	msg->completion = completion;

	// write in descriptor and type
	msg->descriptor = &this->descriptor;
	msg->type = type;

	return 0;
}

/**
 * Waits for the action this message requests to be completed.
 *
 * @param msg Message to wait on
 * @return 0 if success, error code otherwise.
 */
int File::waitOnMessage(fs_message_t *msg) {
	BaseType_t ok;
	int err = 0;

	// wait on semaphore
	ok = xSemaphoreTake(msg->completion, portMAX_DELAY);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't take semaphore: %u", ok);
		err = -1;
	}

	// delete semaphore
	vSemaphoreDelete(msg->completion);

	return err;
}

} /* namespace fs */
