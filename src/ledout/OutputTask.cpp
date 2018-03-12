/*
 * OutputTask.cpp
 *
 * Handles the conversion of RGBW to waveform data and ensuring that data is
 * output at regular intervals.
 *
 * @note This assumes SK6812RGBW-type LEDs are used.
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#define LOG_MODULE "OUT"
#define OUTPUTTASK_PRIVATE

#include "OutputTaskPrivate.h"
#include "OutputTask.h"
#include "Output.h"

#include "OutputBitPatternLUT.h"

#include <LichtensteinApp.h>

#include <cstring>

#define ADDR_BITBAND(base, bit)				(uint32_t *) (0x22000000 + ((((uint32_t) base) - 0x20000000) * 32) + (bit * 4))

// size of the output buffers
static const int numOutputChannels = 2;
static const int ledsPerChannel = 300;
static const int bytesPerPixel = 4;

static const int pixelBufSz = (ledsPerChannel * bytesPerPixel);

// factors for sizing the SPI output DMA buffer
static const int outputBitsPerDataBit = 3;
static const int outputBytesPerPixel = (bytesPerPixel * 8 * outputBitsPerDataBit) / 8;
static const int outputBufSz = (ledsPerChannel * outputBytesPerPixel) + 4;

namespace ledout {

/**
 * Callback for the FPS calculation timer.
 */
void OutputFPSTimerCallback(TimerHandle_t timer) {
	void *ctx = pvTimerGetTimerID(timer);
	OutputTask *task = static_cast<OutputTask *>(ctx);

	// ~ do stuff ~
//	LOG(S_INFO, "FPS: %d %d %d %d", task->fpsCounter[0], task->fpsCounter[1], task->fpsCounter[2], task->fpsCounter[3]);

	// clear the counters back to zero
	for(int i = 0; i < OutputTask::maxOutputBuffers; i++) {
		task->fps[i] = task->fpsCounter[i];
		task->fpsCounter[i] = 0;
	}
}

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _OutputTaskTrampoline(void *ctx) {
	(static_cast<OutputTask *>(ctx))->taskEntry();
}

/**
 * Sets up the LED output task.
 *
 * @note If the task couldn't be created, an exception is thrown.
 */
OutputTask::OutputTask() {
	BaseType_t ok;

	// create message queue
	this->messageQueue = xQueueCreate(OutputTask::MessageQueueDepth,
			sizeof(output_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue");
	}

	// create output task
	ok = xTaskCreate(_OutputTaskTrampoline, "LEDOut", OutputTask::TaskStackSize,
					 this, OutputTask::TaskPriority, &this->handle);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't create task");
	}

	// allocate buffers
	memset(&this->outputBuffer, 0, sizeof(this->outputBuffer));

	for(int i = 0; i < OutputTask::maxOutputBuffers; i++) {
		this->fpsCounter[i] = this->fps[i] = 0;

		// conservative default value
		this->ledsPerBuffer[i] = 150;
	}

	this->allocBuffers();
}

/**
 * If the task is still running, kill it, and free the memory.
 */
OutputTask::~OutputTask() {
	// destroy FPS timer
	xTimerStop(this->fpsTimer, portMAX_DELAY);
	xTimerDelete(this->fpsTimer, portMAX_DELAY);

	// TODO: do this a bit nicer so the task can deallocate its resources
	vTaskDelete(this->handle);

	// free buffers
	for(int i = 0; i < numOutputChannels; i++) {
		vPortFree(this->outputBuffer[i]);
	}
}

/**
 * Allocates the various buffers needed to output pixel data.
 */
void OutputTask::allocBuffers(void) {
	for(int i = 0; i < numOutputChannels; i++) {
		// allocate buffers
		this->outputBuffer[i] = (uint8_t *) pvPortMalloc(outputBufSz);

		LOG(S_DEBUG, "allocated buffer %d: output = 0x%x, size %u",
					 i, this->outputBuffer[i], outputBufSz);

		// clear them
		memset(this->outputBuffer[i], 0, outputBufSz);
	}
}

/**
 * Entry point for the task.
 */
void OutputTask::taskEntry(void) noexcept {
	BaseType_t ok;
	output_message_t msg;

	// create the FPS timer
	this->fpsTimer = xTimerCreate("OutFPS", pdMS_TO_TICKS(1000), pdTRUE, this,
								 OutputFPSTimerCallback);
	xTimerStart(this->fpsTimer, portMAX_DELAY);

	// process messages in here
	while(1) {
		// pull messages off the queue
		ok = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		if(ok != pdPASS) {
			LOG(S_ERROR, "Error reading queue: %u", ok);
			continue;
		}

		// interpret the message type
		switch(msg.type) {
			// convert an output buffer
			case kOutputMessageConvert:
				this->taskConvertBuffer(&msg);
				break;

			// output a buffer
			case kOutputMessageSend:
				this->taskSendBuffer(&msg);
				break;
		}
	}
}

/**
 * Performs the conversion of the pixel data and executes the callback.
 *
 * @param msg Message
 */
void OutputTask::taskConvertBuffer(output_message_t *msg) {
	// perform conversion
	if(msg->payload.convert.isRGBW) {
		this->convertRGBW(msg->channel, msg->payload.convert.numLEDs,
				msg->payload.convert.buffer);
	} else {
		// TODO: RGB conversion
		LOG(S_FATAL, "RGB buffer conversion not implemented yet");
	}

	// run callback if specified
	if(msg->callback != nullptr) {
		msg->callback(msg->cbContext1, msg->cbContext2);
	}
}

/**
 * Sends an already converted buffer to the output.
 *
 * @param msg Received message
 */
void OutputTask::taskSendBuffer(output_message_t *msg) {
	unsigned int i = msg->channel;

	// perform output
	Output::sharedInstance()->outputData(i, this->outputBuffer[i],
			this->outputBufferBytesWritten[i]);

	// increment frame counter
	this->fpsCounter[i]++;

	// run callback if specified
	if(msg->callback != nullptr) {
		msg->callback(msg->cbContext1, msg->cbContext2);
	}
}



/**
 * Sends a message to the output task.
 *
 * @param msg Pointer to the message to post
 * @return 0 if successful, error code otherwise
 */
int OutputTask::sendMessage(output_message_t *msg, int timeout) {
	BaseType_t ok;

	// send message
	ok = xQueueSendToBack(this->messageQueue, msg, timeout);

	// handle errors
	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't send message to task: %u", ok);
		return 1;
	}

	// success if we get down here
	return 0;
}



/**
 * Converts the RGBW buffer into the SPI bitstream that is output. This writes
 * various bit patterns, that when output with SPI, create the correct PWM
 * waveform for the WS2812B strips:
 *
 * - A 0 bit is 0b100
 * - A 1 bit is 0b110
 */
void OutputTask::convertRGBW(int index, int pixels, void *bufferPtr) {
	uint8_t *buf = (this->outputBuffer[index]); // first byte is zero

	// get the read buffer
	rgbw_pixel_t *read = reinterpret_cast<rgbw_pixel_t *>(bufferPtr);

	if(read == nullptr) {
		LOG(S_ERROR, "buffer may not be null");

		return;
	}

	// keep track of how many bytes we write
	uint8_t *bufStart = buf;
	this->outputBufferBytesWritten[index] = 0;

	// first byte is zero
	*buf++ = 0x00;

	// read each LED
	for(int i = 0; i < pixels; i++) {
		// read each color
		for(int j = 0; j < 4; j++) {
			/*
			 * Copy the bytes in kind of a whacky order. The hardware is
			 * designed for the SK6812RGBW LEDs - the datasheets online seem
			 * to be incorrect about the order of the bytes. The plain SK6812
			 * takes data in GRB order (wtf) and it seems like the SK6812RGBW
			 * take it in GRBW order rather than RGBW as the datasheets say,
			 * so we do a bit of swapping here to make it work.
			 */
			uint8_t byte = 0x00;

			if(j == 0) byte = read->g;
			else if(j == 1) byte = read->r;
			else if(j == 2) byte = read->b;
			else if(j == 3) byte = read->w;

			// TODO: cleaner way of doing this?
			uint32_t patternData = bitPatternLut[byte];
			uint8_t *pattern = reinterpret_cast<uint8_t *>(&patternData);

			// copy the bytes in REVERSE order because ARM is little endian
			*buf++ = pattern[2];
			*buf++ = pattern[1];
			*buf++ = pattern[0];
		}

		// go to the next pixel
		read++;
	}

	// write zero byte at the end
	*buf++ = 0x00;

	// get how many bytes we wrote
	this->outputBufferBytesWritten[index] = (buf - bufStart);

}

} /* namespace ledout */
