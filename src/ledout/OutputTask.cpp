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

#include "OutputTask.h"
#include "Output.h"

#include "OutputBitPatternLUT.h"

#include "LichtensteinApp.h"

#include <cstring>

#define ADDR_BITBAND(base, bit)				(uint32_t *) (0x22000000 + ((((uint32_t) base) - 0x20000000) * 32) + (bit * 4))

// configuration
#define LEDOUT_TASK_STACK_SZ 				150
#define LEDOUT_TASK_PRIORITY					(3)

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
	LOG(S_INFO, "FPS: %d %d %d %d", task->fpsCounter[0], task->fpsCounter[1], task->fpsCounter[2], task->fpsCounter[3]);

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

	ok = xTaskCreate(_OutputTaskTrampoline, "LEDOut", LEDOUT_TASK_STACK_SZ,
					 this, LEDOUT_TASK_PRIORITY, &this->handle);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't create LEDOut task!");
	}

	// allocate buffers
	memset(&this->rgbwBuffer, 0, sizeof(this->rgbwBuffer));
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
	// TODO: do this a bit nicer so the task can deallocate its resources
	vTaskDelete(this->handle);

	// free buffers
	for(int i = 0; i < numOutputChannels; i++) {
		vPortFree(this->rgbwBuffer[i]);
		vPortFree(this->outputBuffer[i]);
	}
}

/**
 * Allocates the various buffers needed to output pixel data.
 */
void OutputTask::allocBuffers(void) {
	for(int i = 0; i < numOutputChannels; i++) {
		// allocate buffers
		this->rgbwBuffer[i] = (rgbw_pixel_t *) pvPortMalloc(pixelBufSz);
		this->outputBuffer[i] = (uint8_t *) pvPortMalloc(outputBufSz);

		LOG(S_DEBUG, "allocated buffer %d: rgb = 0x%x, size %u, output = 0x%x, size %u",
					 i, this->rgbwBuffer[i], (ledsPerChannel * bytesPerPixel),
					 this->outputBuffer[i], outputBufSz);

		// clear them
		memset(this->rgbwBuffer[i], 0, pixelBufSz);
		memset(this->outputBuffer[i], 0, outputBufSz);

		this->rgbwBuffer[0][0].r = 0x80;
		this->rgbwBuffer[0][1].g = 0x80;
		this->rgbwBuffer[0][2].b = 0x80;
		this->rgbwBuffer[0][3].w = 0x80;

		this->rgbwBuffer[0][6].r = 0x80;
		this->rgbwBuffer[0][6].g = 0x80;
	}
}

/**
 * Entry point for the task.
 */
void OutputTask::taskEntry(void) noexcept {
	Output *o = Output::sharedInstance();

	// create the FPS timer
	this->fpsTimer = xTimerCreate("OutFPS", pdMS_TO_TICKS(1000), pdTRUE, this,
								 OutputFPSTimerCallback);
	xTimerStart(this->fpsTimer, portMAX_DELAY);

	// enable output
	o->setOutputEnable(true);

	// keep pulling new RGBW buffers off the queue and process them
	while(1) {
		// convert each RGB buffer
		for(int i = 0; i < numOutputChannels; i++) {
			this->convertBuffer(i, this->ledsPerBuffer[i]);
		}

		// enable output and output each buffer
		for(int i = 0; i < numOutputChannels; i++) {
			// TODO: output the second buffer correctly too ;)
			o->outputData(i, this->outputBuffer[0], this->outputBufferBytesWritten[0]);

			this->fpsCounter[i]++;
		}

		this->rgbwBuffer[0][0].r += 1;
		this->rgbwBuffer[0][1].g += 2;
		this->rgbwBuffer[0][2].b += 3;
		this->rgbwBuffer[0][3].w += 4;

		// delay lol
		vTaskDelay(2);
	}
}

/**
 * Converts the RGBW buffer into the SPI bitstream that is output. This writes
 * various bit patterns, that when output with SPI, create the correct PWM
 * waveform for the WS2812B strips:
 *
 * - A 0 bit is 0b100
 * - A 1 bit is 0b110
 */
void OutputTask::convertBuffer(int index, int pixels, void *bufferPtr) {
	uint8_t *buf = (this->outputBuffer[index]); // first byte is zero

	// get the read buffer
	uint8_t *read;
	if(bufferPtr) {
		read = reinterpret_cast<uint8_t *>(bufferPtr);
	} else {
		read = reinterpret_cast<uint8_t *>(this->rgbwBuffer[index]);
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
			uint8_t byte = *read++;
			uint32_t patternData = bitPatternLut[byte];
			uint8_t *pattern = reinterpret_cast<uint8_t *>(&patternData);

//			LOG(S_DEBUG, "Read byte 0x%02x, pattern 0x%06x", byte, patternData);
//			LOG(S_DEBUG, "\t %02x %02x %02x", pattern[2], pattern[1], pattern[0]);

			// copy the bytes in REVERSE order because ARM is little endian
			*buf++ = pattern[2];
			*buf++ = pattern[1];
			*buf++ = pattern[0];
		}
	}

	// write zero byte at the end
	*buf++ = 0x00;

	// get how many bytes we wrote
	this->outputBufferBytesWritten[index] = (buf - bufStart);

}

} /* namespace ledout */
