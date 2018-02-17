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
#include "OutputTask.h"
#include "Output.h"

#include "OutputBitPatternLUT.h"

#include <Errors.h>

#include <cstring>

#define ADDR_BITBAND(base, bit)				(uint32_t *) (0x22000000 + ((((uint32_t) base) - 0x20000000) * 32) + (bit * 4))

// configuration
#define LEDOUT_TASK_STACK_SZ 				configMINIMAL_STACK_SIZE
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
		trace_puts("Couldn't create LEDOut task!");
	}

	// allocate buffers
	memset(&this->rgbwBuffer, 0, sizeof(this->rgbwBuffer));
	memset(&this->outputBuffer, 0, sizeof(this->outputBuffer));

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

		trace_printf("allocated buffer %d: rgb = 0x%x, size %u, output = 0x%x, size %u\n",
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

	while(1) {
		// convert each RGB buffer
		for(int i = 0; i < numOutputChannels; i++) {
			this->convertBuffer(i);
		}

		// enable output and output each buffer
		o->setOutputEnable(true);

		for(int i = 0; i < numOutputChannels; i++) {
			// TODO: output the second buffer correctly too ;)
			o->outputData(i, this->outputBuffer[0], outputBufSz);
		}


		this->rgbwBuffer[0][0].r += 1;
		this->rgbwBuffer[0][1].g += 2;
		this->rgbwBuffer[0][2].b += 3;
		this->rgbwBuffer[0][3].w += 4;

		// delay lol
		vTaskDelay(4);
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
void OutputTask::convertBuffer(int buffer) {
	uint8_t *buf = (this->outputBuffer[buffer] + 1); // first byte is zero
	uint8_t *read = reinterpret_cast<uint8_t *>(this->rgbwBuffer[buffer]);
//	uint32_t *bitBand = ADDR_BITBAND(buf, 0);

//	trace_printf("BitBand for bit 0 of 0x%x: 0x%x\n", buf, bitBand);

	// read each LED
	for(int i = 0; i < ledsPerChannel; i++) {
		// read each color
		for(int j = 0; j < 4; j++) {
			uint8_t byte = *read++;
			uint32_t patternData = bitPatternLut[byte];
			uint8_t *pattern = reinterpret_cast<uint8_t *>(&patternData);

//			trace_printf("Read byte 0x%02x, pattern 0x%06x\n", byte, patternData);
//			trace_printf("\t %02x %02x %02x\n", pattern[2], pattern[1], pattern[0]);

			// copy the bytes in REVERSE order because ARM is little endian
			*buf++ = pattern[2];
			*buf++ = pattern[1];
			*buf++ = pattern[0];
		}
	}


}

} /* namespace ledout */
