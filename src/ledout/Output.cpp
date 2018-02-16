/*
 * Output.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "Output.h"

#include "cmsis_device.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"

#if HW == HW_MUSTARD
// level shifter output enable (active low)
#define LED_OE_PORT					GPIOB
#define LED_OE_PIN					GPIO_Pin_14
#define LED_OE_GPIO_CLOCK			RCC_APB2Periph_GPIOB

#define LED_OE_ACTIVE_LOW			1

// LED output 0 (SPI2_MOSI)
#define LED_OUT0_PORT				GPIOB
#define LED_OUT0_PIN					GPIO_Pin_15
#define LED_OUT0_GPIO_CLOCK			RCC_APB2Periph_GPIOB

// baud rate divisor (SPI[2,3] have 36MHz clock)
#define LED_OUT0_SPI_BAUD			SPI_BaudRatePrescaler_16
#define LED_OUT0_SPI					SPI2
#define LED_OUT0_NEEDS_REMAP			0
#define LED_OUT0_REMAP				GPIO_Remap_SPI2

// LED output 1 (SPI3_MOSI AF)
#define LED_OUT1_PORT				GPIOC
#define LED_OUT1_PIN					GPIO_Pin_12
#define LED_OUT1_GPIO_CLOCK			RCC_APB2Periph_GPIOC

// baud rate divisor (SPI[2,3] have 36MHz clock)
#define LED_OUT1_SPI_BAUD			SPI_BaudRatePrescaler_16
#define LED_OUT1_SPI					SPI3
#define LED_OUT1_NEEDS_REMAP			1
#define LED_OUT1_REMAP				GPIO_Remap_SPI3

#endif

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
	// set up hardware
	this->initOutputGPIOs();

	// once hardware setup is complete, start the task
	this->task = new OutputTask();
}

Output::~Output() {
	delete this->task;
}

/**
 * Initializes the GPIOs used for output.
 */
void Output::initOutputGPIOs(void) {
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	// enable clock for alternate function IO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// set up OE (and drive it high to disable output)
	RCC_APB2PeriphClockCmd(LED_OE_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = LED_OE_PIN;
	GPIO_Init(LED_OE_PORT, &gpio);

	GPIO_WriteBit(LED_OE_PORT, LED_OE_PIN, LED_OE_ACTIVE_LOW ? Bit_SET : Bit_RESET);


	// set up LED0 out
	RCC_APB2PeriphClockCmd(LED_OUT0_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = LED_OUT0_PIN;
	GPIO_Init(LED_OUT0_PORT, &gpio);

#if LED_OUT0_NEEDS_REMAP
	GPIO_PinRemapConfig(LED_OUT0_REMAP, ENABLE);
#endif
	// set up LED1 out
	RCC_APB2PeriphClockCmd(LED_OUT1_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = LED_OUT1_PIN;
	GPIO_Init(LED_OUT1_PORT, &gpio);

#if LED_OUT1_NEEDS_REMAP
	GPIO_PinRemapConfig(LED_OUT1_REMAP, ENABLE);
#endif

	// prepare shared SPI settings
	SPI_InitTypeDef spi;
	SPI_StructInit(&spi);

	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_Direction = SPI_Direction_1Line_Tx;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_High;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;


	// set up SPI for LED0 out
	spi.SPI_BaudRatePrescaler = LED_OUT0_SPI_BAUD;

	SPI_Init(LED_OUT0_SPI, &spi);
	SPI_Cmd(LED_OUT0_SPI, ENABLE);

	// set up SPI for LED1 out
	spi.SPI_BaudRatePrescaler = LED_OUT1_SPI_BAUD;

	SPI_Init(LED_OUT1_SPI, &spi);
	SPI_Cmd(LED_OUT1_SPI, ENABLE);
}

/**
 * Initializes the DMA engine for the output SPI drivers.
 */
void Output::initOutputDMA(void) {
	SemaphoreHandle_t handle;

	// allocate semaphores
	for(int i = 0; i < Output::NumOutputChannels; i++) {
		handle = xSemaphoreCreateBinary();
		this->dmaSemaphores[i] = handle;

		// give the semaphore so the DMA routine can take it
//		xSemaphoreGive(handle);
	}
}

/**
 * Sets the status of the output enable signal. IF enabled, the current output
 * from the MCU is visible on the LED output terminals.
 */
void Output::setOutputEnable(bool enable) {
	BitAction state;

	if(enable) {
		state = LED_OE_ACTIVE_LOW ? Bit_RESET : Bit_SET;
	} else {
		state = LED_OE_ACTIVE_LOW ? Bit_SET : Bit_RESET;
	}

	GPIO_WriteBit(LED_OE_PORT, LED_OE_PIN, state);
}

/**
 * Uses DMA to transmit the given data buffer to the specified channel.
 *
 * @return 0 if the DMA was successfully queued, -1 otherwise.
 */
int Output::outputData(int channel, void *data, size_t length) {
	// sanity check the channel number
	if(channel < 0 || channel <= Output::NumOutputChannels) {
		return -1;
	}

	// ensure buffer is non-null and length is nonzero
	if(data == nullptr || length == 0) {
		return -1;
	}

	// if a DMA is in progress, wait for it to complete
	xSemaphoreTake(this->dmaSemaphores[channel], portMAX_DELAY);

	// start a new DMA

	// we're done!
	return 0;
}
