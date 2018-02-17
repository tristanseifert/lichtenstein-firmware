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
#define LED_OE_PORT							GPIOB
#define LED_OE_PIN							GPIO_Pin_14
#define LED_OE_GPIO_CLOCK					RCC_APB2Periph_GPIOB

#define LED_OE_ACTIVE_LOW					1


// LED output 0 (SPI2_MOSI)
#define LED_OUT0_PORT						GPIOB
#define LED_OUT0_PIN							GPIO_Pin_15
#define LED_OUT0_GPIO_CLOCK					RCC_APB2Periph_GPIOB

// baud rate divisor (SPI[2,3] have 36MHz clock)
#define LED_OUT0_SPI_RCC_CLOCK				RCC_APB1Periph_SPI2

#define LED_OUT0_SPI_BAUD					SPI_BaudRatePrescaler_16
#define LED_OUT0_SPI							SPI2
#define LED_OUT0_NEEDS_REMAP					0
#define LED_OUT0_REMAP						GPIO_Remap_SPI2

// DMA for led output 0
#define LED_OUT0_DMA_RCC_CLOCK				RCC_AHBPeriph_DMA1
#define LED_OUT0_DMA_PERIPH					DMA1_Channel5
#define LED_OUT0_DMA_CHANNEL					DMA_Stream_5
#define LED_OUT0_DMA_IRQ_PRIORITY			(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)
#define LED_OUT0_DMA_IRQ						DMA1_Channel5_IRQn
#define LED_OUT0_DMA_COMPLETE_FLAG			DMA1_FLAG_TC5
#define LED_OUT0_DMA_ISR						DMA1_Channel5_IRQHandler
#define LED_OUT0_DMA_PERIPH_BASE				((uint32_t) (&(SPI2->DR)))


// LED output 1 (SPI3_MOSI AF)
#define LED_OUT1_PORT						GPIOC
#define LED_OUT1_PIN							GPIO_Pin_12
#define LED_OUT1_GPIO_CLOCK					RCC_APB2Periph_GPIOC

// baud rate divisor (SPI[2,3] have 36MHz clock)
#define LED_OUT1_SPI_RCC_CLOCK				RCC_APB1Periph_SPI3

#define LED_OUT1_SPI_BAUD					SPI_BaudRatePrescaler_16
#define LED_OUT1_SPI							SPI3
#define LED_OUT1_NEEDS_REMAP					1
#define LED_OUT1_REMAP						GPIO_Remap_SPI3

// DMA for led output 1
#define LED_OUT1_DMA_RCC_CLOCK				RCC_AHBPeriph_DMA2
#define LED_OUT1_DMA_PERIPH					DMA2_Channel2
#define LED_OUT1_DMA_CHANNEL					DMA_Stream_2
#define LED_OUT1_DMA_IRQ_PRIORITY			(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)
#define LED_OUT1_DMA_IRQ						DMA2_Channel2_IRQn
#define LED_OUT1_DMA_COMPLETE_FLAG			DMA2_FLAG_TC2
#define LED_OUT1_DMA_ISR						DMA2_Channel2_IRQHandler
#define LED_OUT1_DMA_PERIPH_BASE				((uint32_t) (&(SPI3->DR)))

// when set, use DMA to transfer pixel data to LEDs.
#define USE_DMA								1

#endif

static Output *gOutput = nullptr;

using namespace ledout;

/**
 * Allocates the shared output handler.
 */
void Output::init(void) {
	taskENTER_CRITICAL();

	if(!gOutput) {
		gOutput = new Output();
	}

	taskEXIT_CRITICAL();
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

	// set up the DMA
	this->initOutputDMA();

	// once hardware setup is complete, start the task
	this->task = new OutputTask();
}

/**
 * Deletes the task, which will stop it.
 */
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

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = LED_OUT0_PIN;
	GPIO_Init(LED_OUT0_PORT, &gpio);

#if LED_OUT0_NEEDS_REMAP
	GPIO_PinRemapConfig(LED_OUT0_REMAP, ENABLE);
#endif
	// set up LED1 out
	RCC_APB2PeriphClockCmd(LED_OUT1_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = LED_OUT1_PIN;
	GPIO_Init(LED_OUT1_PORT, &gpio);

#if LED_OUT1_NEEDS_REMAP
	GPIO_PinRemapConfig(LED_OUT1_REMAP, ENABLE);
#endif

	// enable clocks for SPIs
	RCC_APB1PeriphClockCmd(LED_OUT0_SPI_RCC_CLOCK, ENABLE);
	RCC_APB1PeriphClockCmd(LED_OUT1_SPI_RCC_CLOCK, ENABLE);

	// prepare shared SPI settings
	SPI_InitTypeDef spi;
	SPI_StructInit(&spi);

	spi.SPI_Mode = SPI_Mode_Master;
//	spi.SPI_Direction = SPI_Direction_1Line_Tx;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_Low;
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
		xSemaphoreGive(handle);
	}

	// enable clocks for the DMA
	RCC_AHBPeriphClockCmd(LED_OUT0_DMA_RCC_CLOCK, ENABLE);
	RCC_AHBPeriphClockCmd(LED_OUT1_DMA_RCC_CLOCK, ENABLE);


	// enable the DMA interrupt to pass through the NVIC
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;

	// enable DMA interrupt for LED output channel 0
	nvic.NVIC_IRQChannel = LED_OUT0_DMA_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = LED_OUT0_DMA_IRQ_PRIORITY;

	NVIC_Init(&nvic);

	// enable DMA interrupt for LED output channel 1
	nvic.NVIC_IRQChannel = LED_OUT1_DMA_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = LED_OUT1_DMA_IRQ_PRIORITY;

	NVIC_Init(&nvic);
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
	SPI_TypeDef *spi;

	// sanity check the channel number
	if(channel < 0 || channel > Output::NumOutputChannels) {
		return -1;
	}

	// ensure buffer is non-null and length is nonzero
	if(data == nullptr || length == 0) {
		return -1;
	}

	// get the SPI peripheral
	if(channel == 0) {
		spi = LED_OUT0_SPI;
	} else if(channel == 1) {
		spi = LED_OUT1_SPI;
	}

#if USE_DMA
	// typedefs for DMA
	DMA_InitTypeDef dma;
	DMA_StructInit(&dma);

	DMA_Channel_TypeDef *dmaChannel;

	// if a DMA is in progress, wait for it to complete
//	trace_printf("channel %u, output %u bytes from 0x%x; waiting for lock\n", channel, length, data);
	xSemaphoreTake(this->dmaSemaphores[channel], portMAX_DELAY);
//	trace_printf("got lock!\n");

	// now that we have the semaphore, enter a critical section
	taskENTER_CRITICAL();

	// set up the channel and peripheral base
	if(channel == 0) {
		dmaChannel = LED_OUT0_DMA_PERIPH;
		spi = LED_OUT0_SPI;

		dma.DMA_PeripheralBaseAddr = LED_OUT0_DMA_PERIPH_BASE;
	} else if(channel == 1) {
		dmaChannel = LED_OUT1_DMA_PERIPH;
		spi = LED_OUT1_SPI;

		dma.DMA_PeripheralBaseAddr = LED_OUT1_DMA_PERIPH_BASE;
	}

	// clear/reset the DMA
	DMA_DeInit(dmaChannel);

	// set the shared parameters for DMA
	dma.DMA_BufferSize = length;

	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_Mode = DMA_Mode_Normal;

	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_Priority = DMA_Priority_High;

	dma.DMA_M2M = DMA_M2M_Disable;

	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_MemoryBaseAddr = (uint32_t) data;

	// initialize the DMA
	DMA_Init(dmaChannel, &dma);
	DMA_Cmd(dmaChannel, ENABLE);

	DMA_ITConfig(dmaChannel, DMA_IT_TC, ENABLE);

	// send the DMA request to the SPI
	SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Tx, ENABLE);

	// exit critical section
	taskEXIT_CRITICAL();
#else
	// enter critical section
	taskENTER_CRITICAL();

	uint8_t *dataPtr = static_cast<uint8_t *>(data);

	for(unsigned int i = 0; i < length; i++) {
		// wait for the SPI to be ready to transmit
	    while(SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET) {

	    }

	    // send data
	    SPI_I2S_SendData(spi, *dataPtr++);
	}

	// exit critical section
	taskEXIT_CRITICAL();
#endif

	// we're done!
	return 0;
}

/**
 * DMA completion handler for the first channel. This signals the correct
 * semaphore and performs a context switch if required.
 */
extern "C" void LED_OUT0_DMA_ISR(void) {
	Output *o = Output::sharedInstance();
	static BaseType_t woke;

	// clear end-of-transfer flag and disable DMA request
	DMA_ClearFlag(LED_OUT0_DMA_COMPLETE_FLAG);
	SPI_I2S_DMACmd(LED_OUT0_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

	// signal the end of the transfer
	xSemaphoreGiveFromISR(o->dmaSemaphores[0], &woke);
	portYIELD_FROM_ISR(woke);
}

/**
 * DMA completion handler for the second channel. This signals the correct
 * semaphore and performs a context switch if required.
 */
extern "C" void LED_OUT1_DMA_ISR(void) {
	Output *o = Output::sharedInstance();
	static BaseType_t woke;

	// clear end-of-transfer flag and disable DMA request
	DMA_ClearFlag(LED_OUT1_DMA_COMPLETE_FLAG);
	SPI_I2S_DMACmd(LED_OUT1_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

	// signal the end of the transfer
	xSemaphoreGiveFromISR(o->dmaSemaphores[1], &woke);
	portYIELD_FROM_ISR(woke);
}
