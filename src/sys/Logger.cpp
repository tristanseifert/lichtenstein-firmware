/*
 * Logger.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */
#define LOG_MODULE "LOG"

#include "Logger.h"
#include "LoggerPrivate.h"

#include "LichtensteinApp.h"

#include "mini-printf/mini-printf.h"
#include "diag/Trace.h"

#include <stdarg.h>
#include <cstring>

// shared instance of the logger
static Logger *gLogger = nullptr;

// static SHARED buffer
static const int gPrintBufferSz = 200;
static char gPrintBuffer[gPrintBufferSz];

// buffer for output to the trace port
static const int gTraceBufferSz = 256;
static char gTraceBuffer[gTraceBufferSz];

// when set, the message handler task is running
static bool taskRunning = false;

SemaphoreHandle_t gBufferMutex = nullptr;


// set to use the UART for debug logging (TRACESWO is used otherwise)
#define USE_UART							1

#if HW == HW_MUSTARD
// baud rate of the logging UART (115200/230400/460800/614400/921600)
#define LOG_UART_BAUD						921600
// UART peripheral to use for logging
#define LOG_UART							USART2
// clock for the UART peripheral
#define LOG_UART_RCC_CLOCK					RCC_APB1Periph_USART2

// do pins need to be remapped?
#define LOG_UART_REMAP						GPIO_Remap_USART2

// clock for the TX port GPIO
#define LOG_UART_TX_RCC_CLOCK				RCC_APB2Periph_GPIOD
// UART_TX port
#define LOG_UART_TX_PORT					GPIOD
// UART_TX pin
#define LOG_UART_TX_PIN						GPIO_Pin_5

// clock for the RX port GPIO
#define LOG_UART_RX_RCC_CLOCK				RCC_APB2Periph_GPIOD
// UART_RX port
#define LOG_UART_RX_PORT						GPIOD
// UART_RX pin
#define LOG_UART_RX_PIN						GPIO_Pin_6

// enable DMA-driven transmission
// TODO: fix DMA. it's broken
#define LOG_UART_USE_DMA_TX					0

// DMA configuration
#define LOG_UART_TX_DMA_RCC_CLOCK			RCC_AHBPeriph_DMA1
#define LOG_UART_TX_DMA_PERIPH				DMA1_Channel7
#define LOG_UART_TX_DMA_CHANNEL				DMA_Stream_7
#define LOG_UART_TX_DMA_IRQ_PRIORITY		(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)
#define LOG_UART_TX_DMA_IRQ					DMA1_Channel7_IRQn
#define LOG_UART_TX_DMA_COMPLETE_FLAG		DMA1_FLAG_TC7
#define LOG_UART_TX_DMA_ISR					DMA1_Channel7_IRQHandler
#define LOG_UART_TX_DMA_PERIPH_BASE			((uint32_t) (&(USART2->DR)))
#endif


/**
 * Entry point for the logger task.
 */
void _LoggerTaskTrampoline(void *ctx) {
	(static_cast<Logger *>(ctx))->taskEntry();
}


/**
 * Returns the shared logger instance.
 */
Logger *Logger::sharedInstance() noexcept {
	// allocate the logger if needed
	taskENTER_CRITICAL();

	if(!gLogger) {
		gLogger = new Logger();
	}

	taskEXIT_CRITICAL();

	// return it
	return gLogger;
}


/**
 * Initializes the logger.
 */
Logger::Logger() {
	// create mutex
	gBufferMutex = xSemaphoreCreateMutex();

#if USE_UART != 1
	// initialize the trace facility
	trace_initialize();

	trace_putchar('\n');
	trace_putchar('\n');
#else
	this->initUART();
#endif

	// create logger task
	BaseType_t ok;

	ok = xTaskCreate(_LoggerTaskTrampoline, "Logger", Logger::TaskStackSize,
			this, Logger::TaskPriority, &this->task);

	if(ok != pdPASS) {
		LOG(S_FATAL, "Couldn't create logger task!");
	}

	// create logger queue
	this->messageQueue = xQueueCreate(Logger::logQueueLength, sizeof(logger_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create logging queue");
	}
}

/**
 * Kills the task and deletes the message queue.
 */
Logger::~Logger() {
	// have the log function go straight to the trace output
	taskRunning = false;

	// delete task and queue
	vTaskDelete(this->task);
	vQueueDelete(this->messageQueue);
}



/**
 * Initializes the UART peripheral.
 */
void Logger::initUART(void) {
	USART_InitTypeDef usart;
	USART_StructInit(&usart);

	// parameters used for all GPIOs
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;

	// enable clock for alternate function IO and remap if needed
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

#ifdef LOG_UART_REMAP
	GPIO_PinRemapConfig(LOG_UART_REMAP, ENABLE);
#endif

	// enable GPIO clocks
	RCC_APB2PeriphClockCmd(LOG_UART_RX_RCC_CLOCK | LOG_UART_TX_RCC_CLOCK, ENABLE);

	// configure GPIOs
	gpio.GPIO_Pin = LOG_UART_RX_PIN;
	GPIO_Init(LOG_UART_RX_PORT, &gpio);

	gpio.GPIO_Pin = LOG_UART_TX_PIN;
	GPIO_Init(LOG_UART_TX_PORT, &gpio);

	// enable UART clock and configure it
	RCC_APB1PeriphClockCmd(LOG_UART_RCC_CLOCK, ENABLE);

	RCC_APB1PeriphResetCmd(LOG_UART_RCC_CLOCK, ENABLE);
	RCC_APB1PeriphResetCmd(LOG_UART_RCC_CLOCK, DISABLE);

	USART_Cmd(LOG_UART, ENABLE);

	usart.USART_BaudRate = LOG_UART_BAUD;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(LOG_UART, &usart);

	// send some newline bytes
	static const char *newlineString = "\r\n\r\n";

	for(size_t i = 0; i <= 4; i++) {
		// wait for tx register to be empty
		while(USART_GetFlagStatus(LOG_UART, USART_FLAG_TXE) != SET) { }
		USART_SendData(LOG_UART, newlineString[i]);
	}

#if LOG_UART_USE_DMA_TX
	// set up the UART DMA
	this->initUARTDMA();
#endif
}

/**
 * Sets up the DMA engine for transmitting packets.
 */
void Logger::initUARTDMA(void) {
	// create the mutex to which DMA is synchronized
	this->uartDMALock = xSemaphoreCreateMutex();
	xSemaphoreGive(this->uartDMALock);

	// enable DMA clocks and set up the NVIC
	RCC_AHBPeriphClockCmd(LOG_UART_TX_DMA_RCC_CLOCK, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;

	nvic.NVIC_IRQChannel = LOG_UART_TX_DMA_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = LOG_UART_TX_DMA_IRQ_PRIORITY;

	NVIC_Init(&nvic);
}

/**
 * Prints the specified buffer, either to the trace output pin, or the UART,
 * depending on the configuration.
 */
void Logger::printMessage(char *buffer, size_t length, bool fromISR) {
#if USE_UART != 1
	// print it to the trace output
	trace_write(buffer, length);
#else
#if LOG_UART_USE_DMA_TX
	// if printing from an ISR, kill the DMA and send it out manually
	if(fromISR) {
		DMA_Cmd(LOG_UART_TX_DMA_PERIPH, DISABLE);
		DMA_DeInit(LOG_UART_TX_DMA_PERIPH);

		// give the lock so later writes succeed
		xSemaphoreGive(this->uartDMALock);

		// send buffer
		for(size_t i = 0; i <= length; i++) {
			// wait for tx register to be empty
			while(USART_GetFlagStatus(LOG_UART, USART_FLAG_TXE) != SET) { }

			// transmit byte
			USART_SendData(LOG_UART, buffer[i]);
		}

		// don't execute any of the code below!
		return;
	}
	// otherwise, wait for the DMA to complete
	else {
		xSemaphoreTake(this->uartDMALock, portMAX_DELAY);

		// wait for DMA to complete
//		while(DMA_GetFlagStatus(LOG_UART_TX_DMA_COMPLETE_FLAG) != SET) {}
	}

	// enter critical section
//	if(!fromISR) taskENTER_CRITICAL();
//	else taskENTER_CRITICAL_FROM_ISR();

	// set up DMA
	DMA_InitTypeDef dma;
	DMA_StructInit(&dma);

	dma.DMA_BufferSize = length;

	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_Mode = DMA_Mode_Normal;

	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_Priority = DMA_Priority_Low; // low priority for log outputs

	dma.DMA_M2M = DMA_M2M_Disable;

	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_MemoryBaseAddr = (uint32_t) buffer;
	dma.DMA_PeripheralBaseAddr = LOG_UART_TX_DMA_PERIPH_BASE;

	DMA_Init(LOG_UART_TX_DMA_PERIPH, &dma);
	DMA_Cmd(LOG_UART_TX_DMA_PERIPH, ENABLE);

	DMA_ITConfig(LOG_UART_TX_DMA_PERIPH, DMA_IT_TC, ENABLE);

	// send the DMA request to the UART
	USART_DMACmd(LOG_UART, USART_DMAReq_Tx, ENABLE);

	// exit critical section
//	if(!fromISR) taskEXIT_CRITICAL();
//	else taskEXIT_CRITICAL_FROM_ISR();
#else
	for(size_t i = 0; i <= length; i++) {
		// wait for tx register to be empty
		while(USART_GetFlagStatus(LOG_UART, USART_FLAG_TXE) != SET) { }

		// transmit byte
		USART_SendData(LOG_UART, buffer[i]);
	}
#endif // #if LOG_UART_USE_DMA_TX
#endif // #ifdef USE_UART
}

/**
 * ISR handler for DMA IRQ
 */
extern "C" void LOG_UART_TX_DMA_ISR(void) {
	Logger *l = Logger::sharedInstance();
	static BaseType_t woke;

	// clear end-of-transfer flag and disable DMA request
	DMA_ClearFlag(LOG_UART_TX_DMA_COMPLETE_FLAG);
	USART_DMACmd(LOG_UART, USART_DMAReq_Tx, DISABLE);

	// signal the end of the transfer
	xSemaphoreGiveFromISR(l->uartDMALock, &woke);
	portYIELD_FROM_ISR(woke);
}



/**
 * Entry point for the logging task.
 */
void Logger::taskEntry(void) {
	int ret;
	logger_message_t msg;
	BaseType_t dequeued;

	// task is running
	taskRunning = true;

	// enter a forever loop
	while(1) {
		// dequeue a message at a time
		dequeued = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		if(!dequeued) {
			LOG(S_ERROR, "Returned from xQueueReceive with no item returned");
			continue;
		}

		// handle the message
		if(this->printToTrace) {
			const char *module = (msg.module == nullptr) ? "???" : msg.module;

			ret = mini_snprintf(gTraceBuffer, gTraceBufferSz,
								"[%u %s %s:%u]: %s\r\n", msg.sev, module,
								msg.file, msg.line, msg.buffer);
			this->printMessage(gTraceBuffer, ret);
		}

		// call any additional handlers

		// de-allocate the buffer
		vPortFree(msg.buffer);
	}
}



/**
 * Performs actual logging. Returns the length of the debug message if sent.
 */
int Logger::log(bool fromISR, logger_severity_t severity, const char *module, const char *file, int line, const char *format, va_list ap) {
	int ret;
	char *bufferCopy = nullptr;

	// lock the buffer
	if(!fromISR) {
		xSemaphoreTake(gBufferMutex, portMAX_DELAY);
	}

	// Print to the local buffer
	ret = mini_vsnprintf(gPrintBuffer, gPrintBufferSz, format, ap);

	if(ret) {
		// create a copy of the buffer
		bufferCopy = (char *) pvPortMalloc((ret + 1));
		memset(bufferCopy, 0, (ret + 1));
		memcpy(bufferCopy, gPrintBuffer, ret);
	} else {
		// if the printf didn't evaluate to anything, exit
		if(!fromISR) {
			xSemaphoreGive(gBufferMutex);
		}

		goto done;
	}

	// if the FreeRTOS scheduler is running, push it to the log task
	if(taskRunning && fromISR == false) {
		BaseType_t queued;

		// release the semaphore to the global buffer
		if(!fromISR) {
			xSemaphoreGive(gBufferMutex);
		}

		// create the message
		logger_message_t msg;

		msg.buffer = bufferCopy;
		msg.file = file;
		msg.line = line;
		msg.module = module;
		msg.sev = severity;

		// queue it, dropping the message if not queued
		queued = xQueueSendToBack(this->messageQueue, &msg, portMAX_DELAY);

		if(queued != pdTRUE) {
			vPortFree(bufferCopy);
		}
	}
	// otherwise, just send it via the trace output
	else {
		if(fromISR) {
			ret = mini_snprintf(gPrintBuffer, gPrintBufferSz, "{%u %s:%u}: %s\r\n", severity, file, line, bufferCopy);
		} else {
			ret = mini_snprintf(gPrintBuffer, gPrintBufferSz, "(%u %s:%u): %s\r\n", severity, file, line, bufferCopy);
		}

		this->printMessage(gPrintBuffer, ret, true);
		vPortFree(bufferCopy);

		// release the semaphore to the global buffer
		if(!fromISR) {
			xSemaphoreGive(gBufferMutex);
		}
	}

	done: ;
	return ret;
}

extern "C" int _LoggerDoLog(bool fromISR, logger_severity_t severity, const char *module, const char *file, int line, const char *format, ...) {
	int ret;

	// set up the variadic arguments and call into the logger
	va_list ap;
	va_start(ap, format);

	ret = Logger::sharedInstance()->log(fromISR, severity, module, file, line, format, ap);

	va_end(ap);
	return ret;
}
