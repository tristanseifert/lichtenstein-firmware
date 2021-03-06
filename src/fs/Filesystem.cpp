/*
 * Filesystem.cpp
 *
 * Provides access to filesystem on an SPI flash. This filesystem is used for
 * configuration storage, logging, and a by the bootloader to download new
 * firmware images.
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */
#define FILE_PRIVATE

#define LOG_MODULE "FS"

#include "Filesystem.h"
#include "FSPrivate.h"

#include "File.h"

#include <LichtensteinApp.h>

#include "FlashCommands.h"

#include "flash/SST25VF016.h"

#include <cstring>

static Filesystem *gFilesystem = nullptr;

/**
 * List of supported flash chips. This is used to identify the size of the
 * chip and other features it may or may not support when the filesystem is
 * first initialized.
 */
static const size_t numSupportedChips = 1;

static const flash_info_t supportedChips[numSupportedChips] = {
	// SST25VF016B, as on the el cheapo Chinese dev board
	{
		.jdecId = 0xbf2541,
		.type = kFlashTypeSST25VF016,
		.size = (1024 * 1024 * 2),
		.name = "SST25VF016B"
	}
};

/**
 * Any read or write operations that are larger than this threshold will use
 * DMA. This should be set such that the overhead of configuring and enabling
 * the DMA engine and request is lower than that of a write loop that does
 * busy waiting.
 */
#define FLASH_DMA_THRESHOLD					(1024*1024)

// filesystem configuration
#define FS_SIZE								(1024 * 1024 * 2)

#define FS_PAGE_SIZE							256
#define FS_FLASH_BLOCK_SIZE					(1024 * 4)

/// enable caching in SPIFFS when set
#define USE_CACHE							SPIFFS_CACHE

/// size of the file descriptor buffer
#define FS_FDBUF_SIZE						(32 * 16)
#define FS_CACHE_BUF_SIZE					((FS_PAGE_SIZE + 32) * 4)

/// produce log output for reads from flash
#define LOG_FLASH_READS						0
/// produce log output for writes to flash
#define LOG_FLASH_WRITES						0
/// produce log output for erasing flash
#define LOG_FLASH_ERASE						0

/// formats the filesystem on mount
#define FORMAT_ON_MOUNT						0


#if HW == HW_MUSTARD

// define the SPI peripheral the flash is connected to
#define FLASH_SPI_PERIPH						SPI1
#define FLASH_SPI_RCC_CLOCK					RCC_APB2Periph_SPI1

#define FLASH_SPI_BAUD						SPI_BaudRatePrescaler_32 // this could be 1
#define FLASH_SPI_NEEDS_REMAP				0
#define FLASH_SPI_REMAP						GPIO_Remap_SPI1

// /CS line for the flash
#define FLASH_CS_PORT						GPIOC
#define FLASH_CS_PIN							GPIO_Pin_5
#define FLASH_CS_RCC_CLOCK					RCC_APB2Periph_GPIOC

// flash SPI SCK
#define FLASH_SCK_PORT						GPIOA
#define FLASH_SCK_PIN						GPIO_Pin_5
#define FLASH_SCK_RCC_CLOCK					RCC_APB2Periph_GPIOA

// flash SPI MISO
#define FLASH_MISO_PORT						GPIOA
#define FLASH_MISO_PIN						GPIO_Pin_6
#define FLASH_MISO_RCC_CLOCK					RCC_APB2Periph_GPIOA

// flash SPI MOSI
#define FLASH_MOSI_PORT						GPIOA
#define FLASH_MOSI_PIN						GPIO_Pin_7
#define FLASH_MOSI_RCC_CLOCK					RCC_APB2Periph_GPIOA


// DMA to use for SPI reads
#define FLASH_RX_DMA							DMA1_Channel2
#define FLASH_RX_DMA_CHANNEL					DMA_Stream_2

#define FLASH_RX_DMA_IRQ_PRIORITY			(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)
#define FLASH_RX_DMA_IRQ						DMA1_Channel2_IRQn
#define FLASH_RX_DMA_COMPLETE_FLAG			DMA1_FLAG_TC2
#define FLASH_RX_DMA_ISR						DMA1_Channel2_IRQHandler

#define FLASH_RX_DMA_RCC_CLOCK				RCC_AHBPeriph_DMA1

// DMA to use for SPI writes
#define FLASH_TX_DMA							DMA1_Channel3
#define FLASH_TX_DMA_CHANNEL					DMA_Stream_3

#define FLASH_TX_DMA_IRQ_PRIORITY			(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)
#define FLASH_TX_DMA_IRQ						DMA1_Channel3_IRQn
#define FLASH_TX_DMA_COMPLETE_FLAG			DMA1_FLAG_TC3
#define FLASH_TX_DMA_ISR						DMA1_Channel3_IRQHandler

#define FLASH_TX_DMA_RCC_CLOCK				RCC_AHBPeriph_DMA1

#endif

// define prototypes for the C helper functions for spiffs
s32_t _spiffs_read(u32_t addr, u32_t size, u8_t *dst);
s32_t _spiffs_write(u32_t addr, u32_t size, u8_t *src);
s32_t _spiffs_erase(u32_t addr, u32_t size);



/**
 * C trampoline to go into the FreeRTOS task.
 */
void _FSTaskTrampoline(void *ctx) {
	(static_cast<Filesystem *>(ctx))->taskEntry();
}

/**
 * Timer callback for SPIFFS checking.
 */
void _FSCheckResetTimerCallback(void *ctx) {
	LOG_ISR(S_WARN, "Resetting system after SPIFFS check…");
	NVIC_SystemReset();
}


/**
 * Allocates the shared filesystem instance.
 */
void Filesystem::init(void) {
	taskENTER_CRITICAL();

	if(!gFilesystem) {
		gFilesystem = new Filesystem();
	}

	taskEXIT_CRITICAL();
}
/**
 * Returns the shared filesystem instance.
 */
Filesystem *Filesystem::sharedInstance() noexcept {
	return gFilesystem;
}



/**
 * Sets up the SPI flash and filesystem.
 */
Filesystem::Filesystem() {
	// initialize the hardware
	this->setUpGPIOs();
	this->setUpSPI();
	this->setUpIRQs();
	this->setUpDMA();

	// create the filesystem task
	this->setUpTask();
}

/**
 * Sets up the GPIOs for the SPI bus.
 */
void Filesystem::setUpGPIOs(void) {
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	// enable clock for alternate function IO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// enable remap if required
#if FLASH_SPI_NEEDS_REMAP
	GPIO_PinRemapConfig(FLASH_SPI_REMAP, ENABLE);
#endif

	// set up /CS as an output and drive it high
	RCC_APB2PeriphClockCmd(FLASH_CS_RCC_CLOCK, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = FLASH_CS_PIN;
	GPIO_Init(FLASH_CS_PORT, &gpio);

	this->setFlashCS(false);

	// set up SCK
	RCC_APB2PeriphClockCmd(FLASH_SCK_RCC_CLOCK, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = FLASH_SCK_PIN;
	GPIO_Init(FLASH_SCK_PORT, &gpio);

	// set up MISO
	RCC_APB2PeriphClockCmd(FLASH_MISO_RCC_CLOCK, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = FLASH_MISO_PIN;
	GPIO_Init(FLASH_MISO_PORT, &gpio);

	// set up MOSI
	RCC_APB2PeriphClockCmd(FLASH_MOSI_RCC_CLOCK, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = FLASH_MOSI_PIN;
	GPIO_Init(FLASH_MOSI_PORT, &gpio);
}

/**
 * Configures the SPI peripheral appropriately.
 */
void Filesystem::setUpSPI(void) {
	SPI_InitTypeDef spi;
	SPI_StructInit(&spi);

	// enable clock for the SPI
	RCC_APB2PeriphClockCmd(FLASH_SPI_RCC_CLOCK, ENABLE);

	// configure the SPI peripheral
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;

	spi.SPI_BaudRatePrescaler = FLASH_SPI_BAUD;

	SPI_Init(FLASH_SPI_PERIPH, &spi);
	SPI_Cmd(FLASH_SPI_PERIPH, ENABLE);
}

/**
 * Sets up the various interrupts with the NVIC.
 */
void Filesystem::setUpIRQs(void) {

}

/**
 * Configures the DMA engine.
 */
void Filesystem::setUpDMA(void) {
	// set up clocks
	RCC_AHBPeriphClockCmd(FLASH_RX_DMA_RCC_CLOCK, ENABLE);
	RCC_AHBPeriphClockCmd(FLASH_TX_DMA_RCC_CLOCK, ENABLE);
}

/**
 * Shuts down the filesystem cleanly.
 */
Filesystem::~Filesystem() {
	// delete the HAL
	if(this->flashHAL) {
		delete this->flashHAL;
	}

	// delete task
	if(this->task) {
		vTaskDelete(this->task);
	}

	// mark completion of any outstanding ops and free queue
	fs_message_t msg;
	BaseType_t ok;

	do {
		// read message without waiting and give semaphore
		ok = xQueueReceive(this->messageQueue, &msg, 0);

		xSemaphoreGive(msg.completion);
	} while(ok == pdPASS);

	if(this->messageQueue) {
		vQueueDelete(this->messageQueue);
	}

	// unmount filesystem
	SPIFFS_unmount(&this->fs);

	// de-allocate buffers
	if(this->fsWorkBuf) {
		vPortFree(this->fsWorkBuf);
	}

	if(this->fsFileDescriptors) {
		vPortFree(this->fsFileDescriptors);
	}

#if USE_CACHE
	if(this->fsCache) {
		vPortFree(this->fsCache);
	}
#endif
}



/**
 * Initializes the task and its associated structures.
 */
void Filesystem::setUpTask(void) {
	BaseType_t ok;

	// set up the message queue
	this->messageQueue = xQueueCreate(Filesystem::MessageQueueDepth,
			sizeof(fs_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue");
	}

	// create the task
	ok = xTaskCreate(_FSTaskTrampoline, "FS", Filesystem::TaskStackSize,
					 this, Filesystem::TaskPriority, &this->task);

	if(ok != pdPASS) {
		LOG(S_FATAL, "Couldn't create task");
	}

	// create the mutex protecting the flash
	this->flashMutex = xSemaphoreCreateMutex();
}

/**
 * Entry point for the filesystem task. This waits on requests on its message
 * buffer and acts on them.
 *
 * This also initializes the filesystem on startup, and shuts it down cleanly
 * when requested.
 */
void Filesystem::taskEntry(void) {
	int err = 0;

	BaseType_t ok;
	fs_message_t msg;

	// read the flash memory's ID and sizing
	this->identifyFlash();

	// initialize the filesystem here
	err = this->spiffsMount();

	if(err != 0) {
		LOG(S_FATAL, "Couldn't initialize SPIFFS: %d", err);

		while(1) {
			// should never get here
			vTaskDelay(1000);
		}
		return;
	}

	// message loop
	while(1) {
		// read from the message queue
		ok = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		if(ok != pdPASS) {
			LOG(S_ERROR, "Error reading queue: %u", ok);
			continue;
		}

		// process message here
		switch(msg.type) {
			// open a file
			case kFSMessageOpenFile: {
				spiffs_file file;

				// attempt to open the file
				file = SPIFFS_open(&this->fs,
						msg.payload.open.filename,
						msg.payload.open.flags, 0);

				// handle errors
				if(file < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);

					/*LOG(S_DEBUG, "Opening file \"%s\", flags 0x%04x: err %d",
							msg.payload.open.filename, msg.payload.open.flags,
							*msg.returnValuePtr);*/
				}
				// otherwise, write the file handle
				else {
					*msg.successPtr = true;
					*msg.descriptor = file;

					/*LOG(S_DEBUG, "Opening file \"%s\", flags 0x%04x: success",
							msg.payload.open.filename, msg.payload.open.flags);*/
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}
			// close a file
			case kFSMessageCloseFile: {
				err = SPIFFS_close(&this->fs, *msg.descriptor);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					*msg.successPtr = true;
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}

			// read from a file
			case kFSMessageRead: {
				// attempt read
				err = SPIFFS_read(&this->fs, *msg.descriptor,
						msg.payload.read.buffer,
						msg.payload.read.length);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					// return value is bytes read
					*msg.returnValuePtr = err;
					*msg.successPtr = true;
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}
			// write to a file
			case kFSMessageWrite: {
				// attempt write
				err = SPIFFS_write(&this->fs, *msg.descriptor,
						msg.payload.write.buffer,
						msg.payload.write.length);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					// return value is bytes written
					*msg.returnValuePtr = err;
					*msg.successPtr = true;
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}

			// seek and tell
			case kFSMessageSeek:
			case kFSMessageTell: {
				// attempt the seek
				err = SPIFFS_lseek(&this->fs, *msg.descriptor,
						msg.payload.seek.offset,
						msg.payload.seek.mode);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					// return value is the new position in the file
					*msg.returnValuePtr = err;
					*msg.successPtr = true;
				}

				/*LOG(S_DEBUG, "Seek to %d (mode %u): %d",
						msg.payload.seek.offset, msg.payload.seek.mode,
						*msg.returnValuePtr);*/

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}

			// flush all pending writes
			case kFSMessageFlush: {
				// flush
				err = SPIFFS_fflush(&this->fs, *msg.descriptor);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					*msg.successPtr = true;
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}

			// remove a file
			case kFSMessageRemove: {
				// attempt to delete the file
				err = SPIFFS_remove(&this->fs, msg.payload.remove.filename);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					*msg.successPtr = true;
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}
			// rename a file
			case kFSMessageRename: {
				// attempt to rename the file
				err = SPIFFS_rename(&this->fs, msg.payload.rename.oldName,
						msg.payload.rename.newName);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					*msg.successPtr = true;
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}

			// get info about the file
			case kFSMessageStat: {
				spiffs_stat stat;

				// call stat function
				err = SPIFFS_stat(&this->fs, msg.payload.stat.filename, &stat);

				// handle errors
				if(err < 0) {
					*msg.returnValuePtr = SPIFFS_errno(&this->fs);
				} else {
					*msg.successPtr = true;

					// extract info from the stat structure here
					*msg.payload.stat.sizePtr = stat.size;
				}

				// notify task
				xSemaphoreGive(msg.completion);
				break;
			}

			// unhandled message types
			default:
				LOG(S_WARN, "Unhandled message type: %u", msg.type);
				break;
		}
	}
}

/**
 * Attempts to mount the filesystem.
 */
int Filesystem::spiffsMount(bool triedFormat) {
	int ret;

	// allocate some buffers
	if(!this->fsWorkBuf) {
		this->fsWorkBuf = (uint8_t *) pvPortMalloc(FS_PAGE_SIZE * 2);
	}

	if(!this->fsFileDescriptors) {
		this->fsFileDescriptors = (uint8_t *) pvPortMalloc(FS_FDBUF_SIZE);
	}

#if USE_CACHE
	if(!this->fsCache) {
		this->fsCache = (uint8_t *) pvPortMalloc(FS_CACHE_BUF_SIZE);
	}
#endif

	// set up the spiffs config
	spiffs_config cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.phys_size = this->flashSize; // use all spi flash
	cfg.phys_addr = 0x000000; // start spiffs at start of spi flash
	cfg.phys_erase_block = FS_FLASH_BLOCK_SIZE; // according to datasheet
	cfg.log_block_size = FS_FLASH_BLOCK_SIZE; // let us not complicate things
	cfg.log_page_size = FS_PAGE_SIZE; // as we said

	cfg.hal_read_f = _spiffs_read;
	cfg.hal_write_f = _spiffs_write;
	cfg.hal_erase_f = _spiffs_erase;

	// try and mount it
#if USE_CACHE
	ret = SPIFFS_mount(&this->fs, &cfg, this->fsWorkBuf, this->fsFileDescriptors,
					   FS_FDBUF_SIZE, this->fsCache, FS_CACHE_BUF_SIZE, 0);
#else
	ret = SPIFFS_mount(&this->fs, &cfg, this->fsWorkBuf, this->fsFileDescriptors,
					   FS_FDBUF_SIZE, nullptr, 0, 0);
#endif

	if(ret == SPIFFS_ERR_NOT_A_FS) {
		// if we just tried to re-format but it failed, error out
		if(triedFormat) {
			LOG(S_FATAL, "tried to reformat FS but that didn't work. something is real fucky");
			return ret;
		}

		LOG(S_WARN, "spiffs mount failure, not a filesystem: formatting");

		// attempt to format
		ret = SPIFFS_format(&this->fs);

		if(ret != SPIFFS_OK) {
			LOG(S_ERROR, "error formatting: %d", ret);
			return ret;
		} else {
			// if it was successful, attempt to mount it again
			LOG(S_INFO, "format success, re-mounting...");

			this->spiffsMount(true);
		}
	} else if(ret != 0) {
		// unmount the filesystem
		SPIFFS_unmount(&this->fs);

		LOG(S_ERROR, "spiffs mount returned %d, aborting");
		return ret;
	}

#if FORMAT_ON_MOUNT
	static bool formatted = false;

	if(!formatted) {
		// set flag to avoid formatting again
		formatted = true;

		LOG(S_WARN, "Formatting filesystem");

		// unmount
		SPIFFS_unmount(&this->fs);

		// attempt to format
		ret = SPIFFS_format(&this->fs);

		if(ret != SPIFFS_OK) {
			LOG(S_ERROR, "error formatting: %d", ret);
			return ret;
		} else {
			// if it was successful, attempt to mount it again
			LOG(S_INFO, "format success, re-mounting...");

			return this->spiffsMount(true);
		}
	}
#endif

	// get info about the fs
	uint32_t total, used;
	ret = SPIFFS_info(&this->fs, &total, &used);

	if(ret == 0) {
		LOG(S_INFO, "%d bytes used, %d bytes total", used, total);
	} else {
		LOG(S_ERROR, "error getting fs info: %d", ret);
		return ret;
	}

	// list all files
	this->spiffsListFiles();

	// if we get down here, we mounted successfully
	return 0;
}

/**
 * Prints a listing of all files in the SPIFFS filesystem.
 */
void Filesystem::spiffsListFiles(void) {
	spiffs_DIR d;
	struct spiffs_dirent e;
	struct spiffs_dirent *pe = &e;

	// open directory
	SPIFFS_opendir(&this->fs, "/", &d);

	// keep iterating
	while((pe = SPIFFS_readdir(&d, pe))) {
		LOG(S_DEBUG, "File %04x: %s – %d bytes", pe->obj_id, pe->name,
				pe->size);
	}

	// close it again
	SPIFFS_closedir(&d);
}

/**
 * Starts running the SPIFFS filesystem check function, then schedules a
 * timer for 30 seconds later to reset the system.
 */
void Filesystem::spiffsCheck(void) {
	BaseType_t ok;

	// set up timer
	TimerHandle_t timer;

	timer = xTimerCreate("SPIFFSCheck", (30000 / portTICK_PERIOD_MS), pdFALSE,
			nullptr, _FSCheckResetTimerCallback);

	if(timer == nullptr) {
		LOG(S_ERROR, "Couldn't create timer");
		return;
	}

	// start timer
	ok = xTimerStart(timer, portMAX_DELAY);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't start timer");
		return;
	}

	// run the check function
	SPIFFS_check(&this->fs);
}



/**
 * Posts a message to the filesystem queue.
 *
 * @param msg Message to post
 * @param timeout Maximum number of ticks to wait
 * @return 0 if successful, error code otherwise.
 */
int Filesystem::postFSRequest(fs_message_t *msg, int timeout) {
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
 * Attempts to open a file with the given set of flags.
 *
 * @param name Filename in flash
 * @param flags Open flags
 * @return Pointer to a newly allocated file, or nullptr if error.
 */
fs::File *Filesystem::open(const char *name, int flags) {
	int err;
	BaseType_t ok;

	// structures for the message
	fs_message_t msg;

	bool success = false;
	uint32_t returnValue = 0;
	fs_descriptor_t descriptor = -1;

	// initialize the completion semaphore
	SemaphoreHandle_t completion = xSemaphoreCreateBinary();

	if(completion == nullptr) {
		LOG(S_ERROR, "Can't allocate completion semaphore");
		return nullptr;
	}

	// set up message
	memset(&msg, 0, sizeof(msg));

	msg.completion = completion;

	// write in descriptor and type
	msg.descriptor = &descriptor;
	msg.type = kFSMessageOpenFile;
	// also populate success and return value pointers
	msg.successPtr = &success;
	msg.returnValuePtr = &returnValue;
	// set filename
	msg.payload.open.filename = name;

	// convert flags
	if(flags & Filesystem::READONLY) {
		msg.payload.open.flags = (fs_open_flags_t) (msg.payload.open.flags | kOpenReadOnly);
	} else if(flags & Filesystem::READWRITE) {
		msg.payload.open.flags = (fs_open_flags_t) (msg.payload.open.flags | kOpenReadWrite);
	}

	if(flags & Filesystem::TRUNCATE) {
		msg.payload.open.flags = (fs_open_flags_t) (msg.payload.open.flags | kOpenTruncate);
	} else if(flags & Filesystem::APPEND) {
		msg.payload.open.flags = (fs_open_flags_t) (msg.payload.open.flags | kOpenAppend);
	}

	if(flags & Filesystem::CREATE) {
		msg.payload.open.flags = (fs_open_flags_t) (msg.payload.open.flags | kOpenCreate);
	}
	if(flags & Filesystem::WRITETHROUGH) {
		msg.payload.open.flags = (fs_open_flags_t) (msg.payload.open.flags | kOpenWriteThrough);
	}

	// post message
	err = Filesystem::sharedInstance()->postFSRequest(&msg);

	if(err != 0) {
		vSemaphoreDelete(msg.completion);
		return nullptr;
	}

	// wait for completion
	ok = xSemaphoreTake(msg.completion, portMAX_DELAY);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't take semaphore: %u", ok);

		vSemaphoreDelete(msg.completion);
		return nullptr;
	}

	// delete semaphore
	vSemaphoreDelete(msg.completion);

	// if not successful, exit
	if(!success) {
		return nullptr;
	}

	// otherwise, create a file
	return new fs::File(descriptor);
}




/**
 * Reads the JDEC ID out of the SPI flash.
 */
void Filesystem::identifyFlash(void) {
	// read the jdec id and try to identify the flash
	uint32_t id = this->flashGetJDECId();

	for(unsigned int i = 0; i < numSupportedChips; i++) {
		const flash_info_t *chip = &(supportedChips[i]);

		// does the JDEC ID match?
		if(id == chip->jdecId) {
			LOG(S_INFO, "Identified flash: %s (%u bytes)", chip->name, chip->size);

			this->flashType = chip->type;
			this->flashSize = chip->size;
			break;
		}
	}

	if(this->flashType == kFlashTypeUnknown) {
		LOG(S_FATAL, "Couldn't identify flash with ID 0x%06x", id);
	}

	// perform per-chip initialization
	switch(this->flashType) {
		// is it a SST25VF016?
		case kFlashTypeSST25VF016:
			this->flashHAL = new fs::SST25VF016(this);
			break;

		// hope for the best if it's an unknown flash
		default: ;
	}
}
/**
 * Returns the JDEC ID value of the flash chip. This is used to uniquely
 * identify what chip is populated on the board.
 */
uint32_t Filesystem::flashGetJDECId(void) {
	int err;
	uint32_t id = 0;

	// start a flash transaction
	if(this->startFlashTransaction() != 0) {
		LOG(S_ERROR, "Couldn't start flash transaction for JDEC ID read");

		return 0;
	}

	// send the JDEC ID command
	err = this->flashCommand(FLASH_CMD_JDEC_ID);

	if(err != 0) {
		// end transaction
		this->endFlashTransaction();

		LOG(S_ERROR, "Couldn't send JDEC ID command");
		return 0;
	}

	// read the manufacturer byte
	id |= (this->spiRead() << 16);
	// read the memory type byte
	id |= (this->spiRead() << 8);
	// read the memory size byte
	id |= this->spiRead();

	// end the flash transaction we started earlier
	this->endFlashTransaction();

	return id;
}



/**
 * Sets the state of the flash's /CS line. When the active is true, the chip
 * is selected.
 */
void Filesystem::setFlashCS(bool active) {
	GPIO_WriteBit(FLASH_CS_PORT, FLASH_CS_PIN, active ? Bit_RESET : Bit_SET);
}

/**
 * Begins a transaction to the flash. This waits for the flash mutex, then
 * asserts /CS to select the flash.
 *
 * If this call times out, -1 is returned, 0 otherwise. If -1 is returned, the
 * state of the hardware was not altered in any way.
 *
 * @param timeout Timeout, in ticks (100/sec) to wait for the lock.
 *
 * @note Each call to this function MUST be paired with a call to
 * `endFlashTransaction` or deadlocks WILL result.
 */
int Filesystem::startFlashTransaction(int timeout) {
	// get the lock
	int lockTimeout = (timeout == -1) ? portMAX_DELAY : timeout;

	if(xSemaphoreTake(this->flashMutex, lockTimeout) == pdTRUE) {
		// assert /CS
		taskENTER_CRITICAL();

		this->setFlashCS(true);

		taskEXIT_CRITICAL();

		return 0;
	} else {
		return -1;
	}
}

/**
 * Ends a flash transaction. This will give the lock again and de-assert CS.
 */
void Filesystem::endFlashTransaction(void) {
	taskENTER_CRITICAL();

	// de-assert CS
	this->setFlashCS(false);

	// give the mutex back so another task can access the flash
	xSemaphoreGive(this->flashMutex);

	taskEXIT_CRITICAL();
}



/**
 * Waits for the SPI peripheral to be idle.
 */
void Filesystem::spiWaitIdle(void) {
    while(SPI_I2S_GetFlagStatus(FLASH_SPI_PERIPH, SPI_I2S_FLAG_BSY) == SET) {

    }
}
/**
 * Waits for the SPI to be ready to send another byte.
 */
void Filesystem::spiWaitTXReady(void) {
	// TODO: would this be better with interrupts?
    while(SPI_I2S_GetFlagStatus(FLASH_SPI_PERIPH, SPI_I2S_FLAG_TXE) == RESET) {

    }
}

/**
 * Writes a byte to the SPI peripheral.
 */
uint8_t Filesystem::spiWrite(uint8_t byte) {
	// waits for the SPI peripheral to be ready for data
	 this->spiWaitTXReady();

	// write a byte
	SPI_I2S_SendData(FLASH_SPI_PERIPH, byte);

	// poll the "RX buffer not empty" flag til it's set
	while(SPI_I2S_GetFlagStatus(FLASH_SPI_PERIPH, SPI_I2S_FLAG_RXNE) == RESET) {

	}

	uint8_t read = (uint8_t) SPI_I2S_ReceiveData(FLASH_SPI_PERIPH);

	(void) read; // for debugger
//	LOG(S_DEBUG, "wrote 0x%02x, read 0x%02x", byte, read);

	return read;
}
/**
 * Reads a byte from the SPI peripheral.
 */
uint8_t Filesystem::spiRead(void) {
	return this->spiWrite(0x00);
}

/**
 * Pulses the /CS line for approximately one byte time.
 */
void Filesystem::spiPulseCS(void) {
	// enter critical section
	taskENTER_CRITICAL();

	// wait for the peripheral to not be busy (all data sent)
	this->spiWaitIdle();

	// de-assert CS, write a dummy byte, wait for it to send, re-assert it
	this->setFlashCS(false);

	this->spiWrite(0x00);

	this->setFlashCS(true);

	// exit critical section
	taskEXIT_CRITICAL();
}



/**
 * Sends the given command to the flash chip.
 *
 * This wraps the write in a critical section. Technically, that's not needed
 * and increases interrupt latency slightly, since SCK will not be running when
 * we're not writing bytes and the flash won't be doing anything, but we don't
 * assume that the flash WON'T time out. This is for compatibility with other
 * flash devices later on.
 *
 * @note This should only be called if we're in a transaction already.
 */
int Filesystem::flashCommand(uint8_t command) {
	// ensure we're in a transaction
	// TODO: check that the transaction semaphore cannot be taken

	// enter a critical section
	taskENTER_CRITICAL();

	// check that the SPI is ready to accept a write and transmit the command
	this->spiWrite(command);

	// wait for the command to have been completely transmitted
//	this->spiWaitTXReady();

	// exit critical section
	taskEXIT_CRITICAL();

	// if we get down here, everything's good
	return 0;
}

/**
 * Sends a command to the flash, followed by the three lowest bytes of the
 * address. If requested, a dummy cycle is introduced after the address has
 * been sent.
 *
 * @note This should only be called if we're in a transaction already.
 */
int Filesystem::flashCommandWithAddress(uint8_t command, uint32_t address, bool dummyCycle) {
	int err;

	// enter a critical section
	taskENTER_CRITICAL();

	// send the command
	err = this->flashCommand(command);

	if(err != 0) {
		taskEXIT_CRITICAL();

		return err;
	}

	// write the address, MSB first
	this->spiWrite((address & 0x00FF0000) >> 16);
	this->spiWrite((address & 0x0000FF00) >> 8);
	this->spiWrite((address & 0x000000FF));

	// check if we need to do a dummy cycle
	if(dummyCycle) {
		this->spiWrite(0x00);
	}

	// exit the critical section
	taskEXIT_CRITICAL();

	// if we get down here, everything was good
	return 0;
}



/**
 * Perform a read operation from the SPI flash.
 */
s32_t _spiffs_read(u32_t addr, u32_t size, u8_t *dst) {
	int err = gFilesystem->flashHAL->read(addr, size, dst);

#if LOG_FLASH_READS
	LOG(S_DEBUG, "Read from 0x%06x (size %u): %d", addr, size, err);
#endif

	if(err != 0) {
		return SPIFFS_ERR_INTERNAL;
	} else {
		return SPIFFS_OK;
	}
}

/**
 * Performs a write operation to the SPI flash.
 */
s32_t _spiffs_write(u32_t addr, u32_t size, u8_t *src) {
	int err = gFilesystem->flashHAL->write(addr, size, src);

#if LOG_FLASH_WRITES
	LOG(S_DEBUG, "Write to 0x%06x (size %u): %d", addr, size, err);
#endif

	if(err != 0) {
		return SPIFFS_ERR_INTERNAL;
	} else {
		return SPIFFS_OK;
	}
}

/**
 * Erases the memory at the given address.
 */
s32_t _spiffs_erase(u32_t addr, u32_t size) {
	int err = gFilesystem->flashHAL->erase(addr, size);

#if LOG_FLASH_ERASE
	LOG(S_DEBUG, "Erase %u bytes at 0x%06x: %d", size, addr, err);
#endif

	if(err != 0) {
		return SPIFFS_ERR_INTERNAL;
	} else {
		return SPIFFS_OK;
	}
}
