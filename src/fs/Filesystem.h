/*
 * Filesystem.h
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */

#ifndef FS_FILESYSTEM_H_
#define FS_FILESYSTEM_H_

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "spiffs/src/spiffs.h"

class Filesystem {
	public:
		static void init(void);
		static Filesystem *sharedInstance(void) noexcept;

	private:
		Filesystem();

		void setUpGPIOs(void);
		void setUpSPI(void);
		void setUpIRQs(void);
		void setUpDMA(void);

		void setUpTask(void);

		virtual ~Filesystem();

	private:
		void setFlashCS(bool active);

		int startFlashTransaction(int timeout = -1);
		void endFlashTransaction(void);

		int flashCommand(uint8_t command);
		int flashCommandWithAddress(uint8_t command, uint32_t address, bool dummyCycle = false);

		void flashWaitTBP(void);
		void flashWaitSectorErase(void);

		int flashRead(uint32_t address, size_t size, void *dst);
		int flashWrite(uint32_t address, size_t size, void *src);
		int flashErase(uint32_t address, size_t size);

		SemaphoreHandle_t flashMutex;

	private:
		void spiPulseCS(void);

		void spiWaitIdle(void);
		void spiWaitTXReady(void);
		void spiWaitRXReady(void);

		void spiWrite(uint8_t byte);
		uint8_t spiRead(void);

	private:
		friend void _FSTaskTrampoline(void *);
		void taskEntry(void);

		TaskHandle_t task;

	private:
		friend s32_t _spiffs_read(u32_t addr, u32_t size, u8_t *dst);
		friend s32_t _spiffs_write(u32_t addr, u32_t size, u8_t *src);
		friend s32_t _spiffs_erase(u32_t addr, u32_t size);

		spiffs fs;

		uint8_t *fsWorkBuf = nullptr;
		uint8_t *fsFileDescriptors = nullptr;
		uint8_t *fsCache = nullptr;

		void spiffsMount(void);
};

#endif /* FS_FILESYSTEM_H_ */
