/*
 * Filesystem.h
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */

#ifndef FS_FILESYSTEM_H_
#define FS_FILESYSTEM_H_

#include "FSPrivate.h"

#include <LichtensteinApp.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

#include "spiffs/src/spiffs.h"

#pragma GCC diagnostic pop

namespace fs {
	class FlashHAL;
	class SST25VF016;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class Filesystem {
	friend class fs::FlashHAL;
	friend class fs::SST25VF016;

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

		SemaphoreHandle_t flashMutex = nullptr;

	private:
		void spiPulseCS(void);

		void spiWaitIdle(void);
		void spiWaitTXReady(void);

		uint8_t spiWrite(uint8_t byte);
		uint8_t spiRead(void);

	private:
		friend void _FSTaskTrampoline(void *);
		void taskEntry(void);

		static const size_t taskStackSize = 300;
		static const int taskPriority = 3;

		TaskHandle_t task = nullptr;

	private:
		friend s32_t _spiffs_read(u32_t addr, u32_t size, u8_t *dst);
		friend s32_t _spiffs_write(u32_t addr, u32_t size, u8_t *src);
		friend s32_t _spiffs_erase(u32_t addr, u32_t size);

		spiffs fs;

		uint8_t *fsWorkBuf = nullptr;
		uint8_t *fsFileDescriptors = nullptr;
		uint8_t *fsCache = nullptr;

		int spiffsMount(bool triedFormat = false);

	private:
		void identifyFlash(void);
		uint32_t flashGetJDECId(void);

		fs::FlashHAL *flashHAL = nullptr;
		size_t flashSize = 0;

		flash_type_t flashType = kFlashTypeUnknown;
};

#pragma GCC diagnostic pop

#endif /* FS_FILESYSTEM_H_ */
