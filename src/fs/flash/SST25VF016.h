/*
 * SST25VF016.h
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */

#ifndef FS_FLASH_SST25VF016_H_
#define FS_FLASH_SST25VF016_H_

#include "FlashHAL.h"
#include "../Filesystem.h"

#include <cstdint>

namespace fs {
	class SST25VF016: public FlashHAL {
		public:
			SST25VF016(Filesystem *_fs);
			virtual ~SST25VF016();

		public:
			virtual int read(uint32_t address, size_t size, void *dst);
			virtual int write(uint32_t address, size_t size, void *src);
			virtual int erase(uint32_t address, size_t size);

			virtual uint8_t getStatus(void);
			virtual void setStatus(uint8_t newStatus);

		private:
			void flashWaitTBP(void);
			void flashWaitSectorErase(void);

		private:
			void checkWriteProtectState(void);

			void unWriteProtect(void);

			// bits for block write protection
			static const uint8_t statusBPMask = 0x3C;
			// block protection lock-down bit
			static const uint8_t statusBPL = 0x80;

		private:
			static const uint8_t cmdReadSlow = 0x03;

			static const uint8_t cmdWriteEnable = 0x06; // WREN
			static const uint8_t cmdWriteDisable = 0x04; // WRDI

			static const uint8_t cmdWriteAIW = 0xAD;
			static const uint8_t cmdWriteByte = 0x02;

			static const uint8_t cmdErase4kBlock = 0x20;

			static const uint8_t cmdReadStatus = 0x05; // RDSR
			static const uint8_t cmdWriteStatus = 0x01; // WRSR
			static const uint8_t cmdEnableStatusWrite = 0x50; // EWSR
	};
} /* namespace fs */

#endif /* FS_FLASH_SST25VF016_H_ */
