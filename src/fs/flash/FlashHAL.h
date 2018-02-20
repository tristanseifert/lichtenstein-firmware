/*
 * FlashHAL.h
 *
 * Defines the general interface to accessing a flash chips. This class should
 * not be instantiated – rather, a subclass should be created for each of the
 * individual flash chips.
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */

#ifndef FS_FLASH_FLASHHAL_H_
#define FS_FLASH_FLASHHAL_H_

#include "../Filesystem.h"

#include <cstdint>

namespace fs {
	class FlashHAL {
		public:
			FlashHAL(Filesystem *_fs);
			virtual ~FlashHAL();

		public:
			virtual uint32_t getJDECID(void);

		public:
			virtual int read(uint32_t address, size_t size, void *dst) = 0;
			virtual int write(uint32_t address, size_t size, void *src) = 0;
			virtual int erase(uint32_t address, size_t size) = 0;

			virtual uint8_t getStatus(void) = 0;
			virtual void setStatus(uint8_t newStatus) = 0;

		private:
			static const uint8_t cmdJDECID = 0x9F;

		protected:
			Filesystem *fs = nullptr;
	};

} /* namespace fs */

#endif /* FS_FLASH_FLASHHAL_H_ */
