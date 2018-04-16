/*
 * File.h
 *
 * Represents some operations that can be done on a file.
 *
 *  Created on: Mar 23, 2018
 *      Author: tristan
 */

#ifndef FS_FILE_H_
#define FS_FILE_H_

#include "Filesystem.h"

#include <cstdint>
#include <cstddef>

/*#ifndef FILE_PRIVATE
	#ifndef fs_message_t
		typedef void fs_message_t;
	#endif
#endif*/

class Filesystem;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace fs {
	class File {
		friend class ::Filesystem;

		private:
			File(fs_descriptor_t descriptor);
		public:
			virtual ~File();


		public:
			enum {
				END,
				SET,
				CURRENT
			};

		public:
			int close(void);

			int read(void *buffer, size_t length);
			int write(void *buffer, size_t length);

			int tell(void);
			int seek(int mode, ssize_t offset);



		private:
			int prepareMessage(fs_message_t *msg, fs_message_type_t type);
			int waitOnMessage(fs_message_t *msg);

		private:
			// descriptor passed to the constructor
			fs_descriptor_t descriptor;
			// whether the file has been opened or not
			bool isOpen = false;

	};
} /* namespace fs */

#pragma GCC diagnostic pop

#endif /* FS_FILE_H_ */
