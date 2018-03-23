/*
 * FSTypesPublic.h
 *
 * Public types used by the filesystem implementation.
 *
 *  Created on: Mar 23, 2018
 *      Author: tristan
 */

#ifndef FS_FSTYPESPUBLIC_H_
#define FS_FSTYPESPUBLIC_H_

#include <cstdint>

#include "spiffs/src/spiffs.h"

/**
 * Flags for opening a file.
 */
typedef enum {
	/**
	 * Any writes to a file opened with this flag are appended to the end of
	 * its current contents.
	 */
	kOpenAppend								= SPIFFS_O_APPEND,
	/**
	 * Truncates the file to zero length when it's opened.
	 */
	kOpenTruncate							= SPIFFS_O_TRUNC,

	/**
	 * Creates the file if it doesn't already exist.
	 */
	kOpenCreate								= SPIFFS_O_CREAT,

	/**
	 * Opens the file as read-only.
	 */
	kOpenReadOnly							= SPIFFS_O_RDONLY,
	/**
	 * Opens the file as write-only.
	 */
	kOpenWriteOnly							= SPIFFS_O_WRONLY,
	/**
	 * Opens the file as both read/write.
	 */
	kOpenReadWrite							= (kOpenReadOnly | kOpenWriteOnly),

	/**
	 * Disables caching for writes.
	 */
	kOpenWriteThrough						= SPIFFS_O_DIRECT
} fs_open_flags_t;



/**
 * Define the file descriptor type.
 */
typedef spiffs_file fs_descriptor_t;



#endif /* FS_FSTYPESPUBLIC_H_ */
