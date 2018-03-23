/*
 * FSPrivate.h
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */

#ifndef FS_FSPRIVATE_H_
#define FS_FSPRIVATE_H_

#include "FSTypesPublic.h"

#include <cstdint>

#include <LichtensteinApp.h>

/**
 * Enum defining the supported flash chips.
 */
typedef enum {
	kFlashTypeUnknown = -1,

	kFlashTypeSST25VF016 = 1, // SST25VF016B (HW = 1)
} flash_type_t;

/**
 * An array of this struct type is stored in memory and used to identify the
 * potential flash chips mounted on the board.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// JDEC ID
	uint32_t jdecId;
	// corresponding enum value
	flash_type_t type;

	// size, in bytes
	uint32_t size;

	// descriptive name
	const char *name;
} flash_info_t;

#pragma GCC diagnostic pop



/**
 * Message types sent to the FS task
 */
typedef enum {
	/**
	 * Opens a file for reading/writing.
	 */
	kFSMessageOpenFile						= 1,
	/**
	 * Closes a previously opened file.
	 */
	kFSMessageCloseFile,

	/**
	 * Reads from an already opened file descriptor.
	 */
	kFSMessageRead,
	/**
	 * Writes to an already opened file descriptor.
	 */
	kFSMessageWrite,

	/**
	 * Removes a file by name.
	 */
	kFSMessageRemove,
	/**
	 * Renames a file.
	 */
	kFSMessageRename,

	/**
	 * Gets information about a file (e.g. its size) by name.
	 */
	kFSMessageStat,

	/**
	 * Returns the current position into the file.
	 */
	kFSMessageTell,
	/**
	 * Seeks to the given position into the file.
	 */
	kFSMessageSeek,

	/**
	 * Flushes all pending writes on the file to the flash. This has no effect
	 * if the file was opened in write-through mode.
	 */
	kFSMessageFlush,
} fs_message_type_t;

/**
 * Message passed to the filesystem task.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	fs_message_type_t type;

	union {
		// opening a file
		struct {
			// filename of file to open
			const char *filename;
			// flags to pass to the open call
			fs_open_flags_t flags;
		} open;
		// closing a file
		struct {

		} close;

		// reading from a file
		struct {
			// buffer into which to read
			void *buffer;
			// how many bytes to read
			size_t length;
		} read;
		// writing to a file
		struct {
			// buffer from which to get the data
			void *buffer;
			// how many bytes to write
			size_t length;
		} write;

		// seek in a file/get current position
		struct {
			// mode to use (one of SPIFFS_SEEK_*)
			int mode;
			// new offset
			uint32_t offset;
		} seek;

		// deletes a file
		struct {
			// filename of file to delete
			const char *filename;
		} remove;

		// gets info about a file (stat)
		struct {
			// filename of file to get info on
			const char *filename;

			// pointer to a variable holding its size
			size_t *sizePtr;
		} stat;

		// flushes a file
		struct {

		} flush;

		// renames a file
		struct {
			// old filename
			const char *oldName;
			// new filename
			const char *newName;
		} rename;
	} payload;

	// mutex to wait on for completion
	SemaphoreHandle_t completion;
	// address of a uint32_t variable to store the return value into
	uint32_t *returnValuePtr;
	// address of a boolean to indicate success; it should be initialized to false
	bool *successPtr;

	// address of a file descriptor
	fs_descriptor_t *descriptor;
} fs_message_t;

#pragma GCC diagnostic pop



#endif /* FS_FSPRIVATE_H_ */
