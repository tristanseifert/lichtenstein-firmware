/*
 * FSPrivate.h
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */

#ifndef FS_FSPRIVATE_H_
#define FS_FSPRIVATE_H_

#include <stdint.h>

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



#endif /* FS_FSPRIVATE_H_ */
