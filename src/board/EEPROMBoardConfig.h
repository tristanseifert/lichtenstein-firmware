/*
 * EEPROMBoardConfig.h
 *
 *  Created on: Feb 15, 2018
 *      Author: tristan
 */

#ifndef BOARD_EEPROMBOARDCONFIG_H_
#define BOARD_EEPROMBOARDCONFIG_H_

#include <stdint.h>

/// magic value for board config struct
#define BOARD_CONFIG_MAGIC			0x12

/// current version of the board version struct
#define BOARD_CONFIG_VERSION			1

typedef struct __attribute__ ((__packed__)) {
	// magic value: this is always 0x12.
	uint8_t magic;
	// structure version
	uint8_t version;

	// these three values define the specific version of the hardware
	uint8_t hwModel;
	uint8_t hwVersion;
	uint8_t hwRevision;

	// checksum (add all bytes of the struct up to here, then negate it)
	uint8_t checksum;
} board_config_t;

#endif /* BOARD_EEPROMBOARDCONFIG_H_ */
