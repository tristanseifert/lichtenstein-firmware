/*
 * FlashCommands.h
 *
 * Various commands possible on standard SPI flash chips.
 *
 *  Created on: Feb 17, 2018
 *      Author: tristan
 */

#ifndef FS_FLASHCOMMANDS_H_
#define FS_FLASHCOMMANDS_H_


/**
 * Read status register: Returns the value of the status register.
 *
 * @note This is the only command we can run while a write operation is pending
 *
 * Address bytes: 0
 * Dummy cycles: 0
 * Data cycles: 1
 */
#define FLASH_CMD_RDSR						0x05


/**
 * Read device ID: Allows reading the manufacturer ID byte (addres 0x000000) or
 * the device ID byte (address 0x000001).
 *
 * Address bytes: 3
 * Dummy cycles: 0
 * Data cycles: 1
 */
#define FLASH_CMD_RDID						0x90

/**
 * Read JDEC ID: Allows reading the JDEC ID of the flash chip.
 *
 * Address bytes: 0
 * Dummy cycles: 0
 * Data cycles: 3
 */
#define FLASH_CMD_JDEC_ID					0x9F


/**
 * Read: Allows reading one or more bytes from the flash array at a maximum bus
 * speed of 25MHz.
 *
 * Address bytes: 3
 * Dummy cycles: 0
 * Data cycles: 1 to inf
 */
#define FLASH_CMD_READ_SLOW					0x03
#define FLASH_CMD_READ						FLASH_CMD_READ_SLOW

/**
 * Fast read: Allows reading one or more bytes from the flash array at the
 * fastest speed supported by the chip (typically 80MHz)
 *
 * Address bytes: 3
 * Dummy cycles: 1
 * Data cycles: 1 to inf
 */
#define FLASH_CMD_READ_FAST					0x0B


/**
 * Write enable: enables writing to the array.
 *
 * Address bytes: 0
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_WREN						0x06

/**
 * Write disable: disables writing to the array.
 *
 * Address bytes: 0
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_WRDI						0x04

/**
 * Byte write: Writes a single byte to an arbitrary address in the flash array.
 *
 * Address bytes: 3
 * Dummy cycles: 0
 * Data cycles: 1
 */
#define FLASH_CMD_WRITE_BYTE					0x02

/**
 * Auto-increment word write: Writes continuous words to the flash array. The
 * address must have A0 clear.
 *
 * Address bytes: 3
 * Dummy cycles: 0
 * Data cycles: 2 to inf
 */
#define FLASH_CMD_WRITE_AIW					0xAD


/**
 * Sector erase: Erases a 4KByte sector at the given address.
 *
 * Address bytes: 3
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_ERASE_4K					0x20

/**
 * 32k Block erase: Erases a 32KByte block at the given address.
 *
 * Address bytes: 3
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_ERASE_32K					0x52

/**
 * 64k Block erase: Erases a 64KByte block at the given address.
 *
 * Address bytes: 3
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_ERASE_64K					0xD8

/**
 * Chip erase: Erases the ENTIRE memory array.
 *
 * Address bytes: 0
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_ERASE_ARRAY				0x60



/**
 * Enable busy output: When executed before a word program operation, the state
 * of the flash array is put on the SO line after /CS has been toggled. When
 * the line pulses, the write operation has completed.
 *
 * Address bytes: 0
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_EBSY						0x70

/**
 * Disable busy output: Disables the busy output.
 *
 * Address bytes: 0
 * Dummy cycles: 0
 * Data cycles: 0
 */
#define FLASH_CMD_DBSY						0x80

#endif /* FS_FLASHCOMMANDS_H_ */
