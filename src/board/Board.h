/*
 * Board.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_

#include <LichtensteinApp.h>

#include "EEPROMBoardConfig.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class Board {
	public:
		static void init(void);
		static Board* sharedInstance(void);

	public:
		typedef enum {
			kBoardLEDIdle,
			kBoardLEDIPAct,
			kBoardLEDOutputAct,
			kBoardLEDError
		} board_led_t;

		void toggleLED(board_led_t led);
		void setLED(board_led_t led, bool set);

	private:
		void getGPIOForLED(board_led_t, GPIO_TypeDef **, uint16_t *);

	private:
		Board();
		void initStatusGPIOs(void);

	private:
		void initTestGPIOs(void);

		void testSwIRQ(void);

		uint8_t testSwState = 0;

	// this has to be public to be callable from the IRQ. do not call elsewhere
	public:
		static void TestSwIRQNotify(void) {
			Board::sharedInstance()->testSwIRQ();
		}

	private:
		static const uint8_t configEEPROMI2CAddress = 0x50;
		// page size mask (for 24AA02E48)
		static const uint8_t configEEPROMPageSize = 0x08;
		static const uint8_t configEEPROMPageSizeMask = 0x07;

		static const uint8_t configEEPROMHWInfoOffset = 0x00;
		static const int configEEPROMWriteTimeout = 50000;

		board_config_t config;

		void initConfigEEPROM(void);

		void i2cResetFix(void);

		void i2cWaitForIdle(void);
		int i2cStart(uint8_t address, bool read = true, int timeout = 20000);
		void i2cStop(void);
		void i2cWriteByte(uint8_t data);
		uint8_t i2cReadByte(bool ack = true, int timeout = 20000);

		void _testWriteBoardConfig(void);

		SemaphoreHandle_t i2cMutex = nullptr;

	public:
		void configEEPROMRead(void *buf, uint8_t address, uint8_t numBytes);

	private:
		void configEEPROMWrite(void *buf, uint8_t address, uint8_t numBytes);

		static uint8_t calculateConfigChecksum(board_config_t *cfg);

	private:
		virtual ~Board();
};

#pragma GCC diagnostic pop

#endif /* BOARD_BOARD_H_ */
