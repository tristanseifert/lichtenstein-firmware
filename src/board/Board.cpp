/*
 * Board.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "Board.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_device.h"

#include <cstring>

#define DEBUG_I2C_WRITES					0

#if HW == HW_MUSTARD
// STATUS0 LED (idle)
#define STATUS0_PORT						GPIOD
#define STATUS0_PIN						GPIO_Pin_2
#define STATUS0_GPIO_CLOCK				RCC_APB2Periph_GPIOD

// STATUS1 LED (IP act)
#define STATUS1_PORT						GPIOD
#define STATUS1_PIN						GPIO_Pin_3
#define STATUS1_GPIO_CLOCK				RCC_APB2Periph_GPIOD

// STATUS2 LED (LED output on)
#define STATUS2_PORT						GPIOD
#define STATUS2_PIN						GPIO_Pin_4
#define STATUS2_GPIO_CLOCK				RCC_APB2Periph_GPIOD

// STATUS3 LED (Error)
#define STATUS3_PORT						GPIOD
#define STATUS3_PIN						GPIO_Pin_7
#define STATUS3_GPIO_CLOCK				RCC_APB2Periph_GPIOD

// TEST_SW0
#define TEST_SW0_PORT					GPIOC
#define TEST_SW0_PIN						GPIO_Pin_13
#define TEST_SW0_GPIO_CLOCK				RCC_APB2Periph_GPIOC

#define TEST_SW0_GPIO_EXTI				EXTI15_10_IRQHandler
#define TEST_SW0_GPIO_EXTI_PORT			GPIO_PortSourceGPIOC
#define TEST_SW0_GPIO_EXTI_PIN			GPIO_PinSource13
#define TEST_SW0_GPIO_EXTI_LINE			EXTI_Line13
#define TEST_SW0_GPIO_EXTI_NVIC_IRQ		EXTI15_10_IRQn
#define TEST_SW0_GPIO_EXTI_PRIORITY		(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)

// TEST_SW1
#define TEST_SW1_PORT					GPIOB
#define TEST_SW1_PIN						GPIO_Pin_2
#define TEST_SW1_GPIO_CLOCK				RCC_APB2Periph_GPIOB

#define TEST_SW1_GPIO_EXTI				EXTI2_IRQHandler
#define TEST_SW1_GPIO_EXTI_PORT			GPIO_PortSourceGPIOB
#define TEST_SW1_GPIO_EXTI_PIN			GPIO_PinSource2
#define TEST_SW1_GPIO_EXTI_LINE			EXTI_Line2
#define TEST_SW1_GPIO_EXTI_NVIC_IRQ		EXTI2_IRQn
#define TEST_SW1_GPIO_EXTI_PRIORITY		(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)

// when set, the EXTI is shared among both test switch inputs
#define TEST_SW_GPIO_EXTI_SHARED			0

// hardware config for EEPROM
// I2C peripheral to use
#define CFG_EEPROM_I2C					I2C1
#define CFG_EEPROM_I2C_CLOCK				RCC_APB1Periph_I2C1

// what speed to run the I2C bus at, in Hz
#define	CFG_EEPROM_I2C_SPEED				(400 * 1000)

// pins on which SDA and SCL are connected
#define CFG_EEPROM_I2C_SDA_PORT			GPIOB
#define CFG_EEPROM_I2C_SDA_PIN			GPIO_Pin_9
#define CFG_EEPROM_I2C_SDA_GPIO_CLOCK	RCC_APB2Periph_GPIOB
#define CFG_EEPROM_I2C_SCL_PORT			GPIOB
#define CFG_EEPROM_I2C_SCL_PIN			GPIO_Pin_8
#define CFG_EEPROM_I2C_SCL_GPIO_CLOCK	RCC_APB2Periph_GPIOB

// if I2C is an alternate function, set it here
#define CFG_EEPROM_I2C_REMAP				1
#define CFG_EEPROM_I2C_REMAP_ARG			GPIO_Remap_I2C1
#endif


static Board *gBoard = nullptr;


/**
 * Initializes the shared board instance.
 */
void Board::init() {
	if(!gBoard) {
		gBoard = new Board();
	}
}

/**
 * Returns the shared instance of the board.
 */
Board *Board::sharedInstance() {
	return gBoard;
}

/**
 * Initializes the board.
 */
Board::Board() {
	this->initStatusGPIOs();
	this->initTestGPIOs();
	this->initConfigEEPROM();
}
Board::~Board() {
	// TODO Auto-generated destructor stub
}



/**
 * Initializes the GPIO pins for the status LEDs.
 */
void Board::initStatusGPIOs(void) {
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;

	// set up STATUS0
	RCC_APB2PeriphClockCmd(STATUS0_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS0_PIN;
	GPIO_Init(STATUS0_PORT, &gpio);
	// set up STATUS1
	RCC_APB2PeriphClockCmd(STATUS1_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS1_PIN;
	GPIO_Init(STATUS1_PORT, &gpio);
	// set up STATUS2
	RCC_APB2PeriphClockCmd(STATUS2_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS2_PIN;
	GPIO_Init(STATUS2_PORT, &gpio);
	// set up STATUS3
	RCC_APB2PeriphClockCmd(STATUS3_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS3_PIN;
	GPIO_Init(STATUS3_PORT, &gpio);

	// set default state for LEDs
	this->setLED(kBoardLEDIdle, 0);
	this->setLED(kBoardLEDIPAct, 0);
	this->setLED(kBoardLEDOutputAct, 0);
	this->setLED(kBoardLEDError, 0);
}

/**
 * Sets the state of the status indicators. Each LED on the board has an index:
 * - 0: idle
 * - 1: IP Act
 * - 2: LED out active
 * - 3: Error
 */
void Board::setLED(board_led_t led, uint8_t state) {
	if(led == kBoardLEDIdle) {
		GPIO_WriteBit(STATUS0_PORT, STATUS0_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	} else if(led == kBoardLEDIPAct) {
		GPIO_WriteBit(STATUS1_PORT, STATUS1_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	} else if(led == kBoardLEDOutputAct) {
		GPIO_WriteBit(STATUS2_PORT, STATUS2_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	} else if(led == kBoardLEDError) {
		GPIO_WriteBit(STATUS3_PORT, STATUS3_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	}
}




/**
 * Initializes the GPIOs used for entering various test modes via a DIP switch.
 * These inputs will be at 3V3 when the associated bit is set, or weakly pulled
 * down otherwise.
 */
void Board::initTestGPIOs(void) {
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;


	// set up TEST_SW0
	RCC_APB2PeriphClockCmd(TEST_SW0_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = TEST_SW0_PIN;
	GPIO_Init(TEST_SW0_PORT, &gpio);

	// set up TEST_SW1
	RCC_APB2PeriphClockCmd(TEST_SW1_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = TEST_SW1_PIN;
	GPIO_Init(TEST_SW1_PORT, &gpio);


	// enable the external interrupts from those pins
	EXTI_InitTypeDef exti;
	NVIC_InitTypeDef nvic;
	EXTI_StructInit(&exti);

	GPIO_EXTILineConfig(TEST_SW0_GPIO_EXTI_PORT, TEST_SW0_GPIO_EXTI_PIN);
	GPIO_EXTILineConfig(TEST_SW1_GPIO_EXTI_PORT, TEST_SW1_GPIO_EXTI_PIN);

	// configure the SW0 exti and then enable the interrupt
	exti.EXTI_Line = TEST_SW0_GPIO_EXTI_LINE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	nvic.NVIC_IRQChannel = TEST_SW0_GPIO_EXTI_NVIC_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = TEST_SW0_GPIO_EXTI_PRIORITY;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	// configure the SW1 exti and then enable the interrupt
	exti.EXTI_Line = TEST_SW1_GPIO_EXTI_LINE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	nvic.NVIC_IRQChannel = TEST_SW1_GPIO_EXTI_NVIC_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = TEST_SW1_GPIO_EXTI_PRIORITY;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

/**
 * Called from the IRQ handler for the test switches. This will read out the
 * value of both switches, and notify other processes.
 */
void Board::testSwIRQ(void) {
	bool sw0 = (GPIO_ReadInputDataBit(TEST_SW0_PORT, TEST_SW0_PIN) == Bit_SET);
	bool sw1 = (GPIO_ReadInputDataBit(TEST_SW1_PORT, TEST_SW1_PIN) == Bit_SET);

	this->testSwState = (uint8_t) ((sw0 ? 0x01 : 0x00) | (sw1 ? 0x02 : 0x00));

//	this->setLED(kBoardLEDIPAct, sw0);
//	this->setLED(kBoardLEDOutputAct, sw1);
}

/**
 * Handles the IRQ for the TEST_SW0 external interrupt.
 */
extern "C" void TEST_SW0_GPIO_EXTI(void) {
	Board::TestSwIRQNotify();
}

/**
 * Handles the IRQ for the TEST_SW1 external interrupt, if it is different from
 * the one used by the first switch.
 */
#if !TEST_SW_GPIO_EXTI_SHARED
extern "C" void TEST_SW1_GPIO_EXTI(void) {
	Board::TestSwIRQNotify();
}
#endif



/**
 * Sets up the I2C peripheral connected to the configuration EEPROM, then reads
 * information out of the EEPROM to determine what hardware version we're
 * running on.
 */
void Board::initConfigEEPROM(void) {
	// turn in the clock for the AFIO module
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// Set up GPIOs for I2C
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	// set up SDA line
	RCC_APB2PeriphClockCmd(CFG_EEPROM_I2C_SDA_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = CFG_EEPROM_I2C_SDA_PIN;
	GPIO_Init(CFG_EEPROM_I2C_SDA_PORT, &gpio);

	// set up SCL line
	RCC_APB2PeriphClockCmd(CFG_EEPROM_I2C_SCL_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = CFG_EEPROM_I2C_SCL_PIN;
	GPIO_Init(CFG_EEPROM_I2C_SCL_PORT, &gpio);

	// apply pin remap if needed
#if CFG_EEPROM_I2C_REMAP
	GPIO_PinRemapConfig(CFG_EEPROM_I2C_REMAP_ARG, ENABLE);
#endif


	// enable clocks for the I2C module and reset it
    RCC_APB1PeriphClockCmd(CFG_EEPROM_I2C_CLOCK, ENABLE);

    RCC_APB1PeriphResetCmd(CFG_EEPROM_I2C_CLOCK, ENABLE);
    RCC_APB1PeriphResetCmd(CFG_EEPROM_I2C_CLOCK, DISABLE);

    	// initialize the I2C module
	I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c);

	i2c.I2C_ClockSpeed = CFG_EEPROM_I2C_SPEED;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00;
	i2c.I2C_Ack = I2C_Ack_Disable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(CFG_EEPROM_I2C, &i2c);

	// enable the I2C
    I2C_Cmd(CFG_EEPROM_I2C, ENABLE);

    this->i2cResetFix();

    // TEST: write config info
//    this->_testWriteBoardConfig();

    // read out the config info and verify it
    memset(&this->config, 0, sizeof(board_config_t));
    this->configEEPROMRead(&this->config, Board::configEEPROMHWInfoOffset, sizeof(board_config_t));

    uint8_t checksum = Board::calculateConfigChecksum(&this->config);

    if(checksum != this->config.checksum) {
    		trace_printf("ERROR: mismatch in config checksum; computed 0x%02x, got 0x%02x\n", checksum, this->config.checksum);
    }

    if(this->config.version != BOARD_CONFIG_VERSION) {
			trace_printf("ERROR: invalid config version %u\n", this->config.version);
	}

    if(this->config.magic != BOARD_CONFIG_MAGIC) {
		trace_printf("ERROR: invalid magic %u\n", this->config.magic);
	}

    trace_printf("running on hw 0x%02x, version 0x%02x.%02x\n",
    				this->config.hwModel, this->config.hwVersion,
				this->config.hwRevision);
}

/**
 * Sends a STOP condition, 16 cocks, and another STOP condition to fix the I2C
 * bus state after a micro reset.
 */
void Board::i2cResetFix(void) {
//	this->i2cStop();

/*	I2C_SendData(CFG_EEPROM_I2C, 0x00);
	I2C_SendData(CFG_EEPROM_I2C, 0x00);*/

//	this->i2cStop();
}

/**
 * Waits for the I2C interface to be idle.
 */
void Board::i2cWaitForIdle(void) {
	while(I2C_GetFlagStatus(CFG_EEPROM_I2C, I2C_FLAG_BUSY)) {
		// do stuff here? idk
	}
}

/**
 * Sends a START signal on the bus with the given device address.
 *
 * @param address 7-bit device address. This will be shifted left before it's
 * transmitted.
 * @param read When true, we're operating in read mode, write otherwise.
 * @param timeout Arbitrary "counts" to wait for the slave to respond. Set
 * to zero (or omit) for default.
 *
 * @return 0 if an ACK was received, 1 otherwise.
 */
int Board::i2cStart(uint8_t address, bool read, int timeout) {
	// sanity check timeout
	if(timeout <= 0) {
		timeout = 20000;
	}

	uint8_t direction = read ? I2C_Direction_Receiver : I2C_Direction_Transmitter;

	// re-enable acknowledgements
	I2C_AcknowledgeConfig(CFG_EEPROM_I2C, ENABLE);

	// wait until I2C is not busy anymore
	// TODO: we really should do this
//	this->i2cWaitForIdle();

	// generate START condition
	I2C_GenerateSTART(CFG_EEPROM_I2C, ENABLE);

	// wait for I2C EV5 (slave has acknowledged start condition)
	int counter = timeout;

	while(!I2C_CheckEvent(CFG_EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
		// decrement timeout
		if(counter-- == 0) {
			// generate a stop condition and return if it hits zero
			this->i2cStop();

			return 1;
		}
	}

//	trace_printf("Acknowledged in %u tries\n", 50000 - timeout);

	// send slave address
	I2C_Send7bitAddress(CFG_EEPROM_I2C, (uint8_t) (address <<  1), direction);

	/*
	 * wait for I2C1 EV6, check if the slave either acknowledged the master as
	 * a transmitter, or as a receiver
	 */
	counter = timeout;
	if(direction == I2C_Direction_Transmitter) {
		while(!I2C_CheckEvent(CFG_EEPROM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			// decrement timeout
			if(counter-- == 0) {
				// generate a stop condition and return if it hits zero
				this->i2cStop();

				return 1;
			}
		}
	} else if(direction == I2C_Direction_Receiver) {
		while(!I2C_CheckEvent(CFG_EEPROM_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			// decrement timeout
			if(counter-- == 0) {
				// generate a stop condition and return if it hits zero
				this->i2cStop();

				return 1;
			}
		}
	}

	return 0;
}

/**
 * Generates a STOP condition on the I2C bus.
 */
void Board::i2cStop(void) {
	I2C_GenerateSTOP(CFG_EEPROM_I2C, ENABLE);
}

/**
 * Writes a single byte to the I2C bus. This assumes that the address has been
 * sent already.
 */
void Board::i2cWriteByte(uint8_t data) {
	I2C_SendData(CFG_EEPROM_I2C, data);

	// wait for the byte to be transmitted
	while(!I2C_CheckEvent(CFG_EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {

	}
}

/**
 * Reads a single byte from the device. If ack is true, an acknowledgement is
 * sent after the read, otherwise none is sent.
 */
uint8_t Board::i2cReadByte(bool ack, int timeout) {
	int counter = timeout;

	// set the acknowledge config
	if(ack) {
		I2C_AcknowledgeConfig(CFG_EEPROM_I2C, ENABLE);
	} else {
		I2C_AcknowledgeConfig(CFG_EEPROM_I2C, DISABLE);
//		I2C_GenerateSTOP(CFG_EEPROM_I2C, ENABLE);
	}

	// wait to receive a byte
	while(!I2C_CheckEvent(CFG_EEPROM_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if(counter-- == 0) {
			trace_printf("READ ERROR: timeout reading byte\n");
			return 0;
		}
	}

	// read the byte
	uint8_t data = I2C_ReceiveData(CFG_EEPROM_I2C);
	return data;
}

/**
 * Writes a MAC address to the EEPROM.
 */
void Board::_testWriteBoardConfig(void) {
	trace_puts("WRITING BOARD CONFIG TO EEPROM!");

	this->config.hwModel = HW;
	this->config.hwRevision = 0x00;
	this->config.hwVersion = 0x01;

	this->config.magic = BOARD_CONFIG_MAGIC;
	this->config.version = BOARD_CONFIG_VERSION;

	this->config.checksum = Board::calculateConfigChecksum(&this->config);

	this->configEEPROMWrite(&this->config, Board::configEEPROMHWInfoOffset, sizeof(board_config_t));
}

/**
 * Reads a specified number of bytes from the EEPROM, starting at the given
 * address.
 */
void Board::configEEPROMRead(void *buf, uint8_t address, uint8_t numBytes) {
	// sanity checking
	if(buf == nullptr) {
		return;
	}

	// enter critical section
	taskENTER_CRITICAL();

	// send the address
	if(this->i2cStart(Board::configEEPROMI2CAddress, false) == 1) {
		trace_puts("Timeout starting I2C transaction for address write!\n");

		// leave critical section
		taskEXIT_CRITICAL();

		return;
	}

	this->i2cWriteByte(address);

	this->i2cStop();

	// read the number of bytes the user wants
	if(this->i2cStart(Board::configEEPROMI2CAddress, true) == 1) {
		trace_puts("Timeout starting I2C transaction for data read!\n");

		// leave critical section
		taskEXIT_CRITICAL();

		return;
	}

	uint8_t *byteBuf = static_cast<uint8_t *>(buf);

	for(int i = 0; i < numBytes; i++) {
		bool acknowledge = (i == (numBytes - 1)) ? false : true;
		byteBuf[i] = this->i2cReadByte(acknowledge);
	}

	// send a STOP condition
	this->i2cStop();

	// leave critical section
	taskEXIT_CRITICAL();
}

/**
 * Writes to the config EEPROM. This is really only used to write the initial
 * hardware configuration when the board is first set up.
 */
void Board::configEEPROMWrite(void *buf, uint8_t address, uint8_t numBytes) {
	int bytesWritten = 0;
	uint8_t *ptr = static_cast<uint8_t *>(buf);

	// sanity checking
	if(buf == nullptr) {
		return;
	}

	// enter critical section
	taskENTER_CRITICAL();

#if DEBUG_I2C_WRITES
	trace_printf("** starting write to 0x%02x, size 0x%02x\n", address, numBytes);
#endif

	// write the first page
	uint8_t firstPage = (address & ~Board::configEEPROMPageSizeMask);
	uint8_t offsetIntoPage = address - firstPage;
	uint8_t bytesToWrite = Board::configEEPROMPageSize - offsetIntoPage;

	uint8_t writeAddress = address;

	if(numBytes < bytesToWrite) {
		bytesToWrite = numBytes;
	}

	// send address
#if DEBUG_I2C_WRITES
	trace_printf("\tfirst page is 0x%02x, writing to offset 0x%02x, address 0x%02x, (%u bytes)\n\t", firstPage, offsetIntoPage, writeAddress, bytesToWrite);
#endif

	if(this->i2cStart(Board::configEEPROMI2CAddress, false, Board::configEEPROMWriteTimeout) == 1) {
		trace_puts("Timeout starting I2C transmission for write\n");

		// leave critical section
		taskEXIT_CRITICAL();

		return;
	}

	this->i2cWriteByte(writeAddress);

	// send data
	for(int i = 0; i < bytesToWrite; i++) {
		this->i2cWriteByte(*ptr++);

#if DEBUG_I2C_WRITES
		trace_printf("#");
#endif
		bytesWritten++; writeAddress++;
	}

#if DEBUG_I2C_WRITES
	trace_printf("\n\tsending stop condition\n");
#endif

	// have we got any more bytes to write?
	if(bytesWritten < numBytes) {
		int bytesLeftToWrite = numBytes - bytesWritten;

#if DEBUG_I2C_WRITES
		trace_printf("\tremaining bytes to write: %u, starting at 0x%02x\n", bytesLeftToWrite, writeAddress);
#endif

		for(int i = 0, j = bytesLeftToWrite; i < j; i++) {
			if((writeAddress & Board::configEEPROMPageSizeMask) == 0) {
#if DEBUG_I2C_WRITES
				trace_printf("\tstarting page write at 0x%02x for %u bytes\n\t", writeAddress, bytesLeftToWrite & Board::configEEPROMPageSizeMask);
#endif

				// send address
				if(this->i2cStart(Board::configEEPROMI2CAddress, false, Board::configEEPROMWriteTimeout) == 1) {
					trace_puts("Timeout starting I2C transmission for write\n");

					// leave critical section
					taskEXIT_CRITICAL();

					return;
				}

				this->i2cWriteByte(writeAddress);
			}

			// write byte
			this->i2cWriteByte(*ptr++);

#if DEBUG_I2C_WRITES
			trace_printf("*");
#endif
			bytesWritten++; writeAddress++;

			// have we reached the end of the page?
			if((writeAddress & Board::configEEPROMPageSizeMask) == 0) {
#if DEBUG_I2C_WRITES
				trace_printf("\n\tsending stop condition after writing %u bytes, have %u bytes left\n", Board::configEEPROMPageSize, numBytes - bytesWritten);
#endif

				if((numBytes - bytesWritten) != 0) {
					this->i2cStop();
				}
			}
		}

		// send a stop condition again in case we haven't already
#if DEBUG_I2C_WRITES
		trace_printf("\n\tsending stop condition\n");
#endif

		this->i2cStop();
	}

#if DEBUG_I2C_WRITES
	trace_printf("done writing\n");
#endif

	// leave critical section
	taskEXIT_CRITICAL();
}

/**
 * Calculates checksum over a board config struct.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"

uint8_t Board::calculateConfigChecksum(board_config_t *cfg) {
	uint8_t *ptr = reinterpret_cast<uint8_t *>(cfg);
	int size = sizeof(board_config_t);
	int positionOfChecksumFromEnd = size - offsetof(board_config_t, checksum);

//	trace_printf("Size of board_config_t: %u, checksum at byte %u from end\n", size, positionOfChecksumFromEnd);

	// calculate the checksum over the size
	uint8_t accumulator = 0;

	for(int i = 0; i < (size - positionOfChecksumFromEnd); i++) {
		accumulator += *ptr++;
	}

	accumulator = ~accumulator;

//	trace_printf("Calculated checksum 0x%02x for 0x%x\n", accumulator, cfg);
	return accumulator;
}

#pragma GCC diagnostic pop

/**
 * Set the idle LED in the idle task hook.
 */
extern "C" void vApplicationIdleHook(void) {
	Board::sharedInstance()->setLED(Board::kBoardLEDIdle, 1);
}

/**
 * Clear the idle LED in the tick hook.
 */
extern "C" void vApplicationTickHook(void) {
	Board::sharedInstance()->setLED(Board::kBoardLEDIdle, 0);
}

