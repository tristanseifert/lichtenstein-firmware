/*
 * Board.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#define LOG_MODULE "BRD"

#include "Board.h"

#include "LichtensteinApp.h"

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
#define TEST_SW0_PORT					GPIOE
#define TEST_SW0_PIN						GPIO_Pin_10
#define TEST_SW0_GPIO_CLOCK				RCC_APB2Periph_GPIOE

#define TEST_SW0_GPIO_EXTI				EXTI15_10_IRQHandler
#define TEST_SW0_GPIO_EXTI_PORT			GPIO_PortSourceGPIOE
#define TEST_SW0_GPIO_EXTI_PIN			GPIO_PinSource10
#define TEST_SW0_GPIO_EXTI_LINE			EXTI_Line10
#define TEST_SW0_GPIO_EXTI_NVIC_IRQ		EXTI15_10_IRQn
#define TEST_SW0_GPIO_EXTI_PRIORITY		(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)

// TEST_SW1
#define TEST_SW1_PORT					GPIOE
#define TEST_SW1_PIN						GPIO_Pin_11
#define TEST_SW1_GPIO_CLOCK				RCC_APB2Periph_GPIOE

#define TEST_SW1_GPIO_EXTI				EXTI2_IRQHandler
#define TEST_SW1_GPIO_EXTI_PORT			GPIO_PortSourceGPIOE
#define TEST_SW1_GPIO_EXTI_PIN			GPIO_PinSource11
#define TEST_SW1_GPIO_EXTI_LINE			EXTI_Line11
#define TEST_SW1_GPIO_EXTI_NVIC_IRQ		EXTI15_10_IRQn
#define TEST_SW1_GPIO_EXTI_PRIORITY		(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)

// when set, the EXTI is shared among both test switch inputs
#define TEST_SW_GPIO_EXTI_SHARED			(TEST_SW0_GPIO_EXTI_NVIC_IRQ == TEST_SW1_GPIO_EXTI_NVIC_IRQ)

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
	taskENTER_CRITICAL();

	if(!gBoard) {
		gBoard = new Board();
	}

	taskEXIT_CRITICAL();
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

	// set up watchdog
	this->initIWDG();
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
 * Gets the GPIO and pin for the specified LED.
 */
void Board::getGPIOForLED(board_led_t led, GPIO_TypeDef **port, uint16_t *pin) {
	switch(led) {
		case kBoardLEDIdle:
			*port = STATUS0_PORT;
			*pin = STATUS0_PIN;
			break;

		case kBoardLEDIPAct:
			*port = STATUS1_PORT;
			*pin = STATUS1_PIN;
			break;

		case kBoardLEDOutputAct:
			*port = STATUS2_PORT;
			*pin = STATUS2_PIN;
			break;

		case kBoardLEDError:
			*port = STATUS3_PORT;
			*pin = STATUS3_PIN;
			break;
	}
}

/**
 * Sets the state of the status indicators. Each LED on the board has an index:
 * - 0: idle
 * - 1: IP Act
 * - 2: LED out active
 * - 3: Error
 */
void Board::setLED(board_led_t led, bool set) {
	// get the GPIO pin
	GPIO_TypeDef *port = nullptr;
	uint16_t pin = 0;

	this->getGPIOForLED(led, &port, &pin);

	// set value
	GPIO_WriteBit(port, pin, (set ? Bit_SET : Bit_RESET));
}

/**
 * Toggles an existing LED.
 */
void Board::toggleLED(board_led_t led) {
	// get the GPIO pin
	GPIO_TypeDef *port = nullptr;
	uint16_t pin = 0;

	this->getGPIOForLED(led, &port, &pin);

	// invert the state
	int state = (GPIO_ReadOutputDataBit(port, pin) == Bit_RESET);
	GPIO_WriteBit(port, pin, (state == 1 ? Bit_SET : Bit_RESET));
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

	// configure the SW0 EXTI
	exti.EXTI_Line = TEST_SW0_GPIO_EXTI_LINE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	// configure the SW1 EXTI
	exti.EXTI_Line = TEST_SW1_GPIO_EXTI_LINE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	// enable SW0 EXTI interrupt
	nvic.NVIC_IRQChannel = TEST_SW0_GPIO_EXTI_NVIC_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = TEST_SW0_GPIO_EXTI_PRIORITY;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	// enable SW1 EXTI interrupt
#if TEST_SW_GPIO_EXTI_SHARED == 0
	nvic.NVIC_IRQChannel = TEST_SW1_GPIO_EXTI_NVIC_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = TEST_SW1_GPIO_EXTI_PRIORITY;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
#endif
}

/**
 * Called from the IRQ handler for the test switches. This will read out the
 * value of both switches, and notify other processes.
 */
void Board::testSwIRQ(void) {
	bool sw0 = (GPIO_ReadInputDataBit(TEST_SW0_PORT, TEST_SW0_PIN) == Bit_SET);
	bool sw1 = (GPIO_ReadInputDataBit(TEST_SW1_PORT, TEST_SW1_PIN) == Bit_SET);

	this->testSwState = (uint8_t) ((sw0 ? 0x01 : 0x00) | (sw1 ? 0x02 : 0x00));

	// TODO: handle this
}

/**
 * Handles the IRQ for the TEST_SW0 external interrupt.
 */
extern "C" void TEST_SW0_GPIO_EXTI(void) {
#if TEST_SW_GPIO_EXTI_SHARED
	// check that the correct lines changed
	if(EXTI_GetITStatus(TEST_SW0_GPIO_EXTI_LINE) != RESET ||
	   EXTI_GetITStatus(TEST_SW1_GPIO_EXTI_LINE) != RESET) {
		// call ISR handler
		Board::TestSwIRQNotify();
	}

	// acknowledge interrupt
	EXTI_ClearITPendingBit(TEST_SW0_GPIO_EXTI_LINE);
	EXTI_ClearITPendingBit(TEST_SW1_GPIO_EXTI_LINE);
#else
	// check that the correct lines changed
	if(EXTI_GetITStatus(TEST_SW0_GPIO_EXTI_LINE) != RESET) {
		// call ISR handler
		Board::TestSwIRQNotify();
	}

	// acknowledge interrupt
	EXTI_ClearITPendingBit(TEST_SW0_GPIO_EXTI_LINE);
#endif
}

/**
 * Handles the IRQ for the TEST_SW1 external interrupt, if it is different from
 * the one used by the first switch.
 */
#if !TEST_SW_GPIO_EXTI_SHARED
extern "C" void TEST_SW1_GPIO_EXTI(void) {
	// check that the correct line changed
	if(EXTI_GetITStatus(TEST_SW1_GPIO_EXTI_LINE) != RESET) {
		// call ISR handler
		Board::TestSwIRQNotify();
	}

	// acknowledge interrupt
	EXTI_ClearITPendingBit(TEST_SW1_GPIO_EXTI_LINE);
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

    // create the semaphore
    this->i2cMutex = xSemaphoreCreateMutex();

    this->i2cResetFix();

    // TEST: write config info
//    this->_testWriteBoardConfig();

    // read out the config info and verify it
    memset(&this->config, 0, sizeof(board_config_t));
    this->configEEPROMRead(&this->config, Board::configEEPROMHWInfoOffset, sizeof(board_config_t));

    uint8_t checksum = Board::calculateConfigChecksum(&this->config);

    if(checksum != this->config.checksum) {
    		LOG(S_ERROR, "mismatch in config checksum; computed 0x%02x, got 0x%02x", checksum, this->config.checksum);
    }

    if(this->config.version != BOARD_CONFIG_VERSION) {
    		LOG(S_ERROR, "invalid config version %u", this->config.version);
	}

    if(this->config.magic != BOARD_CONFIG_MAGIC) {
    		LOG(S_ERROR, "invalid magic %u", this->config.magic);
	}

    LOG(S_INFO, "running on hw 0x%02x, version 0x%02x.%02x",
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
	volatile int counter = timeout;

	while(!I2C_CheckEvent(CFG_EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
		// decrement timeout
		if(counter-- == 0) {
			// generate a stop condition and return if it hits zero
			this->i2cStop();

			return 1;
		}
	}

//	LOG(S_DEBUG, "Acknowledged in %u tries", 50000 - timeout);

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
	volatile int counter = timeout;

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
			LOG(S_ERROR, "READ ERROR: timeout reading byte");
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
	LOG(S_WARN, "WRITING BOARD CONFIG TO EEPROM!");

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

	// get lock enter critical section
	xSemaphoreTake(this->i2cMutex, portMAX_DELAY);

	taskENTER_CRITICAL();


	// send the address
	if(this->i2cStart(Board::configEEPROMI2CAddress, false) == 1) {
		LOG(S_ERROR, "Timeout starting I2C transaction for address write!");

		// return mutex
		xSemaphoreGive(this->i2cMutex);

		// leave critical section
		taskEXIT_CRITICAL();

		return;
	}

	this->i2cWriteByte(address);

	this->i2cStop();

	// read the number of bytes the user wants
	if(this->i2cStart(Board::configEEPROMI2CAddress, true) == 1) {
		LOG(S_ERROR, "Timeout starting I2C transaction for data read!");

		// return mutex
		xSemaphoreGive(this->i2cMutex);

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

	// return mutex and exit critical section
	xSemaphoreGive(this->i2cMutex);
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

	// get lock and enter critical section
	xSemaphoreTake(this->i2cMutex, portMAX_DELAY);

	taskENTER_CRITICAL();

#if DEBUG_I2C_WRITES
	LOG(S_DEBUG, "** starting write to 0x%02x, size 0x%02x", address, numBytes);
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
	LOG(S_DEBUG, "first page is 0x%02x, writing to offset 0x%02x, address 0x%02x, (%u bytes)", firstPage, offsetIntoPage, writeAddress, bytesToWrite);
#endif

	if(this->i2cStart(Board::configEEPROMI2CAddress, false, Board::configEEPROMWriteTimeout) == 1) {
		LOG(S_ERROR, "Timeout starting I2C transmission for write");

		// return mutex
		xSemaphoreGive(this->i2cMutex);

		// leave critical section
		taskEXIT_CRITICAL();

		return;
	}

	this->i2cWriteByte(writeAddress);

	// send data
	for(int i = 0; i < bytesToWrite; i++) {
		this->i2cWriteByte(*ptr++);

#if DEBUG_I2C_WRITES
		LOG(S_DEBUG, "#");
#endif
		bytesWritten++; writeAddress++;
	}

#if DEBUG_I2C_WRITES
	LOG(S_DEBUG, "sending stop condition");
#endif

	// have we got any more bytes to write?
	if(bytesWritten < numBytes) {
		int bytesLeftToWrite = numBytes - bytesWritten;

#if DEBUG_I2C_WRITES
		LOG(S_DEBUG, "remaining bytes to write: %u, starting at 0x%02x", bytesLeftToWrite, writeAddress);
#endif

		for(int i = 0, j = bytesLeftToWrite; i < j; i++) {
			if((writeAddress & Board::configEEPROMPageSizeMask) == 0) {
#if DEBUG_I2C_WRITES
				LOG(S_DEBUG, "starting page write at 0x%02x for %u bytes", writeAddress, bytesLeftToWrite & Board::configEEPROMPageSizeMask);
#endif

				// send address
				if(this->i2cStart(Board::configEEPROMI2CAddress, false, Board::configEEPROMWriteTimeout) == 1) {
					LOG(S_ERROR, "Timeout starting I2C transmission for write");

					// return mutex
					xSemaphoreGive(this->i2cMutex);

					// leave critical section
					taskEXIT_CRITICAL();

					return;
				}

				this->i2cWriteByte(writeAddress);
			}

			// write byte
			this->i2cWriteByte(*ptr++);

#if DEBUG_I2C_WRITES
			LOG(S_DEBUG, "*");
#endif
			bytesWritten++; writeAddress++;

			// have we reached the end of the page?
			if((writeAddress & Board::configEEPROMPageSizeMask) == 0) {
#if DEBUG_I2C_WRITES
				LOG(S_DEBUG, "sending stop condition after writing %u bytes, have %u bytes left", Board::configEEPROMPageSize, numBytes - bytesWritten);
#endif

				if((numBytes - bytesWritten) != 0) {
					this->i2cStop();
				}
			}
		}

		// send a stop condition again in case we haven't already
#if DEBUG_I2C_WRITES
		LOG(S_DEBUG, "sending stop condition");
#endif

		this->i2cStop();
	}

#if DEBUG_I2C_WRITES
	LOG(S_DEBUG, "done writing");
#endif

	// return mutex and exit critical section
	xSemaphoreGive(this->i2cMutex);

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

//	LOG(S_DEBUG, "Size of board_config_t: %u, checksum at byte %u from end", size, positionOfChecksumFromEnd);

	// calculate the checksum over the size
	uint8_t accumulator = 0;

	for(int i = 0; i < (size - positionOfChecksumFromEnd); i++) {
		accumulator += *ptr++;
	}

	accumulator = ~accumulator;

//	LOG(S_DEBUG, "Calculated checksum 0x%02x for 0x%x", accumulator, cfg);
	return accumulator;
}

#pragma GCC diagnostic pop



/**
 * Sets up the watchdog. This is configured to expire after 10 FreeRTOS ticks,
 * and is reset within the idle task handler.
 */
void Board::initIWDG(void) {
	// unlock writing to watchdog registers
	IWDG->KR = 0x5555;

	// use /32 prescaler (0.8ms per tick)
	IWDG->PR = 0b011;
	IWDG->RLR = (13 * 10); // 13 per FreeRTOS tick
}

/**
 * Set the idle LED in the idle task hook.
 */
extern "C" void vApplicationIdleHook(void) {
	// set idle LED
	Board::sharedInstance()->setLED(Board::kBoardLEDIdle, true);

	// enter sleep mode
	__asm volatile("dsb"); // ensure there's no outstanding memory accesses
	__asm volatile("wfi"); // wait for interrupt
	__asm volatile("isb"); // ensure wfi is executed in order
}

/**
 * Clear the idle LED in the tick hook.
 */
extern "C" void vApplicationTickHook(void) {
	// start watchdog if needed
	static bool hasEnabledWatchdog = false;

	if(!hasEnabledWatchdog) {
		LOG(S_INFO, "Enabling watchdog for 10 tick timeout");

		// start the watchdog
		IWDG->KR = 0xAAAA;
		IWDG->KR = 0xCCCC;
		IWDG->KR = 0xAAAA;

		hasEnabledWatchdog = true;
	}

	// reset watchdog
	IWDG->KR = 0xAAAA;

	// clear idle LED
	Board::sharedInstance()->setLED(Board::kBoardLEDIdle, false);
}

