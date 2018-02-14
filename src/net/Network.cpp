/*
 * Network.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "Network.h"

#include "../clock/Clock.h"

#include "cmsis_device.h"
#include "diag/Trace.h"

// hardware config for EEPROM
#if HW == HW_MUSTARD
// I2C peripheral to use
#define MAC_EEPROM_I2C					I2C1
#define MAC_EEPROM_I2C_CLOCK				RCC_APB1Periph_I2C1

// what speed to run the I2C bus at, in Hz
#define	MAC_EEPROM_I2C_SPEED				(400 * 1000)

// pins on which SDA and SCL are connected
#define MAC_EEPROM_I2C_SDA_PORT			GPIOB
#define MAC_EEPROM_I2C_SDA_PIN			GPIO_Pin_9
#define MAC_EEPROM_I2C_SDA_GPIO_CLOCK	RCC_APB2Periph_GPIOB
#define MAC_EEPROM_I2C_SCL_PORT			GPIOB
#define MAC_EEPROM_I2C_SCL_PIN			GPIO_Pin_8
#define MAC_EEPROM_I2C_SCL_GPIO_CLOCK	RCC_APB2Periph_GPIOB

// if I2C is an alternate function, set it here
#define MAC_EEPROM_I2C_REMAP				1
#define MAC_EEPROM_I2C_REMAP_ARG			GPIO_Remap_I2C1

#endif

static Network *gNetwork = nullptr;

/**
 * Allocates the shared clock.
 */
void Network::init(void) {
	if(!gNetwork) {
		gNetwork = new Network();
	}
}

/**
 * Returns the shared clock instance.
 */
Network *Network::sharedInstance() noexcept {
	return gNetwork;
}

/**
 * Sets up the Ethernet hardware and the TCP/IP stack.
 */
Network::Network() {
	// read the MAC address
	this->setUpEthParamEEPROM();

	// set up the stack

	// when the stack is set up, start network services
	this->startNetServices();
}

/**
 * Starts various network services, such as DHCP, the NTP server, and so forth.
 */
void Network::startNetServices(void) {
	// start NTP client so we can sync time
	Clock::sharedInstance()->startNTPClient();
}

/**
 * Closes the TCP/IP stack and turns off the Ethernet link.
 */
Network::~Network() {

}


/**
 * Sets up the I2C peripheral that's connected to the EEPROM that contains the
 * Ethernet MAC address. This will typically be a 24AA025E48 or similar, with
 * the 6-byte MAC at address $80.
 *
 * This doesn't _have_ to be one of those types of EEPROMs that come from the
 * factory programmed with a MAC, but it's the most convenient option since
 * we don't implement writing to the EEPROM.
 */
void Network::setUpEthParamEEPROM(void) {
	// turn in the clock for the AFIO module
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// Set up GPIOs for I2C
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	// set up SDA line
	RCC_APB2PeriphClockCmd(MAC_EEPROM_I2C_SDA_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = MAC_EEPROM_I2C_SDA_PIN;
	GPIO_Init(MAC_EEPROM_I2C_SDA_PORT, &gpio);

	// set up SCL line
	RCC_APB2PeriphClockCmd(MAC_EEPROM_I2C_SCL_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = MAC_EEPROM_I2C_SCL_PIN;
	GPIO_Init(MAC_EEPROM_I2C_SCL_PORT, &gpio);

	// apply pin remap if needed
#if MAC_EEPROM_I2C_REMAP
	GPIO_PinRemapConfig(MAC_EEPROM_I2C_REMAP_ARG, ENABLE);
#endif


	// enable clocks for the I2C module and reset it
    RCC_APB1PeriphClockCmd(MAC_EEPROM_I2C_CLOCK, ENABLE);

    RCC_APB1PeriphResetCmd(MAC_EEPROM_I2C_CLOCK, ENABLE);
    RCC_APB1PeriphResetCmd(MAC_EEPROM_I2C_CLOCK, DISABLE);

    	// initialize the I2C module
	I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c);

	i2c.I2C_ClockSpeed = MAC_EEPROM_I2C_SPEED;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00;
	i2c.I2C_Ack = I2C_Ack_Disable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(MAC_EEPROM_I2C, &i2c);

	// enable the I2C
    I2C_Cmd(MAC_EEPROM_I2C, ENABLE);

    // try to probe for the EEPROM and read the MAC
//    this->_i2cScan();
    this->probeEthParamEEPROM();
}

/**
 * Scans the I2C bus to locate the address of the parameter EEPROM.
 */
void Network::probeEthParamEEPROM(void) {
//	this->writeEthParamEEPROM();

	trace_printf("Reading MAC address from I2C address 0x%02x\n", Network::ethParamI2CAddress);

	// generate a START, for writing, and send the address
	if(this->i2cStart(Network::ethParamI2CAddress, false) == 1) {
		trace_puts("Timeout starting I2C transmission for address!\n");
		return;
	}

	this->i2cWriteByte(Network::ethParamMACOffset);

	this->i2cStop();

	// generate another START, for reading, and read 6 bytes
	if(this->i2cStart(Network::ethParamI2CAddress, true) == 1) {
		trace_puts("Timeout starting I2C transmission for data!\n");
		return;
	}

	for(int i = 0; i < 5; i++) {
		this->macAddress[i] = this->i2cReadByte(true);
	}

	this->macAddress[5] = this->i2cReadByte(false);

	// debug
	trace_printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 this->macAddress[0], this->macAddress[1],
				 this->macAddress[2], this->macAddress[3],
				 this->macAddress[4], this->macAddress[5]);

	// send a STOP condition
	this->i2cStop();
}

/**
 * Writes a MAC address to the EEPROM.
 */
void Network::writeEthParamEEPROM(void) {
	trace_puts("WRITING MAC ADDRESS TO EEPROM!");

	// write a MAC address
	const uint8_t testMac[] = {0xde, 0xad, 0xbe, 0xef, 0x14, 0x20};
	for(int i = 0; i < 6; i++) {
		this->macAddress[i] = testMac[i];
	}

	// generate a START, for writing, and send the address and data
	if(this->i2cStart(Network::ethParamI2CAddress, false) == 1) {
		trace_puts("Timeout starting I2C transmission for MAC write!\n");
		return;
	}

	this->i2cWriteByte(Network::ethParamMACOffset);

	for(int i = 0; i < 6; i++) {
		this->i2cWriteByte(this->macAddress[i]);
	}
	// send stop condition
	this->i2cStop();
}

/**
 * Scans the I2C bus for devices.
 */
void Network::_i2cScan(void) {
	trace_puts("Scanning I2C bus:\n");

	for(uint8_t address = 0x00; address <= 0x7F; address++) {
		if(this->i2cStart(address, false) == 0) {
			trace_printf("\tFound device at address 0x%02x\n", address);
			this->i2cStop();
		}
	}
}

/**
 * Waits for the I2C interface to be idle.
 */
void Network::i2cWaitForIdle(void) {
	while(I2C_GetFlagStatus(MAC_EEPROM_I2C, I2C_FLAG_BUSY)) {
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
int Network::i2cStart(uint8_t address, bool read, int timeout) {
	// sanity check timeout
	if(timeout <= 0) {
		timeout = 20000;
	}

	uint8_t direction = read ? I2C_Direction_Receiver : I2C_Direction_Transmitter;

	// wait until I2C is not busy anymore
	// TODO: we really should do this
//	this->i2cWaitForIdle();

	// generate START condition
	I2C_GenerateSTART(MAC_EEPROM_I2C, ENABLE);

	// wait for I2C EV5 (slave has acknowledged start condition)
	while(!I2C_CheckEvent(MAC_EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT)) {
		// decrement timeout
		if(timeout-- == 0) {
			// generate a stop condition and return if it hits zero
			this->i2cStop();

			return 1;
		}
	}

//	trace_printf("Acknowledged in %i tries\n", 50000 - timeout);

	// send slave address
	I2C_Send7bitAddress(MAC_EEPROM_I2C, (uint8_t) (address <<  1), direction);

	/*
	 * wait for I2C1 EV6, check if the slave either acknowledged the master as
	 * a transmitter, or as a receiver
	 */
	if(direction == I2C_Direction_Transmitter) {
		while(!I2C_CheckEvent(MAC_EEPROM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			// do stuff here?
		}
	} else if(direction == I2C_Direction_Receiver) {
		while(!I2C_CheckEvent(MAC_EEPROM_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			// do stuff here?
		}
	}

	return 0;
}

/**
 * Generates a STOP condition on the I2C bus.
 */
void Network::i2cStop(void) {
	I2C_GenerateSTOP(MAC_EEPROM_I2C, ENABLE);
}

/**
 * Writes a single byte to the I2C bus. This assumes that the address has been
 * sent already.
 */
void Network::i2cWriteByte(uint8_t data) {
	I2C_SendData(MAC_EEPROM_I2C, data);

	// wait for the byte to be transmitted
	while(!I2C_CheckEvent(MAC_EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {

	}
}

/**
 * Reads a single byte from the device. If ack is true, an acknowledgement is
 * sent after the read, otherwise none is sent.
 */
uint8_t Network::i2cReadByte(bool ack) {
	// set the acknowledge config
	if(ack) {
		I2C_AcknowledgeConfig(MAC_EEPROM_I2C, ENABLE);
	} else {
		I2C_AcknowledgeConfig(MAC_EEPROM_I2C, DISABLE);
		I2C_GenerateSTOP(MAC_EEPROM_I2C, ENABLE);
	}

	// wait to receive a byte
	while(!I2C_CheckEvent(MAC_EEPROM_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		// do stuff?
	}

	// read the byte
	uint8_t data = I2C_ReceiveData(MAC_EEPROM_I2C);
	return data;
}
