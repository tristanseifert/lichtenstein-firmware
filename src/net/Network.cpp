/*
 * Network.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "Network.h"

#include "../board/Board.h"
#include "../clock/Clock.h"

#include "cmsis_device.h"
#include "diag/Trace.h"

// hardware config for EEPROM
#if HW == HW_MUSTARD

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

	// set up the MAC and PHY, respectively
	this->setUpMAC();
	this->setUpPHY();

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


    // try to probe for the EEPROM and read the MAC
//    this->_i2cScan();
    this->probeEthParamEEPROM();
}

/**
 * Scans the I2C bus to locate the address of the parameter EEPROM.
 */
void Network::probeEthParamEEPROM(void) {
	Board *b = Board::sharedInstance();
	b->configEEPROMRead(&this->macAddress, Network::ethParamMACOffset, 6);

	// debug
	trace_printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 this->macAddress[0], this->macAddress[1],
				 this->macAddress[2], this->macAddress[3],
				 this->macAddress[4], this->macAddress[5]);
}

/**
 * Sets up the Ethernet MAC internal to the chip, sets up the MAC filtering
 * and UDP/ICMP/TCP checksum offloading and configures the DMA engine and
 * various interrupts.
 */
void Network::setUpMAC(void) {

}

/**
 * Initializes the correct PHY based on the hardware.
 */
void Network::setUpPHY(void) {

}
