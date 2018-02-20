/*
 * DP83848C.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: tristan
 */
#define LOG_MODULE "PHY"

#include "DP83848C.h"
#include "DP83848CPrivate.h"

#include "../EthMAC.h"

#include <LichtensteinApp.h>

namespace net {

/**
 * Initializes the PHY. This sets up the LEDs, the (R)MII interface, and then
 * starts autonegotiation.
 */
DP83848C::DP83848C(Network *_net, EthMAC *_mac, bool _rmii, uint16_t _addr) : EthPHY(_net, _mac, _rmii, _addr) {
	// reset the PHY
	this->reset();

	// perform a self-test
	if(this->runBIST() == false) {
		LOG(S_ERROR, "PHY %d failed BIST", this->addr);
	}

	// configure registers
	this->setUpRegisters();

}

/**
 * Initializes the PHY's registers.
 */
void DP83848C::setUpRegisters(void) {
	// get PHY status
	uint16_t status = this->readStatus();
	LOG(S_DEBUG, "status: 0x%04x", status);

	if(status & MDIO_REG_BMSR_100BASETX_FD) {
		LOG(S_DEBUG, "PHY is 100BASE-TX full duplex capable");
	} if(status & MDIO_REG_BMSR_100BASETX_HD) {
		LOG(S_DEBUG, "PHY is 100BASE-TX half duplex capable");
	} if(status & MDIO_REG_BMSR_10BASET_FD) {
		LOG(S_DEBUG, "PHY is 10BASE-T full duplex capable");
	} if(status & MDIO_REG_BMSR_10BASET_HD) {
		LOG(S_DEBUG, "PHY is 10BASE-T half duplex capable");
	}

	// configure LEDs
}

/**
 * Executes the built-in self test (BIST) in the PHY and reports whether it
 * passes or not.
 *
 * This should be used to prevent further initialization steps from taking
 * place if the test fails.
 */
bool DP83848C::runBIST(void) {
	uint16_t temp;

	// how long to wait for the BIST bet to become set
	int timeout = 10000;

	// send the PHY control reg with the BIST bits set
	temp = 0;

//	temp |= MDIO_REG_PHYCR_PSR_15;
	temp |= MDIO_REG_PHYCR_BIST_START;
	temp |= (this->addr & 0x1F);

	if(this->mac->mdioWrite(this->addr, MDIO_REG_PHYCR, temp) != 0) {
		LOG(S_ERROR, "Couldn't write PHYCR register");
		return false;
	}

	// read the register until either the timeout elapsed or we pass
	do {
		int err = this->mac->mdioRead(this->addr, MDIO_REG_PHYCR);

		// check for read errors
		if(err < 0) {
			LOG(S_ERROR, "Couldn't read PHYCR register");
			return false;
		}

		// test the BIST status bit
		if((err & MDIO_REG_PHYCR_BIST_PASS) != 0) {
			LOG(S_DEBUG, "BIST_PASS set: 0x%04x", err);

			// disable BIST by writing back just the PHY address
			temp = (this->addr & 0x1F);

			if(this->mac->mdioWrite(this->addr, MDIO_REG_PHYCR, temp) != 0) {
				LOG(S_ERROR, "Couldn't write PHYCR register");
				return false;
			}

			// we passed, yay!
			return true;
		}
	} while(timeout--);

	// if we get down here, the timeout expired
	return false;
}



/**
 * When de-allocating, just reset the PHY.
 */
DP83848C::~DP83848C() {
	this->reset();
}



/**
 * Resets the PHY.
 */
void DP83848C::reset(void) {
	int temp;

	// Write to BMCR with only the reset bit set
	this->mac->mdioWrite(this->addr, MDIO_REG_BMCR, MDIO_REG_BMCR_RESET);

	// poll the reset bit
	int timeout = 20000;

	do {
		temp = this->mac->mdioRead(this->addr, MDIO_REG_BMCR);

		if(temp < 0) {
			LOG(S_ERROR, "Error reading BMCR");
			return;
		}

		// if the reset bit is clear, the reset procedure completed
		if((temp & MDIO_REG_BMCR_RESET) == 0x0000) {
			return;
		}
	} while(timeout--);

	// if we reach down here, the timeout expired
	LOG(S_ERROR, "Reset timed out");
}


/**
 * If the PHY is capable of loop-back mode, sets its state.
 */
void DP83848C::setLoopbackState(bool enabled) {

}


/**
 * Determines whether there is a link up on the PHY.
 *
 * This is done by just checking if the link valid bit is set in the status
 * register.
 */
bool DP83848C::isLinkUp(void) {
	return (this->readStatus() & MDIO_REG_BMSR_LINK_UP) ? true : false;
}

/**
 * Determines whether the PHY is operating in full duplex mode.
 */
bool DP83848C::isFullDuplex(void) {

}

/**
 * Returns the speed the PHY is operating at.
 */
net_link_speed_t DP83848C::getSpeed(void) {

}

/**
 * If the PHY is capable of detecting whether the network cable is a crossover
 * (MDIX), return the detected state.
 */
bool DP83848C::isMDIPairSwapped(void) {

}


/**
 * Forces the PHY to perform link auto-negotiation.
 */
void DP83848C::performAutonegotiation(void) {

}

/**
 * If the autonegotation is not desired, set the link speed and duplex mode
 * manually with this function.
 *
 * If these parameters could not be satisfied, false is returned.
 */
bool DP83848C::setLinkParams(net_link_speed_t speed, bool duplex) {

}


/**
 * Sets the PHY's power state. When the PHY is in power-down mode, only its
 * registers may be accessed and Ethernet traffic WILL NOT be passed.
 */
void DP83848C::setPowerState(bool powerUp) {

}



/**
 * Reads the status register.
 */
uint16_t DP83848C::readStatus(void) {
	int reg = this->mac->mdioRead(this->addr, MDIO_REG_BMSR);

	// check for error
	if(reg < 0) {
		LOG(S_ERROR, "Error reading BMSR");
		return 0;
	}

	// return the register
	return (uint16_t) (reg & 0xFFFF);
}

} /* namespace net */
