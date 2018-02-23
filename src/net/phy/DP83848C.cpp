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

// set to use interrupts instead of polling for link state
#define POLL_LINK_STATE						1

#if !POLL_LINK_STATE
#error "Interrupt-driven link state change notification is not supported"
#endif

namespace net::phy {

/**
 * Initializes the PHY. This sets up the LEDs, the (R)MII interface, and then
 * starts autonegotiation.
 */
DP83848C::DP83848C(Network *_net, EthMAC *_mac, bool _rmii, uint16_t _addr) : EthPHY(_net, _mac, _rmii, _addr) {
	// reset the PHY
	this->reset();

	this->toggleLEDsOnInit();

	// perform a self-test
	/*if(this->runBIST() == false) {
		LOG(S_ERROR, "PHY %d failed BIST", this->addr);
	}*/

	// configure registers
	this->setUpRegisters();

	// set up link state timer
#if POLL_LINK_STATE
	this->setUpLinkMonitor();

	// force a link state change to propagate
	this->checkForLinkStateChange();
#endif
}

/**
 * Initializes the PHY's registers.
 */
void DP83848C::setUpRegisters(void) {
	uint16_t temp;

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


	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return;
	}

	// configure BMCR: enable autonegotiation
	temp = 0;

	temp |= MDIO_REG_BMCR_AUTONEG; // enable autonegotiation

	if(this->writeRegister(MDIO_REG_BMCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write BMCR register");
		goto done;
	}


	// Configure ANAR: advertise PAUSE support, 100BASE-TX
	temp = 0;

	temp |= MDIO_REG_ANAR_PAUSE; // support for normal PAUSE frames
	temp |= MDIO_REG_ANAR_ASYM_PAUSE; // assymetric pause

	temp |= MDIO_REG_ANAR_100BASE_TX_FD; // 100BASE-TX full duplex
	temp |= MDIO_REG_ANAR_100BASE_TX_HD; // 100BASE-TX half duplex

	temp |= MDIO_REG_ANAR_10BASE_T_FD; // 10BASE-T full duplex
	temp |= MDIO_REG_ANAR_10BASE_T_HD; // 10BASE-T half duplex

	temp |= (MDIO_REG_ANAR_SELECTOR_ETH & MDIO_REG_ANAR_SELECTOR_MASK); // selector

	if(this->writeRegister(MDIO_REG_ANAR, temp) == false) {
		LOG(S_ERROR, "Couldn't write ANAR register");
		goto done;
	}


	// Configure ABER: do NOT support next page
	temp = 0;

	if(this->writeRegister(MDIO_REG_ANER, temp) == false) {
		LOG(S_ERROR, "Couldn't write ANER register");
		goto done;
	}


	// Configure ANNPTR: no desire for a new page
	temp = 0;

	if(this->writeRegister(MDIO_REG_ANNPTR, temp) == false) {
		LOG(S_ERROR, "Couldn't write ANNPTR register");
		goto done;
	}


	// Configure PCSR: standard 100Mbps operation
	temp = 0;

	if(this->writeRegister(MDIO_REG_PCSR, temp) == false) {
		LOG(S_ERROR, "Couldn't write PCSR register");
		goto done;
	}


	// Configure 10BTSCR: default squelch threshold
	temp = 0;

	temp |= (MDIO_REG_10BTSCR_SQUELCH_330MV & MDIO_REG_10BTSCR_SQUELCH_MASK);

	temp |= MDIO_REG_10BTSCR_LOOPBACK_DIS; // don't receive TX in half duplex

	if(this->writeRegister(MDIO_REG_10BTSCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write 10BTSCR register");
		goto done;
	}


	// Configure RBR: select MII/RMII, 14 bit clock tolerance
	temp = 0;

	if(this->rmii) {
		temp |= MDIO_REG_RBR_RMII;
		// TODO: should we use RMII version 1.0?
	}

	if(this->writeRegister(MDIO_REG_RBR, temp) == false) {
		LOG(S_ERROR, "Couldn't write RBR register");
		goto done;
	}


	// Configure PHYCR: enable auto-MDIX, PAUSE RX/TX, stretch LEDs, mode 0
	temp = 0;

	temp |= MDIO_REG_PHYCR_AUTO_MDIX; // enable Auto-MDIX

	temp |= MDIO_REG_PHYCR_PAUSE_RX; // receive PAUSE frames
	temp |= MDIO_REG_PHYCR_PAUSE_TX; // transmit PAUSE frames

	temp &= (uint16_t) ~(MDIO_REG_PHYCR_LED_CFG1); // LED mode 0
	temp |= MDIO_REG_PHYCR_LED_CFG0; // LED mode 0

	temp |= (this->addr & 0x1F); // keep the same address

	if(this->writeRegister(MDIO_REG_PHYCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write RBR register");
		goto done;
	}

done: ;
	// we've finished writing so end the transaction
	this->endMDIOTransaction();
}

/**
 * Executes the built-in self test (BIST) in the PHY and reports whether it
 * passes or not.
 *
 * This should be used to prevent further initialization steps from taking
 * place if the test fails.
 *
 * TODO: fix this. it's broken
 */
bool DP83848C::runBIST(void) {
	uint16_t temp;
	bool pass = false;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return false;
	}

	// how long to wait for the BIST bet to become set
	int timeout = 10000;

	// send the PHY control reg with the BIST bits set
	temp = 0;

//	temp |= MDIO_REG_PHYCR_PSR_15;
	temp |= MDIO_REG_PHYCR_BIST_START;
	temp |= (this->addr & 0x1F);

	if(this->writeRegister(MDIO_REG_PHYCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write PHYCR register");
		goto done;
	}

	// read the register until either the timeout elapsed or we pass
	do {
		if(this->readRegister(MDIO_REG_PHYCR, &temp) == false) {
			LOG(S_ERROR, "Couldn't read PHYCR register");
			goto done;
		}


		// test the BIST status bit
		if((temp & MDIO_REG_PHYCR_BIST_PASS) != 0) {
			LOG(S_DEBUG, "BIST_PASS set: 0x%04x", temp);

			// disable BIST by writing back just the PHY address
			temp = (this->addr & 0x1F);

			if(this->writeRegister(MDIO_REG_PHYCR, temp) == false) {
				LOG(S_ERROR, "Couldn't write PHYCR register");
				goto done;
			}

			// we passed, yay!
			pass = true;
		}
	} while(timeout--);

	// end the MDIO transaction
done: ;
	this->endMDIOTransaction();
	return pass;
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
	uint16_t temp = 0;

	// reset timeout
	int timeout = 100000;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return;
	}

	// Write to BMCR with only the reset bit set
	if(this->writeRegister(MDIO_REG_BMCR, MDIO_REG_BMCR_RESET) == false) {
		LOG(S_ERROR, "Couldn't write BMCR register");
		goto done;
	}

	// poll the reset bit
	do {
		// read the BMCR register
		if(this->readRegister(MDIO_REG_BMCR, &temp) == false) {
			goto done;
		}

		// if the reset bit is clear, the reset procedure completed
		if((temp & MDIO_REG_BMCR_RESET) == 0x0000) {
			goto done;
		}
	} while(timeout--);

	// if we reach down here, the timeout expired
	LOG(S_ERROR, "Reset timed out");

done: ;
	// finish up the transaction
	this->endMDIOTransaction();
}


/**
 * If the PHY is capable of loop-back mode, sets its state.
 */
void DP83848C::setLoopbackState(bool enabled) {
	uint16_t bmcr;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return;
	}

	// read the BMCR register
	if(this->readRegister(MDIO_REG_BMCR, &bmcr) == false) {
		LOG(S_ERROR, "Couldn't read BMCR register");
		goto done;
	}


	// set the loopback flag as needed
	if(enabled) {
		bmcr |= MDIO_REG_BMCR_LOOPBACK;
	} else {
		bmcr &= (uint16_t) ~(MDIO_REG_BMCR_LOOPBACK);
	}


	// write it back
	if(this->writeRegister(MDIO_REG_BMCR, bmcr) == false) {
		LOG(S_ERROR, "Couldn't write BMCR register");
		goto done;
	}

done: ;
	// assume success if we get down here
	this->endMDIOTransaction();
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
	uint16_t physts;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return false;
	}

	// read the PHYSTS register
	if(this->readRegister(MDIO_REG_PHYSTS, &physts) == false) {
		LOG(S_ERROR, "Couldn't read PHYSTS register");
		goto done;
	}

done: ;
	// assume success if we get down here
	this->endMDIOTransaction();

	// return the state of the full duplex flag
	return (physts & MDIO_REG_PHYSTS_FULL_DUPLEX) ? true : false;
}

/**
 * Returns the speed the PHY is operating at.
 */
net_link_speed_t DP83848C::getSpeed(void) {
	uint16_t physts;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return kLinkSpeedUnknown;
	}

	// read the PHYSTS register
	if(this->readRegister(MDIO_REG_PHYSTS, &physts) == false) {
		LOG(S_ERROR, "Couldn't read PHYSTS register");

		// end transaction
		this->endMDIOTransaction();
		return kLinkSpeedUnknown;
	} else {
		// end transaction
		this->endMDIOTransaction();

		// return the state of the 10Mbps flag
		return (physts & MDIO_REG_PHYSTS_10MBPS) ? kLinkSpeed10Mbps : kLinkSpeed100Mbps;
	}
}

/**
 * If the PHY is capable of detecting whether the network cable is a crossover
 * (MDIX), return the detected state.
 */
bool DP83848C::isMDIPairSwapped(void) {
	uint16_t physts;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return false;
	}

	// read the PHYSTS register
	if(this->readRegister(MDIO_REG_PHYSTS, &physts) == false) {
		LOG(S_ERROR, "Couldn't read PHYSTS register");
		goto done;
	}

done: ;
	// end transaction
	this->endMDIOTransaction();

	// return the state of the MDI swapped flag
	return (physts & MDIO_REG_PHYSTS_MDI_SWAPPED) ? true : false;
}


/**
 * Forces the PHY to perform link auto-negotiation.
 *
 * This is done by setting the restart autonegotiation bit in BMCR.
 */
void DP83848C::performAutonegotiation(void) {
	uint16_t bmcr;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return;
	}

	// read the BMCR register
	if(this->readRegister(MDIO_REG_BMCR, &bmcr) == false) {
		LOG(S_ERROR, "Couldn't read BMCR register");
		goto done;
	}


	// force autonegotiation to happen
	bmcr |= MDIO_REG_BMCR_AUTONEG; // make sure autonegotiation is enabled
	bmcr |= MDIO_REG_BMCR_RESTART_AUTONEG; // restart autonegotiation


	// write it back
	if(this->writeRegister(MDIO_REG_BMCR, bmcr) == false) {
		LOG(S_ERROR, "Couldn't write BMCR register");
		goto done;
	}

	// assume success if we get down here
done: ;
	// end transaction
	this->endMDIOTransaction();
}

/**
 * If the autonegotation is not desired, set the link speed and duplex mode
 * manually with this function.
 *
 * If these parameters could not be satisfied, false is returned.
 */
bool DP83848C::setLinkParams(net_link_speed_t speed, bool duplex) {
	uint16_t bmcr;

	bool linkStateSet = false;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return false;
	}

	// read the BMCR register
	if(this->readRegister(MDIO_REG_BMCR, &bmcr) == false) {
		LOG(S_ERROR, "Couldn't read BMCR register");
		goto done;
	}


	// disable autonegotiation
	bmcr &= (uint16_t) ~(MDIO_REG_BMCR_AUTONEG);

	// set speed
	if(speed == kLinkSpeed100Mbps) {
		bmcr |= MDIO_REG_BMCR_100Mbps;
	} else if(speed == kLinkSpeed10Mbps) {
		bmcr &= (uint16_t) ~(MDIO_REG_BMCR_100Mbps);
	}

	// set duplex mode
	if(duplex) {
		bmcr |= MDIO_REG_BMCR_FULL_DUPLEX;
	} else {
		bmcr &= (uint16_t) ~(MDIO_REG_BMCR_FULL_DUPLEX);
	}


	// write it back
	if(this->writeRegister(MDIO_REG_BMCR, bmcr) == false) {
		LOG(S_ERROR, "Couldn't write BMCR register");
		goto done;
	}

	// assume success
	linkStateSet = true;

done: ;
	// end transaction
	this->endMDIOTransaction();

	return linkStateSet;
}


/**
 * Sets the PHY's power state. When the PHY is in power-down mode, only its
 * registers may be accessed and Ethernet traffic WILL NOT be passed.
 */
void DP83848C::setPowerState(bool powerUp) {
	uint16_t bmcr;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return;
	}

	// read the BMCR register
	if(this->readRegister(MDIO_REG_BMCR, &bmcr) == false) {
		LOG(S_ERROR, "Couldn't read BMCR register");
		goto done;
	}


	// set power state mode
	if(powerUp) {
		bmcr &= (uint16_t) ~(MDIO_REG_BMCR_POWERDOWN);
	} else {
		bmcr |= MDIO_REG_BMCR_POWERDOWN;
	}


	// write it back
	if(this->writeRegister(MDIO_REG_BMCR, bmcr) == false) {
		LOG(S_ERROR, "Couldn't write BMCR register");
		goto done;
	}

	// if we get down here, assume the power status was set
done: ;
	// end transaction
	this->endMDIOTransaction();
}



/**
 * Reads the status register.
 */
uint16_t DP83848C::readStatus(void) {
	uint16_t bmsr;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return 0;
	}

	// attempt to read the BMSR register
	if(this->readRegister(MDIO_REG_BMSR, &bmsr) == false) {
		LOG(S_ERROR, "Error reading BMSR");

		bmsr = 0;
	}

	// return the value
	this->endMDIOTransaction();
	return bmsr;
}


/**
 * Toggles the LEDs on the Ethernet port on initialization.
 */
void DP83848C::toggleLEDsOnInit(void) {
	uint16_t temp = 0;
	const int timeout = 2500000;
	volatile int counter;

	// start the transaction
	if(this->startMDIOTransaction() == false) {
		return;
	}

	// set up the LEDs to be manually controlled
	temp |= MDIO_REG_LEDCR_SPD_MODE;
	temp |= MDIO_REG_LEDCR_LINK_MODE;
	temp |= MDIO_REG_LEDCR_ACT_MODE;

	if(this->writeRegister(MDIO_REG_LEDCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write LEDCR");
		goto done;
	}

	// set the speed LED
	temp |= MDIO_REG_LEDCR_SPD;

	if(this->writeRegister(MDIO_REG_LEDCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write LEDCR");
		goto done;
	}

	counter = timeout;
	while(counter--) {}


	// set the link LED
	temp |= MDIO_REG_LEDCR_LINK;

	if(this->writeRegister(MDIO_REG_LEDCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write LEDCR");
		goto done;
	}

	counter = timeout;
	while(counter--) {}


	// set the activity LED
	temp |= MDIO_REG_LEDCR_ACT;

	if(this->writeRegister(MDIO_REG_LEDCR, temp) == false) {
		LOG(S_ERROR, "Couldn't write LEDCR");
		goto done;
	}

	counter = timeout;
	while(counter--) {}

	// enable automatic LED control
	if(this->writeRegister(MDIO_REG_LEDCR, 0x0000) == false) {
		LOG(S_ERROR, "Couldn't write LEDCR");
		goto done;
	}

	// ends the MDIO transaction
done: ;
	this->endMDIOTransaction();
}



/**
 * Reads the contents of a particular register.
 *
 * @return true if the register was read, false otherwise.
 */
bool DP83848C::readRegister(uint16_t reg, uint16_t *value) {
	int err;

	// read the register
	err = this->mac->mdioRead(this->addr, reg);

	// check for read errors
	if(err < 0) {
		LOG(S_ERROR, "Couldn't read register 0x%02x", reg);
		return false;
	}

	// good read, write it into the buffer provided
	*value = (uint16_t) (err & 0xFFFF);

	return true;
}

/**
 * Writes to a particular register.
 *
 * @return true if the register was written, false otherwise.
 */
bool DP83848C::writeRegister(uint16_t reg, uint16_t value) {
	int err;

	// perform write operation
	err = this->mac->mdioWrite(this->addr, reg, value);

	// check for write errors
	if(err < 0) {
		LOG(S_ERROR, "Couldn't write BMCR register");
		return false;
	}

	// assume successful write
	return true;
}

} /* namespace net */
