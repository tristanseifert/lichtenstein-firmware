/*
 * EthPHY.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: tristan
 */
#define LOG_MODULE "PHY"

#include "EthPHY.h"
#include "EthMAC.h"

#include "Network.h"

#include "phy/DP83848C.h"

#include <LichtensteinApp.h>

namespace net {

/**
 * Base constructor: this doesn't do anything.
 */
EthPHY::EthPHY(Network *_net, EthMAC *_mac, bool _rmii, uint16_t _addr) : net(_net), mac(_mac), rmii(_rmii), addr(_addr)  {
	// allocate the MDIO lock
	this->mdioLock = xSemaphoreCreateBinary();
	xSemaphoreGive(this->mdioLock);

}

/**
 * The constructor doesn't do anything either.
 */
EthPHY::~EthPHY() {
	// kill the link monitor if needed
	if(this->linkMon) {
		xTimerStop(this->linkMon, portMAX_DELAY);
		xTimerDelete(this->linkMon, portMAX_DELAY);
	}

	// delete the MDIO lock
	if(this->mdioLock) {
		vSemaphoreDelete(this->mdioLock);
	}
}



/**
 * Factory method to construct a PHY from a PHY ID. If we can't handle a
 * particular PHY, nullptr is returned.
 */
EthPHY *EthPHY::phyForId(Network *net, uint32_t id, uint16_t address, bool useRMII) {
	switch(id) {
		// TI DP83848CVV
		case 0x20005C90:
			return new phy::DP83848C(net, net->mac, useRMII, address);
			break;

		// no suitable PHY could be created
		default:
			LOG(S_ERROR, "Couldn't instantiate PHY with ID %08X", id);
			return nullptr;
	}
}



/**
 * Callback for the link monitor timer.
 */
void EthPHYLinkMonitorTimerCallback(TimerHandle_t timer) {
	void *ctx = pvTimerGetTimerID(timer);
	EthPHY *phy = static_cast<EthPHY *>(ctx);

	phy->checkForLinkStateChange();
}

/**
 * Sets up the link monitor timer. This polls the link status register and
 * calls into the network stack when it's changed.
 */
void EthPHY::setUpLinkMonitor(void) {
	// create the FPS timer
	this->linkMon = xTimerCreate("LinkMon",
			pdMS_TO_TICKS(EthPHY::linkMonitorTimerInterval), pdTRUE, this,
			EthPHYLinkMonitorTimerCallback);

	// also start the timer
	xTimerStart(this->linkMon, portMAX_DELAY);
}

/**
 * This is called from the link monitor timer; checks the state of the link and
 * checks if it's changed. Call into the network stack if it has.
 *
 * By definition, this will never be called from an ISR.
 */
void EthPHY::checkForLinkStateChange(void) {
	bool linkState = this->isLinkUp();

	// if it differs, call into the stack
	if(linkState != this->lastLinkState) {
		this->net->_phyLinkStateChange(linkState, false);

		this->lastLinkState = linkState;
	}
}



/**
 * Attempts to take the MDIO lock, waiting until the specified timeout is
 * expired.
 *
 * @note This function is _not_ ISR safe.
 */
bool EthPHY::startMDIOTransaction(int timeout) {
	// limit timeout
	if(timeout == -1) {
		timeout = portMAX_DELAY;
	}

	// attempt to take the lock
	if(xSemaphoreTake(this->mdioLock, timeout) == pdTRUE) {
		return true;
	} else {
		LOG(S_ERROR, "Couldn't take MDIO lock after %d ticks", timeout);

		return false;
	}
}

/**
 * Ends the MDIO transaction.
 *
 * @note This function is _not_ ISR safe.
 */
bool EthPHY::endMDIOTransaction(void) {
	xSemaphoreGive(this->mdioLock);

	// nothing can really go wrong here
	return true;
}

} /* namespace net */
