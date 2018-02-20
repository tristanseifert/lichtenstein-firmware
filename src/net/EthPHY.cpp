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

}

/**
 * The constructor doesn't do anything either.
 */
EthPHY::~EthPHY() {

}



/**
 * Factory method to construct a PHY from a PHY ID. If we can't handle a
 * particular PHY, nullptr is returned.
 */
EthPHY *EthPHY::phyForId(Network *net, uint32_t id, uint16_t address, bool useRMII) {
	switch(id) {
		// TI DP83848CVV
		case 0x20005C90:
			return new DP83848C(net, net->mac, useRMII, address);
			break;

		// no suitable PHY could be created
		default:
			LOG(S_ERROR, "Couldn't instantiate PHY with ID %08X", id);
			return nullptr;
	}
}

} /* namespace net */
