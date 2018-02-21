/*
 * EthMAC.h
 *
 * Exports a more pleasant interface for the STM32's built-in Ethernet MAC,
 * since there's no stdperiph library for it.
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */

#ifndef NET_ETHMAC_H_
#define NET_ETHMAC_H_

#include <cstdint>

class Network;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace net {
	class EthMAC {
		public:
			EthMAC(Network *_net, bool useRMII);
			virtual ~EthMAC();

		// MAC addresses
		public:
			/**
			 * Sets the unicast MAC address of this station, e.g. the first
			 * of the four MAC addresses that are filtered.
			 */
			inline void setMACAddr(uint8_t *address) {
				this->setMACAddr(0, address, true);
			}
			void setMACAddr(int slot, uint8_t *address, bool enable);

			void setPromiscuousMode(bool enable);

		// PHY management
		public:
			uint32_t readPHYId(uint16_t address);

		// management counters
		public:
			// TODO: implement accessors for MMC counters

		private:
			uint64_t mmcTxAfterSingleCollision = 0;
			uint64_t mmcTxAfterMultipleCollision = 0;
			uint64_t mmcTxFrames = 0;

			uint64_t mmcRxBadCRC = 0;
			uint64_t mmcRxAlignmentError = 0;
			uint64_t mmcRxUnicast = 0;

			void setUpMMCRegisters(void);

			void readMMCCounters(void);

		// MDIO interface
		public:
			int mdioRead(uint16_t phy, uint16_t reg);
			int mdioWrite(uint16_t phy, uint16_t reg, uint16_t value);

		private:
			int mdioWait(unsigned int timeout = 200000);

			int mdioSendAddress(uint16_t phy, uint16_t reg, bool write);

		private:

		// initialization
		private:
			void setUpClocks(void);

			void setUpMACRegisters(void);

		private:
			Network *net = nullptr;
			bool rmii = false;
	};
} /* namespace net */

#pragma GCC diagnostic pop

#endif /* NET_ETHMAC_H_ */
