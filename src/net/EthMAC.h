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

#include <LichtensteinApp.h>

#include <cstddef>
#include <cstdint>

#include "EthMACPrivate.h"

class Network;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace net {
	class EthMAC {
		public:
			EthMAC(Network *_net, bool useRMII);
			virtual ~EthMAC();

		public:
			// maximum payload size
			static const size_t MTU = 1500;

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

		// PHY management
		public:
			uint32_t readPHYId(uint16_t address);

		// Reset Handling
		private:
			void reset(void);

		// DMA
		public:
			void setRxBuffers(void *buffers, size_t numBufs);
			void setTxBuffers(void *buffers, size_t numBufs);

			bool getRxPacket(uint8_t **data, size_t *length, unsigned int *bufIndex);
			void releaseRxPacket(int index);

			void dbgCheckDMAStatus(void);

			// TODO: determine if the buffers could be smaller
			static const size_t rxBufSize = (EthMAC::MTU + 100);
			static const size_t txBufSize = (EthMAC::MTU + 100);

		private:
			void setUpDMARegisters(void);

			void resumeTxDMA(void);
			void resumeRxDMA(void);

			void shutDownDMA(void);

			SemaphoreHandle_t txDescriptorLock = nullptr;
			size_t numTxDescriptors = 0;

			void *txDescriptorsMem = nullptr;
			mac_tx_dma_descriptor_t *txDescriptors = nullptr;

			SemaphoreHandle_t rxDescriptorLock = nullptr;
			size_t numRxDescriptors = 0;

			void *rxDescriptorsMem = nullptr;
			mac_rx_dma_descriptor_t *rxDescriptors = nullptr;

		// interrupts
		public:
			void handleIRQ(void);

			void handleMMCInterrupt(void);

			void handleDMAInterrupt(uint32_t dmasr);
			void handleDMAErrorInterrupt(uint32_t dmasr);

			void enableEthernetIRQ(void);
			void disableEthernetIRQ(void);

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
