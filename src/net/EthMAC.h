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

		// transmit task
		private:
			friend void _MACTXTaskTrampoline(void *);

			void setUpTransmitTask(void);

			void transmitTaskEntry(void);

		private:
			static const size_t TransmitQueueDepth = 8;

			static const size_t TransmitTaskStackSize = 200;
			static const size_t TransmitTaskPriority = 2;

			SemaphoreHandle_t txCompleteSignal = nullptr;
			TaskHandle_t transmitTask = nullptr;
			QueueHandle_t transmitQueue = nullptr;

		// DMA
		public:
			void setRxBuffers(void *buffers, size_t numBufs);
			void releaseRxBuffer(int index);

			void transmitPacket(void *buffer, size_t length, uint32_t userData);

			int availableRxDescriptors(void);

			void dbgCheckDMAStatus(void);

			// TODO: determine if the buffers could be smaller
			static const size_t rxBufSize = (EthMAC::MTU + 100);
			static const size_t txBufSize = (EthMAC::MTU + 100);

		private:
			friend void EthMACDebugTimerCallback(TimerHandle_t);

			void setUpDMARegisters(void);
			void shutDownDMA(void);

			void resumeTxDMA(void);
			void resumeRxDMA(void);

			void relinkRxDescriptors(void);

		private:
			SemaphoreHandle_t txDescriptorLock = nullptr;
			size_t numTxDescriptors = 0;
			void *txDescriptorsMem = nullptr;
			volatile mac_tx_dma_descriptor_t *txDescriptors = nullptr;

			bool dmaTransmittedFramesReady[32];


			SemaphoreHandle_t rxDescriptorLock = nullptr;
			size_t numRxDescriptors = 0;
			void *rxDescriptorsMem = nullptr;
			volatile mac_rx_dma_descriptor_t *rxDescriptors = nullptr;

			bool dmaReceivedFramesReady[32];

			uint64_t dmaReceivedFrames = 0;
			uint64_t dmaReceivedFramesDiscarded = 0;

		// interrupts
		public:
			void handleIRQ(void);

		private:
			void handleMMCInterrupt(void);

			void handleDMAInterrupt(uint32_t dmasr);
			void handleDMAErrorInterrupt(uint32_t dmasr);

			void enableEthernetIRQ(void);
			void disableEthernetIRQ(void);

			uint32_t indexOfLastReceivedISR(void);
			uint32_t indexOfLastTransmittedISR(void);
			void discardLastRxPacket(void);

		private:
			// set if we get a "receive buffer unavailable" error
			bool dmaReceiveStopped = false;
			// set if we get a "transmit buffer unavailable" error
			bool dmaTransmitStopped = false;

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
