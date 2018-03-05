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
#include "EthPHYTypes.h"

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

			int setMulticastAddr(uint8_t *address, bool enable);

			void setPromiscuousMode(bool enable);

		// CRC
		private:
			uint32_t calcCRC(void *data, size_t length);

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

		// transmitting packets
		private:
			friend void _MACTXTaskTrampoline(void *);

			void setUpTransmitTask(void);
			void transmitTaskEntry(void);

		public:
			void transmitPacket(void *buffer, size_t length, uint32_t userData);

		private:
			// how many writes may be pending at a time
			static const size_t TransmitQueueDepth = 8;
			// size of the stack for the write task
			static const size_t TransmitTaskStackSize = 100;
			// write task priority
			static const size_t TransmitTaskPriority = 2;

			// the transmit ISR signals this semaphore
			SemaphoreHandle_t txCompleteSignal = nullptr;

			TaskHandle_t transmitTask = nullptr;
			// write requests are stored here
			QueueHandle_t transmitQueue = nullptr;

			// lock this semaphore when we modify the descriptor
			SemaphoreHandle_t txDescriptorLock = nullptr;
			volatile mac_tx_dma_descriptor_t txDescriptor;
			volatile mac_tx_dma_descriptor_t txDescriptor2;

			// number of frames transmitted
			volatile uint64_t dmaTransmittedFrames = 0;

		// DMA
		public:
			void setRxBuffers(void *buffers, size_t numBufs);
			void releaseRxBuffer(int index);

			int availableRxDescriptors(void);

			void resetReceiveDescriptors(void);

			void dbgCheckDMAStatus(void);

			// TODO: determine if the buffers could be smaller
			static const size_t rxBufSize = (EthMAC::MTU);
			static const size_t txBufSize = (EthMAC::MTU);

		private:
			friend void EthMACDebugTimerCallback(TimerHandle_t);

			void setUpDMARegisters(void);
			void shutDownDMA(void);

			void relinkRxDescriptors(void);

		// Receive structures
		private:
			SemaphoreHandle_t rxDescriptorLock = nullptr;

			// number of receive descriptors
			size_t numRxDescriptors = 0;
			// memory for receive descriptors
			void *rxDescriptorsMem = nullptr;
			// address of first receive descriptor
			volatile mac_rx_dma_descriptor_t *rxDescriptors = nullptr;

			// size of the frame ready array
			static const size_t dmaReceivedFramesReadySz = 32;
			// which descriptors can we receive packets in to?
			volatile bool dmaReceivedFramesReady[dmaReceivedFramesReadySz];

			// how many frames were received with DMA
			volatile uint64_t dmaReceivedFrames = 0;
			// how many frames were discarded due to unavailable buffers
			volatile uint64_t dmaReceivedFramesDiscarded = 0;

			volatile mac_rx_dma_descriptor_t *rxLastReceived = nullptr;

		// interrupts
		public:
			// called when the ISR comes in
			void handleIRQ(void);

		private:
			void handleMMCInterrupt(void);

			void handleDMAInterrupt(uint32_t dmasr);
			void handleDMAErrorInterrupt(uint32_t dmasr);

			void enableEthernetIRQ(void);
			void disableEthernetIRQ(void);

			uint32_t indexOfLastReceivedISR(void);
			void discardLastRxPacket(void);

		// Interrupt state
		private:
			// set if we get a "receive buffer unavailable" error
			volatile bool dmaReceiveStopped = false;
			// set if we get a "transmit buffer unavailable" error
			volatile bool dmaTransmitStopped = false;

		// initialization
		private:
			void setUpClocks(void);
			void setUpMACRegisters(void);

		// link state change
		public:
			void linkStateChanged(bool linkUp, bool duplex, net_link_speed_t speed);

		// pointer to the network class
		private:
			Network *net = nullptr;
			bool rmii = false;
	};
} /* namespace net */

#pragma GCC diagnostic pop

#endif /* NET_ETHMAC_H_ */
