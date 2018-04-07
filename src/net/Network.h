/*
 * Network.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef NET_NETWORK_H_
#define NET_NETWORK_H_

#include "NetworkPrivate.h"

#include <cstdint>

#include <LichtensteinApp.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

extern "C" void ETH_IRQHandler(void);

namespace net {
	class EthMAC;
	class EthPHY;
}

namespace ip {
	class Stack;
	class UDPSocket;
}

class Network {
	friend class net::EthMAC;
	friend class net::EthPHY;

	friend class ip::Stack;

	public:
		static void init(void);
		static Network *sharedInstance(void) noexcept;

	private:
		Network();

		void startNetServices(void);

	// initialization
	private:
		void setUpTask(void);

		void setUpClocks(void);
		void setUpMAC(void);
		void setUpEthernetGPIOs(void);

		void scanForPHYs(void);

		void setUpStack(void);

		void readIPConfig(void);
		void parseIPConfig(void *);
		void setUpIPConfigDefaults(void);

	// global helpers
	public:
		static ip::UDPSocket *getUDPSocket(void) noexcept;

		const ip::Stack *getStack(void) const {
			return this->stack;
		}

	// network message task
	private:
		friend void _NetTaskTrampoline(void *ctx);

		void taskEntry(void);

		bool postMessage(network_message_t *msg);

		void handleReceivedFrame(network_message_t *msg);

		void handleTransmittedFrame(network_message_t *msg);

	private:
		// size of the network task's stack, in words
		static const size_t TaskStackSize = 300;
		// priority of the network task
		static const int TaskPriority = 3;

		// how many messages may be pending on the message queue at a time
		static const size_t messageQueueSize = 20;

		TaskHandle_t task = nullptr;
		QueueHandle_t messageQueue = nullptr;

		// used to determine how many free buffers we have
		SemaphoreHandle_t txBuffersFreeSemaphore = nullptr;


	// PHY and MAC
	private:
		friend void ETH_IRQHandler(void);

		net::EthMAC *mac = nullptr;
		net::EthPHY *phy = nullptr;


	// buffers
	private:
		void allocBuffers(void);

		static const size_t numRxBuffers = 8;
		void *rxBuffers[numRxBuffers];

		static const size_t numTxBuffers = 4;
		void *txBuffers[numTxBuffers];

		bool txBuffersFree[numTxBuffers];
		SemaphoreHandle_t txBuffersFreeMutex = nullptr;

	// network stack
	private:
		ip::Stack *stack = nullptr;

	public:
		int releaseRxPacket(uint32_t userData);
		int registerMulticastMAC(uint8_t *address);
		int unregisterMulticastMAC(uint8_t *address);

	// API for network stack
	public:
		void *getTxBuffer(size_t, int);
		void queueTxBuffer(void *);
		void releaseTxBuffer(void *);

	private:
		size_t bytesToTransmit[numTxBuffers];


	// callbacks from PHY
	protected:
		void _phyLinkStateChange(bool isLinkUp, bool fromISR);

	// MAC address reading
	private:
		static const uint8_t ethParamMACOffset = 0xFA;

		uint8_t macAddress[6];

		void readMACFromEEPROM(void);

	private:
		virtual ~Network();
};

#pragma GCC diagnostic pop

#endif /* NET_NETWORK_H_ */
