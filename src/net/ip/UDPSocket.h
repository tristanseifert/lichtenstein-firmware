/*
 * UDPSocket.h
 *
 *  Created on: Mar 2, 2018
 *      Author: tristan
 */

#ifndef NET_IP_UDPSOCKET_H_
#define NET_IP_UDPSOCKET_H_

#include "Socket.h"

#include <LichtensteinApp.h>



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace ip {
	class UDP;

	class UDPSocket: public Socket {
		friend class UDP;

		public:
			UDPSocket(UDP *_udp);
			virtual ~UDPSocket();

		public:
			virtual int receive(void **buffer, size_t *bytesRead, unsigned int timeout = -1);
			virtual int discardRx(void *buffer);

		public:
			virtual int prepareTx(void **buffer, size_t length, unsigned int timeout = -1);
			virtual int queueTx(void *buffer, unsigned int timeout = -1);
			virtual int discardTx(void *buffer);

			int sendTo(void *buffer, stack_ipv4_addr_t addr, uint16_t port, unsigned int timeout = -1);


		public:
			virtual int open(void);
			virtual int close(void);

			virtual int connect(stack_ipv4_addr_t address, unsigned int port);
			virtual int bind(unsigned int port);


		public:
			virtual int setSockOpt(socket_protocol_t protocol, socket_option_t option, void *value, size_t length);
			virtual int getSockOpt(socket_protocol_t protocol, socket_option_t option, void *out, size_t length);

		// API called by UDP class
		protected:
			enum {UNICAST, MULTICAST, BROADCAST};

			bool receivedFrame(void *ipRxBuffer, int type);

		private:
			UDP *udp = nullptr;

		// receive message queue
		private:
			// number of pending messages in the receive queue
			static const size_t rxMessageQueueSize = 4;
			// message queue used for reads
			QueueHandle_t rxMessageQueue = nullptr;

			// number of packets dropped because the receive queue was full
			uint64_t droppedRxPackets = 0;

			// size of the rx buffer map
			static const size_t rxMapSize = UDPSocket::rxMessageQueueSize;
			// map a buffer value to the rx struct
			struct {
				void *buffer;
				void *rxBuffer;
			} rxBufferMap[rxMapSize];

		// transmit queue map
		private:
			// number of outstanding tx packets
			static const size_t txMapSize = 2;
			// tx map (buffer -> packet)
			struct {
				void *buffer;
				void *txPacket;
			} txBufferMap[txMapSize];

		private:
			unsigned int multicastRefCount = 0;
	};

} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_UDPSOCKET_H_ */
