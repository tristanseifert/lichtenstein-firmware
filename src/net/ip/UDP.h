/*
 * UDP.h
 *
 * Handles UDP packet reception and transmission.
 *
 *  Created on: Feb 26, 2018
 *      Author: tristan
 */

#ifndef NET_IP_UDP_H_
#define NET_IP_UDP_H_

#include "StackTypes.h"

#include <LichtensteinApp.h>

#include <cstddef>

// forward declare the type of the listening struct
#ifndef UDP_PRIVATE
typedef void udp_listen_t;
typedef void udp_tx_packet_t;
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace ip {
	class Stack;
	class IPv4;
	class UDPSocket;

	class UDP {
		friend class UDPSocket;

		public:
			UDP(Stack *, IPv4 *);
			virtual ~UDP();

			public:
				enum {
					ErrSuccess				= 0,

					ErrNoBookkeepingSpace	= -3000,
					ErrPortInUse				= -3001,
					ErrTxFailure				= -3002,
				};

			// API for the IPv4 handler to call into
			public:
				inline void processUnicastFrame(void *_rx) {
					this->handleReceivedFrame(_rx, UNICAST);
				}
				inline void processMulticastFrame(void *_rx) {
					this->handleReceivedFrame(_rx, MULTICAST);
				}
				inline void processBroadcastFrame(void *_rx) {
					this->handleReceivedFrame(_rx, BROADCAST);
				}

			// private packet handling
			private:
				enum {UNICAST, MULTICAST, BROADCAST};

				void handleReceivedFrame(void *, int);

			// byte order conversion helpers
			private:
				void convertPacketByteOrder(void *);

				void packetNetworkToHost(void *_packet) {
					this->convertPacketByteOrder(_packet);
				}
				void packetHostToNetwork(void *_packet) {
					this->convertPacketByteOrder(_packet);
				}

			// public socket API
			public:
				UDPSocket *createSocket(void);

			// socket API
			protected:
				int bindSocketForPort(UDPSocket *sock, uint16_t port);
				void unbindSocket(UDPSocket *sock);

				int setMulticastReceptionState(UDPSocket *sock, bool enabled);
				int setBroadcastReceptionState(UDPSocket *sock, bool enabled);

				udp_tx_packet_t *getTxBuffer(UDPSocket *sock, size_t payloadLength, int timeout);
				int discardTxBuffer(UDPSocket *sock, udp_tx_packet_t *buffer);
				int sendTxBuffer(UDPSocket *sock, stack_ipv4_addr_t address, unsigned int port, udp_tx_packet_t *buffer);

			// counters
			private:
				// number of packets received that we didn't handle
				uint64_t unhandledRxPackets = 0;
				// number of handled unicast packets
				uint64_t handledRxUnicast = 0;
				// number of handled multicast packets
				uint64_t handledRxMulticast = 0;
				// number of handled broadcast packets
				uint64_t handledRxBroadcast = 0;

			// ports we're listening on
			private:
				// how many listening sockets we can have at a time
				static const size_t listenPortsEntries = 8;
				// all ports that we're listening on
				udp_listen_t *listenPorts;

			// private pointers to the stack
			private:
				Stack *stack = nullptr;
				IPv4 *ipv4 = nullptr;
	};

} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_UDP_H_ */
