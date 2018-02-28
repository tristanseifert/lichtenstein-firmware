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
typedef void *udp_listen_t;
#endif

namespace ip {
	class Stack;
	class IPv4;

	class UDP {
		public:
			UDP(Stack *, IPv4 *);
			virtual ~UDP();

			// API for the IPv4 handler to call into
			public:
				void processUnicastFrame(void *);
				void processMulticastFrame(void *);
				void processBroadcastFrame(void *);

			// private packet handling
			private:
				enum {UNICAST, MULTICAST, BROADCAST};

				void hadleReceivedFrame(void *, int);

			// byte order conversion helpers
			private:
				void convertPacketByteOrder(void *);

				void packetNetworkToHost(void *_packet) {
					this->convertPacketByteOrder(_packet);
				}
				void packetHostToNetwork(void *_packet) {
					this->convertPacketByteOrder(_packet);
				}

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

#endif /* NET_IP_UDP_H_ */
