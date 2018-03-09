/*
 * IGMP.h
 *
 * Implements the means to transmit IGMP packet for joining/leaving multicast
 * groups. This presently only implements a limited subset of IGMPv2, in that
 * group queries are not implemented.
 *
 *  Created on: Mar 9, 2018
 *      Author: tristan
 */

#ifndef NET_IP_IGMP_H_
#define NET_IP_IGMP_H_

#include "StackTypes.h"

#include <LichtensteinApp.h>



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace ip {
	class Stack;
	class IPv4;

	class IGMP {
		public:
			IGMP(Stack *stack, IPv4 *ipv4);
			virtual ~IGMP();

		// API for the IPv4 handler to call into
		public:
			void processMulticastFrame(void *);

		// byte order conversion helpers
		private:
			void convertPacketByteOrder(void *);

			void packetNetworkToHost(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}
			void packetHostToNetwork(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}

		private:
			uint64_t receivedPackets = 0;

		private:
			Stack *stack = nullptr;
			IPv4 *ipv4 = nullptr;
	};
} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_IGMP_H_ */
