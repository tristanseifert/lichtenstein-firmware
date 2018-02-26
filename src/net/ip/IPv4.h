/*
 * IPv4.h
 *
 * Handles IPv4 packets, calling in to the appropriate application protocol
 * handlers.
 *
 *  Created on: Feb 25, 2018
 *      Author: tristan
 */

#ifndef NET_IP_IPV4_H_
#define NET_IP_IPV4_H_

#include "StackTypes.h"

#include <cstddef>



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace ip {
	class Stack;

	class IPv4 {
		public:
			IPv4(Stack *_s);
			virtual ~IPv4();

		// IPv4 API
		public:
			void handleIPv4Frame(void *);

		private:
			void convertPacketByteOrder(void *);

			void packetNetworkToHost(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}
			void packetHostToNetwork(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}

		// API for IP protocols to get TX buffers
		public:
			void *getIPv4TxBuffer(size_t payloadLength, uint8_t protocol);
			bool transmitIPv4TxBuffer(void *buffer);

		private:
			// starting TTL value for transmitted packets
			static const uint8_t defaultTTL = 0xFF;

		// Multicast address list
		private:
			bool isMulticastAddressAllowed(stack_ipv4_addr_t);
			int addMulticastAddress(stack_ipv4_addr_t);
			int removeMulticastAddress(stack_ipv4_addr_t);

		private:
			// size of the multicast filter
			static const size_t multicastFilterSize = 16;
			// multicast addresses we will receive on
			stack_ipv4_addr_t multicastFilter[multicastFilterSize];
			// reference count for each address
			size_t multicastFilterRefCount[multicastFilterSize];

		private:
			Stack *stack = nullptr;
	};

} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_IPV4_H_ */
