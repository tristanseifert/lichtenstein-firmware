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
	class IGMP;
	class ICMP;
	class UDP;

	class Socket;
	class UDPSocket;

	class IPv4 {
		friend class Stack;
		friend class IGMP;
		friend class Socket;

		public:
			IPv4(Stack *_s);
			virtual ~IPv4();

		// IPv4 API
		public:
			void handleIPv4Frame(void *);

		// byte order conversion helpers
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
			void *getIPv4TxBuffer(size_t payloadLength, uint8_t protocol, int timeout = -1);
			bool transmitIPv4TxBuffer(void *buffer, bool requireValidIP = true);
			bool discardTxBuffer(void *buffer);

			void setIPv4Destination(void *buffer, stack_ipv4_addr_t addr);
			void setIPv4Source(void *buffer, stack_ipv4_addr_t addr);

			void setIPv4TTL(void *buffer, uint8_t ttl);

		private:
			// ID of the current packet
			uint16_t currentPacketID = 0;

		// API for IP protocols to deal with RX buffers
		public:
			void releaseRxBuffer(void *buffer);

			stack_ipv4_addr_t getRxBufferSource(void *buffer);
			stack_ipv4_addr_t getRxBufferDestination(void *buffer);

			size_t getRxBufferPayloadLength(void *buffer);

		private:
			// starting TTL value for transmitted packets
			static const uint8_t defaultTTL = 64;

		// Multicast address list
		private:
			bool isMulticastAddressAllowed(stack_ipv4_addr_t);
			int addMulticastAddress(stack_ipv4_addr_t);
			int removeMulticastAddress(stack_ipv4_addr_t);

		private:
			// size of the multicast filter
			static const size_t multicastFilterSize = 8;
			// multicast addresses we will receive on
			stack_ipv4_addr_t multicastFilter[multicastFilterSize];
			// reference count for each address
			unsigned int multicastFilterRefCount[multicastFilterSize];

		private:
			IGMP *igmp = nullptr;
			ICMP *icmp = nullptr;
			UDP *udp = nullptr;

		private:
			Stack *stack = nullptr;
	};

} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_IPV4_H_ */
