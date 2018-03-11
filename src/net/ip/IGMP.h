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

			void insertIGMPChecksum(void *, ssize_t length = -1);
			bool verifyIGMPChecksum(void *, ssize_t length = -1);

		private:
			uint64_t receivedPackets = 0;

		// reply processing task
		private:
			friend void _IGMPTaskTrampoline(void *);

			void taskEntry(void);

			void taskSendMembershipReport(void *);
			void taskSendLeaveGroup(void *);

			int postMessageToTask(void *, int timeout = portMAX_DELAY);

		private:
			// size of the ICMP task's stack, in words
			static const size_t TaskStackSize = 150;
			// priority of the ICMP task
			static const int TaskPriority = 3;
			// how many messages may be pending on the message queue at a time
			static const size_t messageQueueSize = 4;

			TaskHandle_t task = nullptr;
			QueueHandle_t messageQueue = nullptr;

		// helper functions called by the IPv4 stack to send IGMP messages
		public:
			void joinedGroup(stack_ipv4_addr_t address);
			void leftGroup(stack_ipv4_addr_t address);

		private:
			Stack *stack = nullptr;
			IPv4 *ipv4 = nullptr;
	};
} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_IGMP_H_ */