/*
 * ICMP.h
 *
 * A rudimentary ICMP implementation. For now, all this can do is reply to
 * any ICMP Echo Request frames.
 *
 *  Created on: Feb 26, 2018
 *      Author: tristan
 */

#ifndef NET_IP_ICMP_H_
#define NET_IP_ICMP_H_

#include "StackTypes.h"

#include <LichtensteinApp.h>



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace ip {
	class Stack;
	class IPv4;

	class ICMP {
		public:
			ICMP(Stack *stack, IPv4 *ipv4);
			virtual ~ICMP();

		// API for the IPv4 handler to call into
		public:
			void processUnicastFrame(void *);

		// byte order conversion helpers
		private:
			void convertPacketByteOrder(void *);

			void packetNetworkToHost(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}
			void packetHostToNetwork(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}

		// processing task
		private:
			friend void _ICMPTaskTrampoline(void *);

			void taskEntry(void);

			bool postMessageToTask(void *msg, int timeout = portMAX_DELAY);

			void taskSendEchoReplyTo(void *msg);

		private:
			// size of the ICMP task's stack, in words
			static const size_t TaskStackSize = 128;
			// priority of the ICMP task
			static const int TaskPriority = 3;

			// how many messages may be pending on the message queue at a time
			static const size_t messageQueueSize = 4;

			TaskHandle_t task = nullptr;
			QueueHandle_t messageQueue = nullptr;

		// configuration
		private:
			bool respondToEchoRequests = true;

		private:
			Stack *stack = nullptr;
			IPv4 *ipv4 = nullptr;
	};

} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_ICMP_H_ */
