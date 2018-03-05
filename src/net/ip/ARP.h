/*
 * ARP.h
 *
 * Handles ARP packets received on the wire, and provides services to convert
 * between an IP address and a MAC address, if required to send a packet.
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */

#ifndef NET_IP_ARP_H_
#define NET_IP_ARP_H_

#include "ARPPrivate.h"

#include <LichtensteinApp.h>

#include <cstddef>

namespace ip {
	class Stack;

	class ARP {
		public:
			ARP(Stack *s);
			virtual ~ARP();

		// ARP API
		public:
			void handleARPFrame(void *);

			bool resolveIPv4(stack_ipv4_addr_t addr, stack_mac_addr_t *result, int timeout);

			void sendGratuitousARP(void);
			void clearARPCache(void);

		private:
			void handleARPRequest(void *);
			void handleARPReply(void *);

			void sendARPReply(void *);

		// ARP cache
		private:
			int numFreeCacheEntries(void);
			void discardOldestEntry(void);

			void insertAddress(stack_mac_addr_t mac, stack_ipv4_addr_t addr);

			void dbgDumpCache(void);

		private:
			// maximum number of entries
			static const size_t cacheEntries = 16;
			// ARP cache
			arp_ipv4_cache_entry_t cache[cacheEntries];

		// ARP resolution notification list
		private:
			// maximum tasks that can be waiting on an IP resolution
			static const size_t notificationsEntries = 6;

			// list of tasks waiting on an IP resolution
			arp_resolve_notifications_t notifications[notificationsEntries];

		// ARP task
		private:
			friend void _ARPTaskTrampoline(void *);

			void taskEntry(void);
			void taskGenerateResponse(void *);
			void taskResolveIP(void *);
			void taskSendGratuitous(void *);

			bool postMessageToTask(void *, int timeout = portMAX_DELAY);

			void sendReceivedPacketToTask(void *, int timeout = portMAX_DELAY);
			void sendResponseRequestToTask(void *, int timeout = portMAX_DELAY);

		private:
			// size of the ARP task's stack, in words
			static const size_t TaskStackSize = 150;
			// priority of the ARP task
			static const int TaskPriority = 2;

			// how many messages may be pending on the message queue at a time
			static const size_t messageQueueSize = 4;

			TaskHandle_t task = nullptr;
			QueueHandle_t messageQueue = nullptr;

		// byte order helpers
		private:
			void convertPacketByteOrder(void *);

			void packetNetworkToHost(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}
			void packetHostToNetwork(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}

		private:
			Stack *stack = nullptr;
	};

} /* namespace ip */

#endif /* NET_IP_ARP_H_ */
