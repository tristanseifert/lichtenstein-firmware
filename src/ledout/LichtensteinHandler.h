/*
 * LichtensteinHandler.h
 *
 *  Created on: Mar 10, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_LICHTENSTEINHANDLER_H_
#define LEDOUT_LICHTENSTEINHANDLER_H_

#include <cstddef>

#include <net/ip/StackTypes.h>

#include <LichtensteinApp.h>

// forward declare message types
#ifndef LICHTENSTEIN_PRIVATE
typedef void lichtenstein_header_t;
typedef void lichtenstein_framebuffer_data_t;
typedef void lichtenstein_sync_output_t;
#endif

namespace ip {
	class UDPSocket;
};

namespace ledout {
	class LichtensteinHandler {
		public:
			LichtensteinHandler();
			virtual ~LichtensteinHandler();


		private:
			// size of the task's stack, in words
			static const size_t TaskStackSize = 200;
			// priority of the task
			static const int TaskPriority = 1;
			// how many messages may be pending on the message queue at a time
			static const size_t MessageQueueSize = 4;

			TaskHandle_t task = nullptr;
			QueueHandle_t messageQueue = nullptr;

			// how frequently we send discovery packets
			static const size_t DiscoveryPeriod = 1000;
			TimerHandle_t discoveryTimer = nullptr;

		private:
			// multicast group used for Lichtenstein protocol
			static const stack_ipv4_addr_t MulticastGroup = __builtin_bswap32(0xef2a0045);
			// port assigned to the Lichtenstein server
			static const uint16_t Port = 7420;

			// how many ticks to wait on receive calls (controls message latency)
			static const size_t ReceiveTimeout = 10;

			ip::UDPSocket *sock = nullptr;

			// TODO: fill this in
			stack_ipv4_addr_t serverAddr = kIPv4AddressZero;

		private:
			friend void _DoMulticastAnnouncement(TimerHandle_t);

			// messages to pass in the queue
			typedef enum {
				// multicast discovery
				kSendMulticastDiscovery
			} message_type_t;

			int postMessageToTask(message_type_t, int timeout = portMAX_DELAY);

			void taskHandleRequest(message_type_t);
			void taskSendMulticastDiscovery(void);

		private:
			friend void _LichtensteinTaskTrampoline(void *);
			friend void _ConversionCompleteCallback(void *, void *);

			void taskEntry(void);

			void setUpSocket(void);
			void tearDownSocket(void);

			bool taskHandleFBData(lichtenstein_framebuffer_data_t *);
			bool taskHandleSyncOut(lichtenstein_sync_output_t *);

			int ackPacket(lichtenstein_header_t *, bool nack = false);

		private:
			void populateLichtensteinHeader(lichtenstein_header_t *, uint16_t);

			uint32_t calculatePacketCRC(lichtenstein_header_t *, size_t);

			void setUpCRC(void);
			void cleanUpCRC(void);
			uint32_t doHWCRC(void *, size_t);

			uint32_t doSWCRC(void *, size_t);

		// byte order conversion helpers
		private:
			int convertPacketByteOrder(void *, bool, size_t);

			inline int packetNetworkToHost(void *_packet, size_t length) {
				return this->convertPacketByteOrder(_packet, true, length);
			}
			inline int packetHostToNetwork(void *_packet, size_t length) {
				return this->convertPacketByteOrder(_packet, false, length);
			}
	};

} /* namespace ledout */

#endif /* LEDOUT_LICHTENSTEINHANDLER_H_ */
