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

#include "OutputTask.h"

// forward declare message types
#ifndef LICHTENSTEIN_PRIVATE
typedef void lichtenstein_header_t;
typedef void lichtenstein_framebuffer_data_t;
typedef void lichtenstein_sync_output_t;
typedef void lichtenstein_node_adoption_t;
#endif

namespace ip {
	class UDPSocket;
};



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

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
			static const TickType_t DiscoveryPeriod = 1000;
			TimerHandle_t discoveryTimer = nullptr;

			// how many ticks to wait before abandoning an adoption
			static const TickType_t AdoptionAbandonPeriod = (100 * 60 * 5);
			TimerHandle_t abandonTimer = nullptr;

		private:
			// multicast group used for Lichtenstein protocol
			static const stack_ipv4_addr_t MulticastGroup = __builtin_bswap32(0xef2a0045);
			// port assigned to the Lichtenstein server
			static const uint16_t Port = 7420;

			// how many ticks to wait on receive calls (controls message latency)
			static const size_t ReceiveTimeout = 10;

			ip::UDPSocket *sock = nullptr;

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

		// processing task
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
			friend void _AbandonTimerCallback(TimerHandle_t);

			void abandonTimerFired(void);

		// adoption
		private:
			// whether the node has been successfully adopted or not
			bool isAdopted = false;

			// IP address of the server that adopted this node
			stack_ipv4_addr_t serverAddr = kIPv4AddressZero;
			// port number to which to send responses
			uint16_t serverPort = 0;

			// number of LEDs on each output
			unsigned int numOutputLEDs[ledout::OutputTask::maxOutputBuffers];

			bool taskHandleAdoption(lichtenstein_node_adoption_t *);

		// packet handling
		private:
			void populateLichtensteinHeader(lichtenstein_header_t *, uint16_t);

			uint32_t calculatePacketCRC(lichtenstein_header_t *, size_t);

			void setUpCRC(void);
			void cleanUpCRC(void);
			uint32_t doHWCRC(void *, size_t);

			uint32_t doSWCRC(void *, size_t);

		// counters
		private:
			uint32_t invalidCRCErrors = 0;

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

#pragma GCC diagnostic pop

#endif /* LEDOUT_LICHTENSTEINHANDLER_H_ */
