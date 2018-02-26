/*
 * ICMP.cpp
 *
 *  Created on: Feb 26, 2018
 *      Author: tristan
 */
#define LOG_MODULE "ICMP"

#include "ICMP.h"
#include "ICMPPrivate.h"

#include "IPv4.h"
#include "IPv4Private.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>

#include <cstddef>
#include <cstring>

// generate logs for each received ICMP packet
#define LOG_RECEIVED_PACKETS					0
// generate logs for each received ping
#define LOG_RECEIVED_PINGS					0
// generate logs for each transmitted ICMP packet
#define LOG_TRANSMITTED_PACKETS				0


namespace ip {

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _ICMPTaskTrampoline(void *ctx) {
	(static_cast<ICMP *>(ctx))->taskEntry();
}

/**
 * Initializes the ICMP handler.
 */
ICMP::ICMP(Stack *_stack, IPv4 *_ipv4) : stack(_stack), ipv4(_ipv4) {
	BaseType_t ok;

	LOG(S_DEBUG, "ICMP packet size: %u", sizeof(icmp_packet_ipv4_t));

	// create the queue
	this->messageQueue = xQueueCreate(ICMP::messageQueueSize, sizeof(icmp_task_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue!");
	}

	// now, create the task
	ok = xTaskCreate(_ICMPTaskTrampoline, "ICMP", ICMP::TaskStackSize,
					 this, ICMP::TaskPriority, &this->task);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task!");
	}
}

/**
 * Deletes the ICMP task and queue.
 */
ICMP::~ICMP() {
	// delete task and queue
	if(this->task) {
		vTaskDelete(this->task);
	}

	if(this->messageQueue) {
		vQueueDelete(this->messageQueue);
	}
}



/**
 * Processes an ICMP packet received as an unicast frame.
 */
void ICMP::processUnicastFrame(void *_packet) {
	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *) _packet;

	// get the packet and convert byte order
	icmp_packet_ipv4_t *packet = (icmp_packet_ipv4_t *) rx->payload;
	this->packetNetworkToHost(packet);

	// get the source address
	stack_ipv4_addr_t source = this->ipv4->getRxBufferSource(rx);

	// do some logging
#if LOG_RECEIVED_PINGS || LOG_RECEIVED_PACKETS
	char srcIpStr[16];
	Stack::ipToString(source, srcIpStr, 16);
#endif
#if LOG_RECEIVED_PACKETS
	LOG(S_DEBUG, "Received ICMP frame with type %u from %s", packet->type, srcIpStr);
#endif

	// this message struct is used if we need to act on the message
	icmp_task_message_t msg;
	msg.source = source;

	// is it an echo request?
	if(packet->type == kCIMPTypeEchoRequest) {
#if LOG_RECEIVED_PINGS
		LOG(S_DEBUG, "Received echo request from %s", srcIpStr);
#endif

		// generate a response if enabled
		if(this->respondToEchoRequests) {
			msg.type = kICMPMessageSendEchoReply;

			msg.data.echoRequest.identifier = packet->data.echoRequest.identifier;
			msg.data.echoRequest.sequence = packet->data.echoRequest.sequence;

			// copy the additional data in this packet
			int payloadSz = this->ipv4->getRxBufferPayloadLength(rx);
			payloadSz -= 8; // basic ICMP header is 8 bytes long

			if(payloadSz > 0) {
				// allocate a buffer for the payload
				void *buffer = pvPortMalloc(payloadSz);

				if(buffer == nullptr) {
					// we couldn't allocate a buffer :(
					LOG(S_ERROR, "Can't allocate payload buffer");

					// we still will send the packet, but without the payload
					msg.data.echoRequest.additionalDataLength = 0;
				} else {
					// copy the payload and pass it with the message
					memcpy(buffer, packet->data.echoRequest.payload, payloadSz);

					msg.data.echoRequest.additionalData = buffer;
					msg.data.echoRequest.additionalDataLength = payloadSz;
				}
			} else {
				// no payload available
				msg.data.echoRequest.additionalDataLength = 0;
			}

			if(this->postMessageToTask(&msg, 0) == false) {
				LOG(S_ERROR, "Can't respond to echo request");

				// deallocate the buffer we allocated earlier
				vPortFree(msg.data.echoRequest.additionalData);
			}
		}
	}

	// release the packet
	this->ipv4->releaseRxBuffer(rx);
}



/**
 * Entry point for the ICMP task. This continuously checks for messages on
 * the message queue, and acts on them.
 */
void ICMP::taskEntry(void) {
	BaseType_t ok;
	icmp_task_message_t msg;

	while(1) {
		// attempt to receive a message
		ok = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG(S_ERROR, "Error reading from queue");
			continue;
		}

		// process it
		switch(msg.type) {
			// generate an echo response
			case kICMPMessageSendEchoReply:
				this->taskSendEchoReplyTo(&msg);
				break;
		}
	}
}

/**
 * Submits a message to the ICMP task.
 *
 * @return true if the message was submitted, false otherwise (queue full
 * 		   and timeout expired or some other error)
 */
bool ICMP::postMessageToTask(void *msg, int timeout) {
	BaseType_t ok;

	ok = xQueueSendToBack(this->messageQueue, msg, timeout);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't send message to task: %u", ok);
		return false;
	}

	return true;
}



/**
 * Generates an echo reply.
 */
void ICMP::taskSendEchoReplyTo(void *_msg) {
	icmp_task_message_t *msg = (icmp_task_message_t *) _msg;

	stack_ipv4_addr_t destination = msg->source;

	// logging
#if LOG_TRANSMITTED_PACKETS
	char destIpStr[16];
	Stack::ipToString(destination, destIpStr, 16);

	LOG(S_DEBUG, "Sending Echo Reply to %s", destIpStr);
#endif

	// attempt to get a TX buffer
	size_t responseSize = 8;
	responseSize += msg->data.echoRequest.additionalDataLength;

	void *_tx = this->ipv4->getIPv4TxBuffer(responseSize, kIPv4ProtocolICMP);
	stack_ipv4_tx_packet_t *tx = (stack_ipv4_tx_packet_t *) _tx;

	if(tx == nullptr) {
		// exit if we can't get a buffer
		LOG(S_ERROR, "Couldn't get transmit buffer");
		return;
	}

	// populate the ICMP packet
	icmp_packet_ipv4_t *icmp = (icmp_packet_ipv4_t *) tx->payload;
	memset(icmp, 0, 8); // clear only the first 8 bytes (that's all we're sending)

	icmp->type = kICMPTypeEchoReply;
	icmp->code = 0;

	icmp->checksum = 0x0000; // inserted by MAC (see RM0008 pg 987)

	// copy the sequence/ID from the original packet
	icmp->data.echoRequest.identifier = msg->data.echoRequest.identifier;
	icmp->data.echoRequest.sequence = msg->data.echoRequest.sequence;

	// copy the payload buffer
	if(msg->data.echoRequest.additionalDataLength) {
		memcpy(&icmp->data.echoRequest.payload,
				msg->data.echoRequest.additionalData,
				msg->data.echoRequest.additionalDataLength);

		// deallocate the buffer
		vPortFree(msg->data.echoRequest.additionalData);
		msg->data.echoRequest.additionalData = nullptr;
	}

	// set the destination address
	this->ipv4->setIPv4Destination(tx, destination);

	// byteswap the buffer and send it
	this->packetHostToNetwork(icmp);

	if(this->ipv4->transmitIPv4TxBuffer(tx) == false) {
		LOG(S_ERROR, "Couldn't send echo reply");
	}
}



/**
 * Converts the byte order of an ICMP packet.
 */
void ICMP::convertPacketByteOrder(void *_icmp) {
	icmp_packet_ipv4_t *icmp = (icmp_packet_ipv4_t *) _icmp;

	icmp->checksum = __builtin_bswap16(icmp->checksum);
}

} /* namespace ip */
