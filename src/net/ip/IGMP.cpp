/*
 * IGMP.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: tristan
 */
#define LOG_MODULE "IGMP"

#include "IGMP.h"
#include "IGMPPrivate.h"

#include "IPv4.h"
#include "IPv4Private.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>

#include <cstddef>
#include <cstring>



// log transmitted IGMP packets
#define LOG_TRANSMITTED_PACKETS				1
// log received IGMP packets
#define LOG_RECEIVED_PACKETS					1



namespace ip {

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _IGMPTaskTrampoline(void *ctx) {
	(static_cast<IGMP *>(ctx))->taskEntry();
}

/**
 * Initializes the IGMP handler.
 */
IGMP::IGMP(Stack *_stack, IPv4 *_ipv4) : stack(_stack), ipv4(_ipv4) {
	int err = 0;
	BaseType_t ok;

	// create the queue
	this->messageQueue = xQueueCreate(IGMP::messageQueueSize, sizeof(igmp_task_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue!");
	}

	// now, create the task
	ok = xTaskCreate(_IGMPTaskTrampoline, "IGMP", IGMP::TaskStackSize,
					 this, IGMP::TaskPriority, &this->task);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task!");
	}

	// register for the all hosts (224.0.0.1) address
	err = this->ipv4->addMulticastAddress(kIPv4AddressAllHosts);

	if(err != 0) {
		LOG(S_FATAL, "Couldn't subscribe to all hosts group");
	}

}

/**
 * Tears down the IGMP handler.
 */
IGMP::~IGMP() {
	// unregister for the all hosts (224.0.0.1) address
	this->ipv4->removeMulticastAddress(kIPv4AddressAllHosts);

	// delete task and queue
	if(this->task) {
		vTaskDelete(this->task);
	}

	if(this->messageQueue) {
		vQueueDelete(this->messageQueue);
	}
}



/**
 * Handles a received IGMP packet.
 *
 * @param _packet Address of payload (start of IGMP packet)
 */
void IGMP::processMulticastFrame(void *_packet) {
	int err;

	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *)  _packet;

	// increment received packets counter
	this->receivedPackets++;

	// parse packet
	igmp_packet_ipv4_t *igmp = (igmp_packet_ipv4_t *) rx->payload;
	this->packetNetworkToHost(igmp);

	// handle IGMPv1 timeouts
	if(igmp->timeout == 0) {
		igmp->timeout = 100;
	}

	// logging
#if LOG_RECEIVED_PACKETS
	char ipStr[16];
	Stack::ipToString(igmp->address, ipStr, 16);

	LOG(S_DEBUG, "Received IGMP message 0x%02x for %s", igmp->type, ipStr);
#endif

	// was it a membership query?
	if(igmp->type == kIGMPMessageMembershipQuery) {
		// if the address is zero, it's a general query
		if(igmp->address == kIPv4AddressZero) {
			// TODO: respond to general queries
			LOG(S_ERROR, "Unhandled general query");
		} else {
			// respond to just this address
			igmp_task_message_t msg;
			memset(&msg, 0, sizeof(igmp_task_message_t));

			msg.type = kIGMPSendMembershipForGroup;
			msg.address = igmp->address;

			err = this->postMessageToTask(&msg, 0);

			if(err != 0) {
				LOG(S_ERROR, "Couldn't queue membership reply");
			}
		}
	}

	// release packet
	this->ipv4->releaseRxBuffer(rx);
}



/**
 * Processing task entry point
 */
void IGMP::taskEntry(void) {
	BaseType_t ok;
	igmp_task_message_t msg;

	while(1) {
		// attempt to receive a message
		ok = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG(S_ERROR, "Error reading from queue");
			continue;
		}

		// process message
		switch(msg.type) {
			// respond to a membership query for the given group
			case kIGMPSendMembershipForGroup: {
				this->taskSendMembershipReport(&msg);
				break;
			}

			// send a "leave group" message
			case kIGMPSendLeaveGroup: {
				this->taskSendLeaveGroup(&msg);
				break;
			}
		}
	}
}

/**
 * Prepares and sends a membership report message.
 *
 * @param _msg IGMP task message struct
 */
void IGMP::taskSendMembershipReport(void *_msg) {
	igmp_task_message_t *msg = (igmp_task_message_t *) _msg;

	// attempt to get a TX buffer
	const size_t responseSize = sizeof(igmp_packet_ipv4_t);

	void *_tx = this->ipv4->getIPv4TxBuffer(responseSize, kIPv4ProtocolIGMP);
	stack_ipv4_tx_packet_t *tx = (stack_ipv4_tx_packet_t *) _tx;

	if(tx == nullptr) {
		// exit if we can't get a buffer
		LOG(S_ERROR, "Couldn't get transmit buffer");
		return;
	}

	// logging
#if LOG_TRANSMITTED_PACKETS
	char destIpStr[16];
	Stack::ipToString(msg->address, destIpStr, 16);

	LOG(S_DEBUG, "Sending membership report for %s", destIpStr);
#endif

	// populate the IGMP packet
	igmp_packet_ipv4_t *igmp = (igmp_packet_ipv4_t *) tx->payload;
	memset(igmp, 0, responseSize);

	igmp->type = kIGMPMessageMembershipReport;
	igmp->address = msg->address;

	// set the destination address
	this->ipv4->setIPv4Destination(tx, msg->address);

	// byteswap the buffer and send it
	this->packetHostToNetwork(igmp);

	if(this->ipv4->transmitIPv4TxBuffer(tx) == false) {
		LOG(S_ERROR, "Couldn't send membership report");
	}
}

/**
 * Prepares and sends a "leave group" message.
 *
 * @param _msg IGMP task message struct
 */
void IGMP::taskSendLeaveGroup(void *_msg) {
	igmp_task_message_t *msg = (igmp_task_message_t *) _msg;

	// attempt to get a TX buffer
	const size_t responseSize = sizeof(igmp_packet_ipv4_t);

	void *_tx = this->ipv4->getIPv4TxBuffer(responseSize, kIPv4ProtocolIGMP);
	stack_ipv4_tx_packet_t *tx = (stack_ipv4_tx_packet_t *) _tx;

	if(tx == nullptr) {
		// exit if we can't get a buffer
		LOG(S_ERROR, "Couldn't get transmit buffer");
		return;
	}

	// logging
#if LOG_TRANSMITTED_PACKETS
	char destIpStr[16];
	Stack::ipToString(msg->address, destIpStr, 16);

	LOG(S_DEBUG, "Sending leave request for %s", destIpStr);
#endif

	// populate the IGMP packet
	igmp_packet_ipv4_t *igmp = (igmp_packet_ipv4_t *) tx->payload;
	memset(igmp, 0, responseSize);

	igmp->type = kIGMPMessageLeaveGroup;
	igmp->address = msg->address;

	// set the destination address
	this->ipv4->setIPv4Destination(tx, kIPv4AddressAllRouters);

	// byteswap the buffer and send it
	this->packetHostToNetwork(igmp);

	if(this->ipv4->transmitIPv4TxBuffer(tx) == false) {
		LOG(S_ERROR, "Couldn't send leave request");
	}
}



/**
 * Generates an unsolicited membership report for the given group. This should
 * be called when a new group is listened on.
 *
 * @param address Multicast group address
 */
void IGMP::joinedGroup(stack_ipv4_addr_t address) {
	int err;

	// make sure address is multicast
	if(isIPv4Multicast(address) == false) {
		char ipStr[16];
		Stack::ipToString(address, ipStr, 16);

		LOG(S_ERROR, "%s is not a multicast address", ipStr);
		return;
	}

	// post message
	igmp_task_message_t msg;
	memset(&msg, 0, sizeof(igmp_task_message_t));

	msg.type = kIGMPSendMembershipForGroup;
	msg.address = address;

	err = this->postMessageToTask(&msg, 0);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't queue group joined message");
	}
}

/**
 * Generates a "leave group" message. This should be called when the last
 * client leaves a group.
 *
 * @param address Multicast group address
 */
void IGMP::leftGroup(stack_ipv4_addr_t address) {
	int err;

	// make sure address is multicast
	if(isIPv4Multicast(address) == false) {
		char ipStr[16];
		Stack::ipToString(address, ipStr, 16);

		LOG(S_ERROR, "%s is not a multicast address", ipStr);
		return;
	}

	// post message
	igmp_task_message_t msg;
	memset(&msg, 0, sizeof(igmp_task_message_t));

	msg.type = kIGMPSendLeaveGroup;
	msg.address = address;

	err = this->postMessageToTask(&msg, 0);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't queue group left message");
	}
}

/**
 * Submits a message to the ICMP task.
 *
 * @return true if the message was submitted, false otherwise (queue full
 * 		   and timeout expired or some other error)
 */
bool IGMP::postMessageToTask(void *msg, int timeout) {
	BaseType_t ok;

	ok = xQueueSendToBack(this->messageQueue, msg, timeout);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't send message to task: %u", ok);
		return false;
	}

	return true;
}



/**
 * Converts the byte order of multi-byte fields in an IGMP packet.
 */
void IGMP::convertPacketByteOrder(void *_igmp) {
	igmp_packet_ipv4_t *igmp = (igmp_packet_ipv4_t *) _igmp;

	igmp->checksum = __builtin_bswap16(igmp->checksum);
}

} /* namespace ip */
