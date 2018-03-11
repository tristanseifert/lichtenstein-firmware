/*
 * LichtensteinHandler.cpp
 *
 *  Created on: Mar 10, 2018
 *      Author: tristan
 */
#define LOG_MODULE "LICT"

#include "LichtensteinHandler.h"
#include "lichtenstein_proto.h"

#include <net/Network.h>
#include <net/ip/UDPSocket.h>
#include <net/ip/Stack.h>

#include <sys/System.h>

#include <LichtensteinApp.h>

#include <cstring>


namespace ledout {

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _LichtensteinTaskTrampoline(void *ctx) {
	(static_cast<LichtensteinHandler *>(ctx))->taskEntry();
}

/**
 * Called when the announcement timer expires.
 */
void _DoMulticastAnnouncement(TimerHandle_t timer) {
	int err;

	// get the context out of the timer
	void *ctx = pvTimerGetTimerID(timer);
	LichtensteinHandler *handler = static_cast<LichtensteinHandler *>(ctx);

	// post the message
	err = handler->postMessageToTask(LichtensteinHandler::kSendMulticastDiscovery);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't post message to task");
	}

}

/**
 * Initializes the Lichtenstein protocol handler. This sets up the task and
 * any auxiliary structures.
 */
LichtensteinHandler::LichtensteinHandler() {
	BaseType_t ok;
	int err;

	// create the queue
	this->messageQueue = xQueueCreate(LichtensteinHandler::MessageQueueSize,
			sizeof(message_type_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue!");
	}

	// now, create the task
	ok = xTaskCreate(_LichtensteinTaskTrampoline, "Protocol",
			LichtensteinHandler::TaskStackSize, this,
			LichtensteinHandler::TaskPriority, &this->task);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task!");
	}

	// set up a timer to periodically produce multicast announcements
	this->discoveryTimer = xTimerCreate("Discover",
			LichtensteinHandler::DiscoveryPeriod, pdTRUE, this,
			_DoMulticastAnnouncement);

	if(this->discoveryTimer == nullptr) {
		LOG(S_FATAL, "Couldn't create timer");
	}

	ok = xTimerStart(this->discoveryTimer, portMAX_DELAY);

	if(ok != pdPASS) {
		LOG(S_FATAL, "Couldn't start timer");
	}

	// for good measure, queue a discovery request right off the bat
	err = this->postMessageToTask(LichtensteinHandler::kSendMulticastDiscovery);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't post message to task");
	}
}

/**
 * Terminates the task and releases memory.
 */
LichtensteinHandler::~LichtensteinHandler() {
	// delete discovery timer
	if(this->discoveryTimer) {
		xTimerDelete(this->discoveryTimer, portMAX_DELAY);
	}

	// delete task and queue
	if(this->task) {
		vTaskDelete(this->task);
	}

	if(this->messageQueue) {
		vQueueDelete(this->messageQueue);
	}

	// kill socket
	if(this->sock != nullptr) {
		this->tearDownSocket();
	}
}



/**
 * Task entry point
 */
void LichtensteinHandler::taskEntry() {
	int err;
	void *buffer;
	size_t bytesRead;

	BaseType_t ok;
	message_type_t msg;

	// set up the socket
	this->setUpSocket();

	// message loop
	while(1) {
		// read from the socket
		err = this->sock->receive(&buffer, &bytesRead, LichtensteinHandler::ReceiveTimeout);

		// was the error a timeout?
		if(err == ip::Socket::ErrTimeout) {
			// if so, skip processing
			goto readMessages;
		}
		// was it another error?
		else if(err != ip::Socket::ErrSuccess) {
			// log the error and read message queue
			LOG(S_ERROR, "Error reading socket: %d", err);

			goto readMessages;
		}



		// process packet
		LOG(S_DEBUG, "Received %u bytes", bytesRead);


		// clean up and release packet
doneProcessing: ;
		this->sock->discardRx(buffer);



readMessages: ;
		// read from the message buffer and handle request
		ok = xQueueReceive(this->messageQueue, &msg, 0);

		if(ok == pdPASS) {
			this->taskHandleRequest(msg);
		}
	}

	// we shouldn't get down here but clean up anyways
	this->tearDownSocket();
}



/**
 * Posts a message to the task.
 *
 * @param msg Message to send
 * @return 0 if successful, an error code otherwise.
 */
int LichtensteinHandler::postMessageToTask(message_type_t msg, int timeout) {
	BaseType_t ok;

	ok = xQueueSendToBack(this->messageQueue, &msg, timeout);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't send message to task: %u", ok);
		return 1;
	}

	return 0;
}

/**
 * Handles a message put on the message queue.
 *
 * @param msg Message
 */
void LichtensteinHandler::taskHandleRequest(message_type_t msg) {
	LOG(S_DEBUG, "Received message: %u", msg);

	// handle the message
	switch(msg) {
		case kSendMulticastDiscovery: {
			this->taskSendMulticastDiscovery();
			break;
		}
	}
}

/**
 * Sends a multicast discovery packet.
 */
void LichtensteinHandler::taskSendMulticastDiscovery(void) {
	int err;
	void *buffer;

	// get pointer to stack and hostname length
	const ip::Stack *stack = Network::sharedInstance()->getStack();

	size_t hostnameLen = strlen(stack->getHostname());

	// get a TX buffer
	size_t bytes = sizeof(lichtenstein_node_announcement_t);
	bytes += hostnameLen;

	err = this->sock->prepareTx(&buffer, bytes);

	if(err != ip::Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't get TX buffer: %d", err);
		return;
	}

	memset(buffer, 0, bytes);

	// populate packet
	lichtenstein_node_announcement_t *packet;
	packet = (lichtenstein_node_announcement_t *) buffer;

	packet->swVersion = 0x0000100;
	packet->hwVersion = 0x0000100;

	memcpy(packet->macAddr, stack->getMacAddress().bytes, 6);

	packet->port = LichtensteinHandler::Port;
	packet->ip = stack->getIPAddress();

	packet->fbSize = (300 * 4);
	packet->channels = 2;

	packet->hostnameLen = (uint16_t) hostnameLen;
	strncpy((char *) &packet->hostname, stack->getHostname(), hostnameLen);

	// fill out the header
	packet->header.payloadLength = sizeof(lichtenstein_node_announcement_t);
	packet->header.payloadLength -= sizeof(lichtenstein_header_t);
	packet->header.payloadLength += hostnameLen;

	packet->header.flags |= kFlagMulticast;

	this->populateLichtensteinHeader(&packet->header, kOpcodeNodeAnnouncement);

	// byteswap and insert checksum
	this->packetHostToNetwork(packet);

	packet->header.checksum = this->calculatePacketCRC(packet, bytes);
	packet->header.checksum = __builtin_bswap32(packet->header.checksum);

	// transmit the packet
	err = this->sock->sendTo(buffer, LichtensteinHandler::MulticastGroup,
			LichtensteinHandler::Port);

	if(err != ip::Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't send node discovery: %d", err);

		return;
	}
}

/**
 * Populates the header of a Lichtenstein packet.
 *
 * @param _hdr Pointer to either lichtenstein_header_t or another packet struct
 * that has the header as its first element.
 * @param opcode Opcode to insert into the packet.
 */
void LichtensteinHandler::populateLichtensteinHeader(void *_hdr, uint16_t opcode) {
	lichtenstein_header_t *header = (lichtenstein_header_t *) _hdr;

	// insert magic, version, and opcode
	header->magic = kLichtensteinMagic;
	header->version = kLichtensteinVersion10;

	header->opcode = opcode;

	// we don't really care about sequences
	header->sequenceIndex = 0;
	header->sequenceNumPackets = 0;

	header->txn = System::sharedInstance()->random();

	// set a checksum flag
	header->flags |= kFlagChecksummed;
	header->checksum = 0;
}

/**
 * Calculates the checksum for the entire packet. The checksum is calculated
 * starting with the `opcode` field of the header, until the last byte of the
 * message.
 *
 * Since this should only be called once the packet is in network byte order,
 * if we need to read any fields, we byteswap them.
 *
 * @param _packet Pointer to packet
 * @param length Total bytes in the buffer pointed to by `_packet`
 * @return CRC32 to insert into the packet
 */
uint32_t LichtensteinHandler::calculatePacketCRC(void *_packet, size_t length) {
	// extract some header info
	lichtenstein_header_t *header = (lichtenstein_header_t *) _packet;
	size_t payloadLen = __builtin_bswap32(header->payloadLength);

	// get CRC offset into the packet
	size_t offset = offsetof(lichtenstein_node_announcement_t, header.opcode);
	void *ptr = ((uint8_t *) _packet) + offset;
	size_t len = sizeof(lichtenstein_header_t) - offset + payloadLen;

	// TODO: implement
	return 0;
}

/**
 * Swaps all multibyte fields in a packet.
 *
 * TODO: Add error checking
 *
 * @param _packet Packet
 * @param fromNetworkorder Set if the packet is in network order
 */
void LichtensteinHandler::convertPacketByteOrder(void *_packet, bool fromNetworkOrder) {
	lichtenstein_header_opcode_t opcode;

	// first, process the header
	lichtenstein_header_t *header = (lichtenstein_header_t *) _packet;

	header->magic = __builtin_bswap32(header->magic);
	header->version = __builtin_bswap32(header->version);
	header->checksum = __builtin_bswap32(header->checksum);

	if(fromNetworkOrder) {
		header->opcode = __builtin_bswap16(header->opcode);
		opcode = (lichtenstein_header_opcode_t) header->opcode;
	} else {
		opcode = (lichtenstein_header_opcode_t) header->opcode;
		header->opcode = __builtin_bswap16(header->opcode);
	}

	header->flags = __builtin_bswap16(header->flags);

	header->sequenceIndex = __builtin_bswap16(header->sequenceIndex);
	header->sequenceNumPackets = __builtin_bswap16(header->sequenceNumPackets);

	header->txn = __builtin_bswap32(header->txn);
	header->payloadLength = __builtin_bswap32(header->payloadLength);

	// handle each packet type individually
	switch(opcode) {
		case kOpcodeNodeAnnouncement: {
			lichtenstein_node_announcement_t *announce;
			announce = (lichtenstein_node_announcement_t *) _packet;

			// byteswap all fields
			announce->swVersion = __builtin_bswap32(announce->swVersion);
			announce->hwVersion = __builtin_bswap32(announce->hwVersion);

			announce->port = __builtin_bswap16(announce->port);
			// don't byteswap IP, it's already in network byte order

			announce->fbSize = __builtin_bswap32(announce->fbSize);
			announce->channels = __builtin_bswap16(announce->channels);

			announce->numGpioDigitalIn = __builtin_bswap16(announce->numGpioDigitalIn);
			announce->numGpioDigitalOut = __builtin_bswap16(announce->numGpioDigitalOut);
			announce->numGpioAnalogIn = __builtin_bswap16(announce->numGpioAnalogIn);
			announce->numGpioAnalogOut = __builtin_bswap16(announce->numGpioAnalogOut);

			announce->hostnameLen = __builtin_bswap16(announce->hostnameLen);
			break;
		}

		default: {
			LOG(S_ERROR, "Unknown packet type %u", opcode);
			break;
		}
	}
}



/**
 * Sets up the socket for the protocol.
 */
void LichtensteinHandler::setUpSocket(void) {
	int err;

	// set up the socket
	this->sock = Network::getUDPSocket();

	if(this->sock == nullptr) {
		LOG(S_FATAL, "Couldn't create socket");
	}

	// open socket
	err = this->sock->open();

	if(err != 0) {
		LOG(S_FATAL, "Couldn't open socket: %d", err);
	}

	// bind the socket
	err = this->sock->bind(LichtensteinHandler::Port);

	if(err != 0) {
		LOG(S_FATAL, "Couldn't bind port: %d", err);
	}

	// join multicast group
	stack_ipv4_addr_t addr = LichtensteinHandler::MulticastGroup;

	err = this->sock->setSockOpt(ip::Socket::kSocketProtocolIPv4,
			ip::Socket::kSockOptJoinMulticast, &addr, sizeof(addr));

	if(err != 0) {
		LOG(S_FATAL, "Couldn't join multicast group: %d", err);
	}
}

/**
 * Tears down the socket and any resources allocated with it.
 */
void LichtensteinHandler::tearDownSocket(void) {
	int err;

	// leave multicast group
	stack_ipv4_addr_t addr = LichtensteinHandler::MulticastGroup;

	err = this->sock->setSockOpt(ip::Socket::kSocketProtocolIPv4,
			ip::Socket::kSockOptLeaveMulticast, &addr, sizeof(addr));

	if(err != 0) {
		LOG(S_FATAL, "Couldn't leave multicast group: %d", err);
	}

	// close socket
	this->sock->close();

	// finally, delete it
	delete this->sock;
	this->sock = nullptr;
}

} /* namespace ledout */
