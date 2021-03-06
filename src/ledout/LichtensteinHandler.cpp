/*
 * LichtensteinHandler.cpp
 *
 *  Created on: Mar 10, 2018
 *      Author: tristan
 */
#define LOG_MODULE "LICT"
#define LICHTENSTEIN_PRIVATE
#define OUTPUTTASK_PRIVATE

#include "lichtenstein_proto.h"
#include "OutputTaskPrivate.h"

#include "LichtensteinHandler.h"

#include "Output.h"
#include "OutputTask.h"

#include <board/Board.h>

#include <net/Network.h>
#include <net/ip/UDPSocket.h>
#include <net/ip/Stack.h>

#include <sys/System.h>

#include <crc32/Crc32.h>

#include <LichtensteinApp.h>

#include <cstring>



// set to use hardware CRC calculation
#define USE_HARDWARE_CRC					0

// log received packets
#define LOG_RECEIVED_PACKETS				0
// produce logging when the adoption state changes
#define LOG_ADOPTION						1



namespace ledout {

/**
 * Called when the abandonment timer expires. This just calls into the class
 * method to handle it.
 *
 * @param timer Timer that expired
 */
void _AbandonTimerCallback(TimerHandle_t timer) {
	// get the context out of the timer
	void *ctx = pvTimerGetTimerID(timer);
	LichtensteinHandler *handler = static_cast<LichtensteinHandler *>(ctx);

	// call the method
	handler->abandonTimerFired();
}

/**
 * Called when a buffer has been completed. This will discard the received
 * buffer with the socket.
 *
 * @param h Pointer to the LichtensteinHandler instance
 * @param rxBuffer Receive buffer
 */
void _ConversionCompleteCallback(void *h, void *rxBuffer) {
	LichtensteinHandler *handler = static_cast<LichtensteinHandler *>(h);

	lichtenstein_framebuffer_data_t *data;
	data = (lichtenstein_framebuffer_data_t *) rxBuffer;

	// acknowledge the message
	handler->ackPacket(&data->header);

	// discard RX buffer
	handler->sock->discardRx(rxBuffer);
}

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
		LOG(S_ERROR, "Couldn't post message to task: %u", err);
	}

}

/**
 * Initializes the Lichtenstein protocol handler. This sets up the task and
 * any auxiliary structures.
 */
LichtensteinHandler::LichtensteinHandler() {
	BaseType_t ok;
	int err;

	// set up the CRC peripheral
#if USE_HARDWARE_CRC
	this->setUpCRC();
#endif

	// create the queue
	this->messageQueue = xQueueCreate(LichtensteinHandler::MessageQueueSize,
			sizeof(message_type_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue");
	}

	// now, create the task
	ok = xTaskCreate(_LichtensteinTaskTrampoline, "Protocol",
			LichtensteinHandler::TaskStackSize, this,
			LichtensteinHandler::TaskPriority, &this->task);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task");
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

	// clear buffers
	memset(&this->numOutputLEDs, 0, sizeof(this->numOutputLEDs));
}

/**
 * Terminates the task and releases memory.
 */
LichtensteinHandler::~LichtensteinHandler() {
	// delete discovery timer
	if(this->discoveryTimer) {
		xTimerDelete(this->discoveryTimer, portMAX_DELAY);
	}
	// delete abandonment timer
	if(this->abandonTimer) {
		xTimerDelete(this->abandonTimer, portMAX_DELAY);
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

	// clean up CRC
#if USE_HARDWARE_CRC
	this->cleanUpCRC();
#endif
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
	lichtenstein_header_t *hdr;

	uint32_t crc;

	bool releasePacket = true;

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
		// did we get a null buffer?
		else if(buffer == nullptr) {
			goto readMessages;
		}



		// check CRC of packet
		hdr = (lichtenstein_header_t *) buffer;

		crc = this->calculatePacketCRC(hdr, bytesRead);

		if(crc != __builtin_bswap32(hdr->checksum)) {
			// if the checksum was invalid, increment the counter
			this->invalidCRCErrors++;

			LOG(S_INFO, "Discarded packet with invalid checksum (0x%08x, expected 0x%08x)",
					__builtin_bswap32(hdr->checksum), crc);

			goto doneProcessing;
		}

		// byteswap all fields in the packet
		err = this->packetNetworkToHost(buffer, bytesRead);

#if LOG_RECEIVED_PACKETS
		LOG(S_DEBUG, "Received %u bytes", bytesRead);
#endif

		if(err != 0) {
			LOG(S_INFO, "Couldn't byteswap packet (%u bytes)", bytesRead);
			goto doneProcessing;
		}

		// make sure that the magic matches
		if(hdr->magic != kLichtensteinMagic) {
			LOG(S_INFO, "Invalid magic: 0x%08x", hdr->magic);
			goto doneProcessing;
		}

		// handle packet based on the opcode
		switch(hdr->opcode) {
			// adoption
			case kOpcodeNodeAdoption: {
				lichtenstein_node_adoption_t *adopt;
				adopt = (lichtenstein_node_adoption_t *) buffer;

				releasePacket = this->taskHandleAdoption(adopt);
				break;
			}

			// framebuffer data received
			case kOpcodeFramebufferData: {
				// ignore if not adopted
				if(!this->isAdopted) {
					goto doneProcessing;
				}

				// reset the abandonment timer
				if(this->abandonTimer) {
					xTimerReset(this->abandonTimer, portMAX_DELAY);
				}

				lichtenstein_framebuffer_data_t *fb;
				fb = (lichtenstein_framebuffer_data_t *) buffer;

				releasePacket = this->taskHandleFBData(fb);
				break;
			}
			// output a particular channel
			case kOpcodeSyncOutput: {
				// ignore if not adopted
				if(!this->isAdopted) {
					goto doneProcessing;
				}

				// reset the abandonment timer
				if(this->abandonTimer) {
					xTimerReset(this->abandonTimer, portMAX_DELAY);
				}

				lichtenstein_sync_output_t *sync;
				sync = (lichtenstein_sync_output_t *) buffer;

				releasePacket = this->taskHandleSyncOut(sync);
				break;
			}

			// ignored opcodes
			case kOpcodeNodeAnnouncement:
				break;

			// unknown opcode (should never happen)
			default: {
				LOG(S_ERROR, "Invalid opcode: %u", hdr->opcode);

				// release packet
				releasePacket = true;
				break;
			}
		}


		// clean up and release packet
doneProcessing: ;
		if(releasePacket) {
			this->sock->discardRx(buffer);
		}


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
 * Handles a framebuffer data packet.
 *
 * @param packet Pointer to packet
 *
 * @return true if the packet should be released, false otherwise.
 */
bool LichtensteinHandler::taskHandleFBData(lichtenstein_framebuffer_data_t *packet) {
	int err;

	// prepare the message
	output_message_t msg;
	memset(&msg, 0, sizeof(output_message_t));

	// fill in type, channel
	msg.type = kOutputMessageConvert;
	msg.channel = packet->destChannel;

	// set pointer to data and buffer length
	msg.payload.convert.buffer = &packet->data;

	msg.payload.convert.isRGBW = (packet->dataFormat == kDataFormatRGBW);
	msg.payload.convert.numLEDs = packet->dataElements;

	// set callback
	msg.callback = _ConversionCompleteCallback;
	msg.cbContext1 = this;
	msg.cbContext2 = packet;

	// send it
	err = Output::sharedInstance()->task->sendMessage(&msg);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't convert buffer");

		return true;
	}

	// if we get here, the send was successful so keep the buffer around
	return false;
}

/**
 * Handles a received "sync output" packet
 *
 * @param  packet Pointer to packet
 *
 * @return true if the packet should be released, false otherwise.
 */
bool LichtensteinHandler::taskHandleSyncOut(lichtenstein_sync_output_t *packet) {
	int err;

	// prepare the message
	output_message_t msg;
	memset(&msg, 0, sizeof(output_message_t));

	// fill in type, channel
	msg.type = kOutputMessageSend;
	msg.channel = 0;

	msg.payload.send.channelBitmask = packet->channel;

	// send it
	err = Output::sharedInstance()->task->sendMessage(&msg);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't output buffer");
	}

	// always get rid of the received packet
	return true;
}



/**
 * Handles a received node adoption packet.
 *
 * @param packet Location of the received packet
 *
 * @return true if the packet should be released, false otherwise.
 */
bool LichtensteinHandler::taskHandleAdoption(lichtenstein_node_adoption_t *packet) {
	int err;

	// return if this node is already adopted
	if(this->isAdopted) {
		return true;
	}

	// since we received this frame, we've been adopted
	this->isAdopted = true;

	// copy the server's IP and port
	this->serverAddr = (stack_ipv4_addr_t) packet->ip;
	this->serverPort = packet->port;

	// process flags

	// copy the number of LEDs per channel
	for(size_t i = 0; i < packet->numChannels; i++) {
		this->numOutputLEDs[i] = packet->pixelsPerChannel[i];
	}

	// acknowledge packet
	err = this->ackPacket(&packet->header);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't acknowledge adoption: %d", err);
	}

	// stop the discovery timer
	xTimerStop(this->discoveryTimer, portMAX_DELAY);

	// create abandonment timer
	if(this->abandonTimer) {
		// delete old timer if it exists
		xTimerDelete(this->abandonTimer, portMAX_DELAY);
		this->abandonTimer = nullptr;
	}

	this->abandonTimer = xTimerCreate("AdoptAbandon",
			LichtensteinHandler::AdoptionAbandonPeriod, false,
			this, _AbandonTimerCallback);

	if(this->abandonTimer == nullptr) {
		LOG(S_ERROR, "Couldn't create abandonment timer");
	}

	xTimerStart(this->abandonTimer, portMAX_DELAY);

	// logging
#if LOG_ADOPTION
	char ipStr[16];
	ip::Stack::ipToString(this->serverAddr, ipStr, 16);
	LOG(S_INFO, "Adopted by %s:%u", ipStr, this->serverPort);
#endif

	// always release the packet
	return true;
}

/**
 * Called when the abandonment timer fires. This fires five minutes after the
 * last reception of an unicast packet.
 */
void LichtensteinHandler::abandonTimerFired(void) {
	int err;

#if LOG_ADOPTION
	LOG(S_WARN, "Abandonment timer fired, resuming multicast discovery");
#endif

	// immediately send a discovery message
	err = this->postMessageToTask(LichtensteinHandler::kSendMulticastDiscovery);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't post message to task: %u", err);
	}

	// re-start the discovery timer
	xTimerReset(this->discoveryTimer, portMAX_DELAY);
	xTimerStart(this->discoveryTimer, portMAX_DELAY);

	// clear adoption flag and server information
	this->isAdopted = false;

	this->serverAddr = kIPv4AddressZero;
	this->serverPort = 0;
}



/**
 * Acknowledges the given packet, if required.
 *
 * @param header Header of the packet, in host byte order
 * @param nack When set, issue a negetive acknowledgement.
 *
 * @return 0 if successful, an error code otherwise.
 */
int LichtensteinHandler::ackPacket(lichtenstein_header_t *header, bool nack) {
	int err;
	void *buffer;

	// return if the packet doesn't require an ack
	if(header->opcode != kOpcodeFramebufferData
			&& header->opcode == kOpcodeSyncOutput) {
		return -1;
	}

	// return if the IP is zero
	if(this->isAdopted == false) {
		LOG(S_ERROR, "Can't acknowledge packet if not adopted");
		return -2;
	}

	// get a TX buffer
	const size_t bytes = sizeof(lichtenstein_header_t);

	err = this->sock->prepareTx(&buffer, bytes);

	if(err != ip::Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't get TX buffer: %d", err);
		return err;
	}

	memset(buffer, 0, bytes);

	// populate header
	lichtenstein_header_t *packet = (lichtenstein_header_t *) buffer;

	this->populateLichtensteinHeader(packet, header->opcode);

	packet->txn = header->txn;

	packet->payloadLength = 0;

	packet->flags |= kFlagChecksummed;
	packet->flags |= nack ? kFlagNAck : kFlagAck;

	// byteswap and insert checksum
	this->packetHostToNetwork(packet, bytes);

	packet->checksum = this->calculatePacketCRC(packet, bytes);
	packet->checksum = __builtin_bswap32(packet->checksum);

	// transmit the packet
	err = this->sock->sendTo(buffer, this->serverAddr, this->serverPort);

	if(err != ip::Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't send ack: %d", err);

		return err;
	}

	// assume success
	return 0;
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
//	LOG(S_DEBUG, "Received message: %u", msg);

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

	LOG(S_DEBUG, "Sending multicast discovery");

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

	packet->swVersion = 0x00001000;

	uint32_t hwVersion = 0;
	hwVersion |= Board::sharedInstance()->getConfig()->hwModel << 24;
	hwVersion |= Board::sharedInstance()->getConfig()->hwVersion << 16;
	hwVersion |= Board::sharedInstance()->getConfig()->hwRevision << 8;
	packet->hwVersion = hwVersion;

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
	err = this->packetHostToNetwork(packet, bytes);

	if(err != 0) {
		LOG(S_FATAL, "Couldn't byteswap discovery packet: %d", err);
	}

	packet->header.checksum = this->calculatePacketCRC(&packet->header, bytes);
	packet->header.checksum = __builtin_bswap32(packet->header.checksum);

	// set TTL to 1
	int ttl = 1;
	err = this->sock->setSockOpt(ip::Socket::kSocketProtocolIPv4, ip::Socket::kSockOptIPTTL, &ttl, sizeof(ttl));
	if(err != ip::Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't set TTL: %d", err);
		this->sock->discardTx(buffer);
		return;
	}

	// send the packet
	err = this->sock->sendTo(buffer, LichtensteinHandler::MulticastGroup,
			LichtensteinHandler::Port);

	if(err != ip::Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't send node discovery: %d", err);
		return;
	}

	// restore TTL
	ttl = -1;
	err = this->sock->setSockOpt(ip::Socket::kSocketProtocolIPv4, ip::Socket::kSockOptIPTTL, &ttl, sizeof(ttl));
	if(err != ip::Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't set TTL: %d", err);
		this->sock->discardTx(buffer);
		return;
	}
}

/**
 * Populates the header of a Lichtenstein packet.
 *
 * @param header Pointer to either lichtenstein_header_t or another packet struct
 * that has the header as its first element.
 * @param opcode Opcode to insert into the packet.
 */
void LichtensteinHandler::populateLichtensteinHeader(lichtenstein_header_t *header, uint16_t opcode) {
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
 * @param header Pointer to packet
 * @param length Total bytes in the buffer pointed to by `_packet`
 * @return CRC32 to insert into the packet
 */
uint32_t LichtensteinHandler::calculatePacketCRC(lichtenstein_header_t *header, size_t length) {
	// get CRC offset into the packet
	size_t offset = offsetof(lichtenstein_header_t, opcode);
	void *ptr = ((uint8_t *) header) + offset;
	size_t len = (length - offset);

//	LOG(S_DEBUG, "Checksum over %u bytes of packet: checksumming %u bytes", length, len);

	// calculate CRC
#if USE_HARDWARE_CRC
	return this->doHWCRC(ptr, len);
#else
	return this->doSWCRC(ptr, len);
#endif
}

/**
 * Swaps all multibyte fields in a packet.
 *
 * TODO: Add error checking
 *
 * @param _packet Packet
 * @param fromNetworkorder Set if the packet is in network order
 * @param length Total number of bytes in packet
 *
 * @return 0 if the conversion was a success, error code otherwise.
 */
int LichtensteinHandler::convertPacketByteOrder(void *_packet, bool fromNetworkOrder, size_t length) {
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

	// return immediately if payload length is zero (its a request)
	if(header->payloadLength == 0) {
		return 0;
	}

	// handle each packet type individually
	switch(opcode) {
		// node announcement
		case kOpcodeNodeAnnouncement: {
			// ensure the length is correct
			if(length < sizeof(lichtenstein_node_announcement_t)) {
				return -1;
			}

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
		// server announcement
		case kOpcodeServerAnnouncement: {
			// ensure the length is correct
			if(length < sizeof(lichtenstein_server_announcement_t)) {
				return -1;
			}

			lichtenstein_server_announcement_t *announce;
			announce = (lichtenstein_server_announcement_t *) _packet;

			// byteswap all fields
			announce->swVersion = __builtin_bswap32(announce->swVersion);
			announce->capabilities = __builtin_bswap32(announce->capabilities);

			announce->port = __builtin_bswap16(announce->port);
			announce->hostnameLen = __builtin_bswap16(announce->hostnameLen);

			break;
		}

		// node status
		case kOpcodeNodeStatusReq: {
			// ensure the length is correct
			if(length < sizeof(lichtenstein_node_status_t)) {
				return -1;
			}

			lichtenstein_node_status_t *status;
			status = (lichtenstein_node_status_t *) _packet;

			// byteswap all fields
			status->uptime = __builtin_bswap32(status->uptime);

			status->totalMem = __builtin_bswap32(status->totalMem);
			status->freeMem = __builtin_bswap32(status->freeMem);

			status->rxPackets = __builtin_bswap32(status->rxPackets);
			status->txPackets = __builtin_bswap32(status->txPackets);
			status->packetsWithInvalidCRC = __builtin_bswap32(status->packetsWithInvalidCRC);

			status->framesOutput = __builtin_bswap32(status->framesOutput);

			status->outputState = __builtin_bswap16(status->outputState);
			status->cpuUsagePercent = __builtin_bswap16(status->cpuUsagePercent);

			status->avgConversionTimeUs = __builtin_bswap32(status->avgConversionTimeUs);

			status->rxBytes = __builtin_bswap32(status->rxBytes);
			status->txBytes = __builtin_bswap32(status->txBytes);

			status->rxSymbolErrors = __builtin_bswap32(status->rxSymbolErrors);

			status->mediumSpeed = __builtin_bswap32(status->mediumSpeed);
			status->mediumDuplex = __builtin_bswap32(status->mediumDuplex);

			break;
		}


		// framebuffer data
		case kOpcodeFramebufferData: {
			// ensure the length is correct
			if(length < sizeof(lichtenstein_framebuffer_data_t)) {
				return -1;
			}

			lichtenstein_framebuffer_data_t *fb;
			fb = (lichtenstein_framebuffer_data_t *) _packet;

			fb->destChannel = __builtin_bswap32(fb->destChannel);

			fb->dataFormat = __builtin_bswap32(fb->dataFormat);
			fb->dataElements = __builtin_bswap32(fb->dataElements);
			break;
		}
		// output command
		case kOpcodeSyncOutput: {
			// ensure the length is correct
			if(length < sizeof(lichtenstein_sync_output_t)) {
				return -1;
			}

			lichtenstein_sync_output_t *out;
			out = (lichtenstein_sync_output_t *) _packet;

			out->channel = __builtin_bswap32(out->channel);
			break;
		}

		// node adoption
		case kOpcodeNodeAdoption: {
			size_t numChannels = 0;

			// ensure the length is correct
			if(length < sizeof(lichtenstein_node_adoption_t)) {
				return -1;
			}

			lichtenstein_node_adoption_t *adopt;
			adopt = (lichtenstein_node_adoption_t *) _packet;

			// byteswap regular fields
			adopt->port = __builtin_bswap16(adopt->port);
			adopt->flags = __builtin_bswap16(adopt->flags);

			if(fromNetworkOrder) {
				adopt->numChannels = __builtin_bswap32(adopt->numChannels);
				numChannels = adopt->numChannels;
			} else {
				numChannels = adopt->numChannels;
				adopt->numChannels = __builtin_bswap32(adopt->numChannels);
			}

			// byteswap the pixels per channel array
			for(size_t i = 0; i < numChannels; i++) {
				adopt->pixelsPerChannel[i] = __builtin_bswap32(adopt->pixelsPerChannel[i]);
			}

			break;
		}

		// reconfig
		case kOpcodeNodeReconfig: {
			// ensure the length is correct
			if(length < sizeof(lichtenstein_node_adoption_t)) {
				return -1;
			}

			lichtenstein_reconfig_t *adopt;
			adopt = (lichtenstein_reconfig_t *) _packet;

			break;
		}

		// should never get here
		default: {
			LOG(S_ERROR, "Unknown packet type %u", opcode);
			return -1;
		}
	}

	// if we get down here, conversion was a success
	return 0;
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



/**
 * Sets up the CRC peripheral.
 */
void LichtensteinHandler::setUpCRC(void) {
	// enable the clock for the CRC peripheral and reset it
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	CRC_ResetDR();
}

/**
 * Resets and disables the CRC peripheral.
 */
void LichtensteinHandler::cleanUpCRC(void) {
	// reset the CRC then disable its clock
	CRC_ResetDR();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);
}

/**
 * Using the hardware CRC peripheral, calculates a CRC over the given input
 * data.
 *
 * @note The hardware CRC polynomial is 0x4C11DB7.
 *
 * @param data Address of data
 * @param length Number of bytes to compute the CRC for
 * @return CRC
 */
uint32_t LichtensteinHandler::doHWCRC(void *data, size_t length) {
	// reset the CRC
	CRC_ResetDR();

	// data length MUST be a multiple of 4 bytes
	if((length & 3) != 0) {
		LOG(S_ERROR, "Packet length must be a multiple of 4!");
		return 0;
	}

	// continuously write to the CRC unit
	uint32_t *ptr = (uint32_t *) data;

	for(size_t i = 0; i < length; i += 4) {
		CRC->DR = *ptr++;
	}

	// return the CRC
	return CRC->DR;
}

/**
 * Calculates a CRC over the given input data using a software method.
 *
 * @param data Address of data
 * @param length Number of bytes to compute the CRC for
 * @return CRC
 */
uint32_t LichtensteinHandler::doSWCRC(void *data, size_t length) {
	return crc32_fast(data, length);
}

} /* namespace ledout */
