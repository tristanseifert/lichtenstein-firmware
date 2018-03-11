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

#include <LichtensteinApp.h>


namespace ledout {

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _LichtensteinTaskTrampoline(void *ctx) {
	(static_cast<LichtensteinHandler *>(ctx))->taskEntry();
}

/**
 * Initializes the Lichtenstein protocol handler. This sets up the task and
 * any auxiliary structures.
 */
LichtensteinHandler::LichtensteinHandler() {
	BaseType_t ok;

	// create the queue
	this->messageQueue = xQueueCreate(LichtensteinHandler::messageQueueSize,
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
}

/**
 * Terminates the task and releases memory.
 */
LichtensteinHandler::~LichtensteinHandler() {
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
 * Handles a message put on the message queue.
 *
 * @param msg Message
 */
void LichtensteinHandler::taskHandleRequest(message_type_t msg) {
	LOG(S_DEBUG, "Received message: %u", msg);
}



/**
 * Sets up the socket for the protocol.
 */
void LichtensteinHandler::setUpSocket(void) {
	int err;

	// set up the socket
	this->sock = Network::getUDPSocket();

	if(this->sock == nullptr) {
		LOG(S_FATAL, "Couldn't create DHCP socket!");
	}

	// open socket
	err = this->sock->open();

	if(err != 0) {
		LOG(S_FATAL, "Couldn't open DHCP socket");
	}

	// bind the socket
	err = this->sock->bind(LichtensteinHandler::Port);

	if(err != 0) {
		LOG(S_FATAL, "Couldn't bind on DHCP port");
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
