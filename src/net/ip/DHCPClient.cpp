/*
 * DHCPClient.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: tristan
 */
#define LOG_MODULE "DHCP"

#include "DHCPClient.h"
#include "DHCPClientPrivate.h"

#include "Stack.h"

//#include "UDP.h"
#include "UDPSocket.h"
#include "../Network.h"

#include <LichtensteinApp.h>

// log state transitions
#define LOG_STATE_TRANSITIONS				1
// produce logs for the DHCP transaction
#define LOG_DHCP_TRANSACTIONS				1



namespace ip {

/**
 * Jumps into the task entry point for the DHCP client.
 */
void  _DHCPClientTaskTrampoline(void *ctx) {
	(static_cast<DHCPClient *>(ctx))->taskEntry();
}

/**
 * Initializes the DHCP client.
 */
DHCPClient::DHCPClient(Stack *_stack) : stack(_stack) {
	BaseType_t ok;

	// set up the task
	ok = xTaskCreate(_DHCPClientTaskTrampoline, "DHCP",
			DHCPClient::taskStackSize, this, DHCPClient::taskPriority,
			&this->task);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task!");
	}

	// set up state change mutex
	this->stateChangeMutex = xSemaphoreCreateMutex();
	xSemaphoreGive(this->stateChangeMutex);
}

/**
 * Stops the DHCP client and any timeout timers.
 */
DHCPClient::~DHCPClient() {
	// clean up task and timer
	if(this->timeoutTimer) {
		xTimerDelete(this->timeoutTimer, portMAX_DELAY);
		this->timeoutTimer = nullptr;
	}

	if(this->task) {
		vTaskDelete(this->task);
		this->task = nullptr;
	}

	if(this->stateChangeMutex) {
		vSemaphoreDelete(this->stateChangeMutex);
		this->stateChangeMutex = nullptr;
	}
}



/**
 * Starts a DHCP request.
 */
void DHCPClient::requestIP(void) {
	// ensure we're in the idle state
	if(this->state != IDLE) {
		LOG(S_ERROR, "Attempting to start DHCP session in state %u", this->state);
		return;
	}

	// change state
	this->changeState(REQUEST);
}

/**
 * Resets the DHCP state machine.
 */
void DHCPClient::reset(void) {
	this->changeState(IDLE);
}



/**
 * Entry point for the DHCP task
 */
int DHCPClient::taskEntry(void) {
	BaseType_t ok;
	int err;
	UDPSocket *sock;

	// set up the socket
	sock = Network::getUDPSocket();

	if(sock == nullptr) {
		LOG(S_ERROR, "Couldn't create DHCP socket!");
		return -1;
	}

	// bind the socket
	err = sock->bind(DHCP_CLIENT_PORT);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't bind on DHCP port");
		return -1;
	}

	// receive broadcast
	bool yes = true;

	err = sock->setSockOpt(Socket::kSocketProtocolUDP,
			Socket::kSockOptAcceptBroadcast, &yes, sizeof(bool));

	if(err != 0) {
		LOG(S_ERROR, "Couldn't enable broadcast reception: %d", err);
		return -1;
	}


	// enter the message loop
	while(1) {
		// wait for a state change
		ok = xSemaphoreTake(this->stateChangeMutex, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG(S_ERROR, "Couldn't take state change mutex: %u", ok);
			continue;
		}

		// handle the state
		switch(this->state) {
			// timeout waiting for response
			case TIMEOUT: {
				LOG(S_ERROR, "Timeout waiting for DHCP response");

				this->changeState(IDLE);
				break;
			}

			// send a DHCPDISCOVER
			case DISCOVER: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Sending DHCPOFFER");
#endif

				// get a TX buffer

				// send a TX buffer

				// wait for a DHCPOFFER
				this->changeState(WAITOFFER);
				break;
			}

			// wait to receive an offer
			case WAITOFFER: {
				void *buffer;
				size_t bytesRead;

				err = sock->receive(&buffer, &bytesRead,
						DHCPClient::receiveTimeout);

				// was the error a timeout?
				if(err == Socket::ErrTimeout) {
					this->changeState(TIMEOUT);
					continue;
				}
				// handle other read errors by retrying the read
				else if(err != Socket::ErrSuccess) {
					LOG(S_ERROR, "Error reading socket: %d", err);

					// retry reading
					this->changeState(this->state);
					continue;
				}

				// send a request
				this->changeState(REQUEST);

				break;
			}

			// send a DHCP request
			case REQUEST: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Sending DHCPREQUEST");
#endif

				// get a TX buffer

				// send a TX buffer

				// wait for a DHCPACK
				this->changeState(WAITACK);
				break;
			}

			// wait for an acknowledgement
			case WAITACK: {
				void *buffer;
				size_t bytesRead;

				err = sock->receive(&buffer, &bytesRead,
						DHCPClient::receiveTimeout);

				// was the error a timeout?
				if(err == Socket::ErrTimeout) {
					this->changeState(TIMEOUT);
					continue;
				}
				// handle other read errors by retrying the read
				else if(err != Socket::ErrSuccess) {
					LOG(S_ERROR, "Error reading socket: %d", err);

					// retry reading
					this->changeState(this->state);
					continue;
				}

				// change IP config

				this->stack->ipConfigBecameValid();

				// go back to the idle state
#if LOG_DHCP_TRANSACTIONS
				LOG(S_DEBUG, "Finished DHCP transaction");
#endif
				this->changeState(IDLE);

				break;
			}


			// idle state: do nothing
			case IDLE: {
				break;
			}
			// default (should never get here)
			default:
				LOG(S_ERROR, "Invalid state %u", this->state);

				this->changeState(IDLE);
				break;
		}
	}

	// close the socket

	return 0;
}

} /* namespace ip */
