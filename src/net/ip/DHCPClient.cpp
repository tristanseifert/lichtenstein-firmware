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

#include <cstdint>
#include <cstring>



// log state transitions
#define LOG_STATE_TRANSITIONS				1
// produce logs for the DHCP transaction
#define LOG_DHCP_TRANSACTIONS				1
// log info about any received DHCP offers
#define LOG_OFFER							1
// output logging for messages relating to the DHCP exiprity timer
#define LOG_RENEWAL_TIMER						1



/**
 * Options to request in a DHCPDISCOVER message:
 *
 * - 53: 1 (DHCP Discover)
 * - 55: 1, 3, 15, 6, 4, 7, 66, 67, 54 (Parameter Request List)
 */
static const uint8_t discoverOptions[] = {
	// DHCP discover
	kDHCPOptionMessageType, 1, kDHCPMessageTypeDiscover,
	// parameter request list
	kDHCPOptionRequestedOptions, 10,
		kDHCPOptionSubnetMask, kDHCPOptionRouter, kDHCPOptionDNS,
		kDHCPOptionsSyslogServer, kDHCPOptionsTFTPServer,
		kDHCPOptionsBootFileName, kDHCPOptionServerAddress,
		kDHCPOptionLeaseTime, kDHCPOptionsNTPServer,
		kDHCPOptionsUTCOffset,

	// end
	kDHCPOptionEnd,
};

/**
 * Options sent in a DHCPREQUEST message:
 *
 * - 50: 0xffffffff (filled in with the offered IP address)
 * - 54: 0xffffffff (filled in with the IP address of the DHCP server)
 * - 53: 3 (DHCP Request)
 */
static const uint8_t requestOptions[] = {
	// DHCP request
	kDHCPOptionMessageType, 1, kDHCPMessageTypeRequest,

	// requested IP address (this is what the offer specified)
	kDHCPOptionRequestedAddress, 4, 0xff, 0xff, 0xff, 0xff,
	// server IP address
	kDHCPOptionServerAddress, 4, 0xff, 0xff, 0xff, 0xff,

	// hostname
	kDHCPOptionsHostname, 0,

	// end
	kDHCPOptionEnd,
};


/**
 * Options sent in a DHCPREQUEST message for renewal
 *
 * - 50: 0xffffffff (filled in with the offered IP address)
 * - 54: 0xffffffff (filled in with the IP address of the DHCP server)
 * - 53: 3 (DHCP Request)
 */
static const uint8_t renewOptions[] = {
	// DHCP discover
	kDHCPOptionMessageType, 1, kDHCPMessageTypeRequest,

	// requested IP address (this is our current address)
	kDHCPOptionRequestedAddress, 4, 0xff, 0xff, 0xff, 0xff,
	// server IP address
	kDHCPOptionServerAddress, 4, 0xff, 0xff, 0xff, 0xff,

	// parameter request list
	kDHCPOptionRequestedOptions, 10,
		kDHCPOptionSubnetMask, kDHCPOptionRouter, kDHCPOptionDNS,
		kDHCPOptionsSyslogServer, kDHCPOptionsTFTPServer,
		kDHCPOptionsBootFileName, kDHCPOptionServerAddress,
		kDHCPOptionLeaseTime, kDHCPOptionsNTPServer,
		kDHCPOptionsUTCOffset,

	// hostname
	kDHCPOptionsHostname, 0,

	// end
	kDHCPOptionEnd,
};

namespace ip {

/**
 * Jumps into the task entry point for the DHCP client.
 */
void  _DHCPClientTaskTrampoline(void *ctx) {
	(static_cast<DHCPClient *>(ctx))->taskEntry();
}

/**
 * DHCP expiry timer
 */
void _DHCPRenewTimeout(TimerHandle_t timer) {
	void *ctx = pvTimerGetTimerID(timer);
	DHCPClient *client = static_cast<DHCPClient *>(ctx);

#if LOG_RENEWAL_TIMER
	LOG(S_DEBUG, "Renewal timer expired");
#endif

	// change into the RENEW state
	client->changeState(DHCPClient::RENEW);
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

	// clear offer info
	memset(&this->offer, 0, sizeof(this->offer));
}

/**
 * Stops the DHCP client and any timeout timers.
 */
DHCPClient::~DHCPClient() {
	// clean up task and timer
	if(this->renewalTimer) {
		xTimerDelete(this->renewalTimer, portMAX_DELAY);
		this->renewalTimer = nullptr;
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

	// clear info from the previous DHCP offer
	memset(&this->offer, 0, sizeof(this->offer));

	// make sure all timers are stopped
	if(this->renewalTimer) {
		xTimerDelete(this->renewalTimer, portMAX_DELAY);
		this->renewalTimer = nullptr;
	}

	// change state
	this->changeState(DISCOVER);
}

/**
 * Resets the DHCP state machine.
 */
void DHCPClient::reset(void) {
	// clear offer info
	memset(&this->offer, 0, sizeof(this->offer));

	// switch to idle state
	this->changeState(IDLE);

	// stop timers
	if(this->renewalTimer) {
		xTimerDelete(this->renewalTimer, portMAX_DELAY);
		this->renewalTimer = nullptr;
	}
}



/**
 * Entry point for the DHCP task
 */
int DHCPClient::taskEntry(void) {
	BaseType_t ok;
	int err;

	// set up the socket
	this->sock = Network::getUDPSocket();

	if(this->sock == nullptr) {
		LOG(S_ERROR, "Couldn't create DHCP socket!");
		return -1;
	}

	// open socket
	err = this->sock->open();

	if(err != 0) {
		LOG(S_ERROR, "Couldn't open DHCP socket");
		return -1;
	}

	// bind the socket
	err = this->sock->bind(DHCP_CLIENT_PORT);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't bind on DHCP port");
		return -1;
	}


	// receive broadcast
	bool yes = true;

	err = this->sock->setSockOpt(Socket::kSocketProtocolUDP,
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
			// send a DHCPDISCOVER
			case DISCOVER: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Sending DHCPDISCOVER");
#endif
				this->taskSendDiscover();
				break;
			}
			// wait to receive an offer
			case WAITOFFER: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Waiting for DHCPOFFER");
#endif
				this->taskHandleOffer();
				break;
			}
			// send a DHCP request
			case REQUEST: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Sending DHCPREQUEST");
#endif
				this->taskSendRequest();
				break;
			}
			// wait for an acknowledgement
			case WAITACK: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Waiting for DHCPACK");
#endif
				this->taskHandleAck();
				break;
			}
			// post-acknowledgement
			case SUCCESS: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Setting IP configuration");
#endif

				this->taskUpdateIPConfig();
				break;
			}

			// renew lease
			case RENEW: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Renewing lease");
#endif

				this->taskRenewLease();
				break;
			}
			// wait for acknowledgement of renewal request
			case WAITRENEWACK: {
#if LOG_STATE_TRANSITIONS
				LOG(S_DEBUG, "Waiting for renewal ack");
#endif

				this->taskRenewWaitAck();
				break;
			}


			// idle state: do nothing
			case IDLE: {
				void *buffer;
				size_t bytesRead;

				// read from the socket (get stuck packets; 100ms timeout)
				err = this->sock->receive(&buffer, &bytesRead,
						(100 / portTICK_PERIOD_MS));

				// ignore timeouts by plopping back into the IDLE state
				if(err == Socket::ErrTimeout) {
					this->changeState(this->state);
					break;
				}
				// ignore read errors
				else if(err != Socket::ErrSuccess) {
					LOG(S_ERROR, "Error reading socket: %d", err);

					// retry read
					this->changeState(this->state);
					break;
				}

				// otherwise, discard the received packet
				this->sock->discardRx(buffer);
				break;
			}
			// timeout waiting for response
			case TIMEOUT: {
				LOG(S_ERROR, "Timeout waiting for DHCP response");

				this->changeState(IDLE);
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
	sock->close();

	return 0;
}

/**
 * Sends a DHCPDISCOVER message, then advances the state machine to wait for
 * a DHCPOFFER message.
 */
void DHCPClient::taskSendDiscover(void) {
	int err;
	void *buffer;

	// get a random number as the transaction id
	this->currentXID = 0xDEADBEEF; // TODO: randomize this

	// get a TX buffer
	const size_t bytes = sizeof(dhcp_packet_ipv4_t) + sizeof(discoverOptions);

	err = this->sock->prepareTx(&buffer, bytes);

	if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't get TX buffer: %d", err);

		this->changeState(IDLE);
		return;
	}

	memset(buffer, 0, bytes);


	// fill in the DHCP packet
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) buffer;

	dhcp->op = kDHCPOpCodeRequest;
	dhcp->htype = kDHCPHardwareTypeEthernet;
	dhcp->hlen = 6;

	dhcp->flags |= kDHCPFlagBroadcast;

	dhcp->hardwareAddr.mac = this->stack->mac;

	dhcp->xid = this->currentXID;
	dhcp->cookie = DHCP_COOKIE;

	// copy options
	memcpy(dhcp->options, discoverOptions, sizeof(discoverOptions));


	// send TX buffer
	this->packetHostToNetwork(dhcp);

	err = this->sock->sendTo(buffer, kIPv4AddressBroadcast, DHCP_SERVER_PORT);

	if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't send TX buffer: %d", err);

		this->changeState(IDLE);
		return;
	}

	// wait for a DHCPOFFER
	this->changeState(WAITOFFER);
}

/**
 * Handles the OFFER state. This waits to read a DHCP offer from the network,
 * then processes it and advances the state machine as needed.
 */
void DHCPClient::taskHandleOffer(void) {
	int err;

	void *buffer;
	size_t bytesRead;

	// read from the socket
	err = this->sock->receive(&buffer, &bytesRead, DHCPClient::receiveTimeout);

	// was the error a timeout?
	if(err == Socket::ErrTimeout) {
		this->changeState(TIMEOUT);
		return;
	}
	// handle other read errors by retrying the read
	else if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Error reading socket: %d", err);

		// retry read
		this->changeState(this->state);
		return;
	}

	// byteswap the received packet
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) buffer;
	this->packetNetworkToHost(dhcp);


	// Read again if the XID is not matching
	if(dhcp->xid != this->currentXID) {
		LOG(S_INFO, "Received DHCP packet for another client");

		this->changeState(WAITOFFER);
		goto cleanup;
	}

	// make sure that the opcode and MAC address match
	if(dhcp->op != kDHCPOpCodeReply) {
		LOG(S_ERROR, "invalid opcode %d", dhcp->op);

		this->changeState(IDLE);
		goto cleanup;
	}
	if(dhcp->hardwareAddr.mac != this->stack->mac) {
		LOG(S_ERROR, "invalid MAC address");

		this->changeState(IDLE);
		goto cleanup;
	}


	// extract the offer information
	err = this->parseOptions(dhcp);

	if(err != kDHCPMessageTypeOffer) {
		LOG(S_ERROR, "Invalid type: %d", err);

		this->changeState(IDLE);
		goto cleanup;
	}
	// the message was an offer, so copy some relevant fields
	else {
		this->offer.address = dhcp->yourAddr;
	}

#if LOG_OFFER
	this->printOfferInfo();
#endif


	// send a request for the IP address we've been offered
	this->changeState(REQUEST);

	// perform cleanup (return RX buffer)
cleanup: ;
	this->sock->discardRx(buffer);
}

/**
 * Sends a DHCPREQUEST message for the IP address the server has previously
 * offered, then waits for the final DHCPACK message.
 */
void DHCPClient::taskSendRequest(void) {
	int err;
	void *buffer;

	// get a TX buffer
	size_t hostnameLength = strlen(this->stack->getHostname());

	if(hostnameLength > DHCP_MAX_HOSTNAME) {
		hostnameLength = DHCP_MAX_HOSTNAME;
	}

	size_t bytes = sizeof(dhcp_packet_ipv4_t) + sizeof(requestOptions);
	bytes += hostnameLength;

	err = this->sock->prepareTx(&buffer, bytes);

	if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't get TX buffer: %d", err);

		this->changeState(IDLE);
		return;
	}

	memset(buffer, 0, bytes);


	// fill in the DHCP packet
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) buffer;

	dhcp->op = kDHCPOpCodeRequest;
	dhcp->htype = kDHCPHardwareTypeEthernet;
	dhcp->hlen = 6;

	dhcp->flags |= kDHCPFlagBroadcast;

	dhcp->hardwareAddr.mac = this->stack->mac;

	dhcp->xid = this->currentXID;
	dhcp->cookie = DHCP_COOKIE;

	dhcp->replyAddr = this->offer.serverAddress;

	// copy options
	memcpy(dhcp->options, requestOptions, sizeof(requestOptions));


	// copy the server and offer IP addresses into the options
	this->fillRequestOptions(dhcp, hostnameLength);


	// send TX buffer
	this->packetHostToNetwork(dhcp);

	err = this->sock->sendTo(buffer, kIPv4AddressBroadcast, DHCP_SERVER_PORT);

	if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't send TX buffer: %d", err);

		this->changeState(IDLE);
		return;
	}


	// wait for a DHCPACK
	this->changeState(WAITACK);
}

/**
 * Handles the ACK state. This waits to read a DHCP ack from the network,
 * then processes it and advances the state machine as needed.
 */
void DHCPClient::taskHandleAck(void) {
	int err;

	void *buffer;
	size_t bytesRead;

	// read from the socket
	err = this->sock->receive(&buffer, &bytesRead, DHCPClient::receiveTimeout);

	// was the error a timeout?
	if(err == Socket::ErrTimeout) {
		this->changeState(TIMEOUT);
		return;
	}
	// handle other read errors by retrying the read
	else if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Error reading socket: %d", err);

		// retry read
		this->changeState(this->state);
		return;
	}

	// byteswap the received packet
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) buffer;
	this->packetNetworkToHost(dhcp);


	// Read again if the XID is not matching
	if(dhcp->xid != this->currentXID) {
		LOG(S_INFO, "Received DHCP packet for another client");

		this->changeState(WAITOFFER);
		goto cleanup;
	}


	// make sure that the opcode and MAC address match
	if(dhcp->op != kDHCPOpCodeReply) {
		LOG(S_ERROR, "invalid opcode %d", dhcp->op);

		this->changeState(IDLE);
		goto cleanup;
	}
	if(dhcp->hardwareAddr.mac != this->stack->mac) {
		LOG(S_ERROR, "invalid MAC address");

		this->changeState(IDLE);
		goto cleanup;
	}


	// parse the options
	err = this->parseOptions(dhcp);

	if(err != kDHCPMessageTypeAck) {
		LOG(S_ERROR, "Invalid type: %d", err);

		this->changeState(IDLE);
		goto cleanup;
	}
	// the message was an ack, so copy some relevant fields
	else {
		this->offer.address = dhcp->yourAddr;
	}

#if LOG_OFFER
	this->printOfferInfo();
#endif


	// change the IP config next
	this->changeState(SUCCESS);

	// perform cleanup (return RX buffer)
cleanup: ;
	this->sock->discardRx(buffer);
}

/**
 * Updates the IP configuration of the stack after the final DHCP ack was
 * received.
 */
void DHCPClient::taskUpdateIPConfig(void) {
	// copy the IP address, netmask and router
	this->stack->ip = this->offer.address;
	this->stack->netMask = this->offer.netmask;
	this->stack->routerIp = this->offer.router;

	// mark it as valid
	this->stack->ipConfigBecameValid();

	// set up renewal timer
	this->setUpRenewalTimer();

	// go back to the idle state
	this->changeState(IDLE);
}

/**
 * Attempts to renew the lease by sending a DHCPREQUEST to the server from
 * which we received the lease originally.
 */
void DHCPClient::taskRenewLease(void) {
	int err;
	void *buffer;

	// get a random number as the transaction id
	this->currentXID = 0xCAFEBABE; // TODO: randomize this

	// get a TX buffer
	size_t hostnameLength = strlen(this->stack->getHostname());

	if(hostnameLength > DHCP_MAX_HOSTNAME) {
		hostnameLength = DHCP_MAX_HOSTNAME;
	}

	size_t bytes = sizeof(dhcp_packet_ipv4_t) + sizeof(renewOptions);
	bytes += hostnameLength;


	err = this->sock->prepareTx(&buffer, bytes);

	if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't get TX buffer: %d", err);

		this->changeState(IDLE);
		return;
	}

	memset(buffer, 0, bytes);


	// fill in the DHCP packet
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) buffer;

	dhcp->op = kDHCPOpCodeRequest;
	dhcp->htype = kDHCPHardwareTypeEthernet;
	dhcp->hlen = 6;

	dhcp->hardwareAddr.mac = this->stack->mac;

	dhcp->xid = this->currentXID;
	dhcp->cookie = DHCP_COOKIE;

	// write the server's address in the packet
	dhcp->replyAddr = this->offer.serverAddress;

	// we have a valid IP address now so copy it
	dhcp->clientAddr = this->stack->ip;

	// copy options
	memcpy(dhcp->options, renewOptions, sizeof(renewOptions));


	// copy the server and offer IP addresses into the options
	this->fillRequestOptions(dhcp, hostnameLength);

	// unicast TX buffer to the DHCP server
	this->packetHostToNetwork(dhcp);

	err = this->sock->sendTo(buffer, this->offer.serverAddress, DHCP_SERVER_PORT);

	// if an error occurred, restart the DHCP process
	if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Couldn't send TX buffer: %d; restarting DHCP process", err);

		this->changeState(DISCOVER);
		return;
	}


	// wait for a DHCPACK
	this->changeState(WAITRENEWACK);
}

/**
 * Waits for an acknowledgement from the DHCP server. If we receive a DHCPACK,
 * the lease was renewed and we update the parameters of the lease. If a
 * DHCPNACK is received instead, get a new lease.
 */
void DHCPClient::taskRenewWaitAck(void) {
	int err;

	void *buffer;
	size_t bytesRead;

	// read from the socket
	err = this->sock->receive(&buffer, &bytesRead, DHCPClient::receiveTimeout);

	// was the error a timeout?
	if(err == Socket::ErrTimeout) {
		this->changeState(TIMEOUT);
		return;
	}
	// handle other read errors by retrying the read
	else if(err != Socket::ErrSuccess) {
		LOG(S_ERROR, "Error reading socket: %d", err);

		// retry read
		this->changeState(this->state);
		return;
	}

	// byteswap the received packet
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) buffer;
	this->packetNetworkToHost(dhcp);


	// Read again if the XID is not matching
	if(dhcp->xid != this->currentXID) {
		LOG(S_INFO, "Received DHCP packet for another client");

		this->changeState(WAITOFFER);
		goto cleanup;
	}


	// make sure that the opcode and MAC address match
	if(dhcp->op != kDHCPOpCodeReply) {
		LOG(S_ERROR, "invalid opcode %d", dhcp->op);

		this->changeState(IDLE);
		goto cleanup;
	}
	if(dhcp->hardwareAddr.mac != this->stack->mac) {
		LOG(S_ERROR, "invalid MAC address");

		this->changeState(IDLE);
		goto cleanup;
	}


	// parse the options
	err = this->parseOptions(dhcp);

	// did we receive a negative ack?
	if(err == kDHCPMessageTypeNack) {
		LOG(S_INFO, "Received NACK from DHCP server, getting new lease!");

		this->changeState(DISCOVER);
		goto cleanup;
	}
	// did we receive a message other than a positive ack?
	else if(err != kDHCPMessageTypeAck) {
		LOG(S_ERROR, "Invalid response: %d; getting new lease", err);

		this->changeState(DISCOVER);
		goto cleanup;
	}
	// the message was an ack, so copy some relevant fields
	else {
		this->offer.address = dhcp->yourAddr;
	}

#if LOG_OFFER
	this->printOfferInfo();
#endif


	// change the IP config next
	this->changeState(SUCCESS);

	// perform cleanup (return RX buffer)
cleanup: ;
	this->sock->discardRx(buffer);
}



/**
 * Fills the options in a request packet with the appropriate data.
 *
 * @note If the hostname should be filled, it must be the last element in the
 * options list.
 */
void DHCPClient::fillRequestOptions(void *_dhcp, size_t hostnameLength) {
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) _dhcp;

	// fill the server address, requested address, and hostname
	bool end = false;
	uint8_t *opt = dhcp->options;

	while(!end) {
		uint8_t option = *opt++;

		switch(option) {
			// server IP address?
			case kDHCPOptionServerAddress: {
				uint8_t length = *opt++;

				// copy server address
				memcpy(opt, &this->offer.serverAddress, 4);

				// increment length
				opt += length;
				break;
			}
			// requested IP address?
			case kDHCPOptionRequestedAddress: {
				uint8_t length = *opt++;

				// copy the offered address
				memcpy(opt, &this->offer.address, 4);

				// increment length
				opt += length;
				break;
			}
			// hostname?
			case kDHCPOptionsHostname: {
				// write the length
				*opt++ = (uint8_t) hostnameLength;

				// copy the hostname
				memcpy(opt, this->stack->getHostname(), hostnameLength);

				// terminate the options list
				opt += hostnameLength;

				*opt++ = kDHCPOptionEnd;
				break;
			}

			// end of list?
			case kDHCPOptionEnd:
				end = true;
				break;
		}
	}
}

/**
 * Sets up the lease renewal timer. This timer fires after half the lease
 * time has expired.
 */
void DHCPClient::setUpRenewalTimer(void) {
	BaseType_t ok;

	// get the period, in ticks
	TickType_t period = (this->offer.leaseExpiration * 1000) / 2;
	period /= portTICK_PERIOD_MS;

	// XXX: debug
//	period = 30000 / portTICK_PERIOD_MS;

	// cancel the old timer if it's set
	if(this->renewalTimer) {
		xTimerDelete(this->renewalTimer, portMAX_DELAY);
		this->renewalTimer = nullptr;
	}

	// create timer
#if LOG_RENEWAL_TIMER
	LOG(S_DEBUG, "DHCP timer fires in %u ticks", period);
#endif

	this->renewalTimer = xTimerCreate("DHCPRenew", period, pdFALSE, this,
			_DHCPRenewTimeout);

	if(this->renewalTimer == nullptr) {
		LOG(S_FATAL, "Couldn't create DHCP renewal timer");
	}

	// start timer
	ok = xTimerStart(this->renewalTimer, portMAX_DELAY);

	if(ok != pdPASS) {
		LOG(S_FATAL, "Couldn't start DHCP renewal timer");
	}
}



/**
 * Parses the options field of a DHCP packet, and return the value of the
 * "message type" option (53) if present.
 */
int DHCPClient::parseOptions(void *_dhcp) {
	bool end = false;
	int type = -1;

	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) _dhcp;

	// go through the options, reading them byte by byte
	uint8_t *opt = dhcp->options;

	while(!end) {
		// read the first byte (option code)
		uint8_t option = *opt++;

		// handle it
		switch(option) {
			// is it the message type?
			case kDHCPOptionMessageType:
				opt++; // ignore length byte
				type = *opt++;
				break;

			// is it the lease expiration time?
			case kDHCPOptionLeaseTime: {
				uint8_t length = *opt++;

				// ensure length is 4
				if(length != 4) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy one 32-bit value
				uint32_t expiry = *((uint32_t *) opt);
				expiry = __builtin_bswap32(expiry);

				this->offer.leaseExpiration = expiry;

				// increment the read pointer
				opt += length;
				break;
			}

			// is it the server's IP address?
			case kDHCPOptionServerAddress: {
				uint8_t length = *opt++;

				// ensure length is 4
				if(length != 4) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy four bytes as an IP address
				this->offer.serverAddress = *((stack_ipv4_addr_t *) opt);

				// increment the read pointer
				opt += length;
				break;
			}

			// is it the subnet mask?
			case kDHCPOptionSubnetMask: {
				uint8_t length = *opt++;

				// ensure length is 4
				if(length != 4) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy four bytes as an IP address
				this->offer.netmask = *((stack_ipv4_addr_t *) opt);

				// increment the read pointer
				opt += length;
				break;
			}

			// is it the router address?
			case kDHCPOptionRouter: {
				uint8_t length = *opt++;

				// ensure length is at least 4, and a multiple thereof
				if(length < 4 && (length % 4) != 0) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy the first four bytes as an IP address
				this->offer.router = *((stack_ipv4_addr_t *) opt);

				// ignore the remaining addresses
				opt += length;
				break;
			}

			// is it the DNS server address?
			case kDHCPOptionDNS: {
				uint8_t length = *opt++;

				// ensure length is at least 4, and a multiple thereof
				if(length < 4 && (length % 4) != 0) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy the first four bytes as an IP address
				this->offer.dnsServer = *((stack_ipv4_addr_t *) opt);

				// ignore the remaining addresses
				opt += length;
				break;
			}

			// is it the log server address?
			case kDHCPOptionsSyslogServer: {
				uint8_t length = *opt++;

				// ensure length is at least 4, and a multiple thereof
				if(length < 4 && (length % 4) != 0) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy the first four bytes as an IP address
				this->offer.syslogServer = *((stack_ipv4_addr_t *) opt);

				// ignore the remaining addresses
				opt += length;
				break;
			}

			// is it the TFTP server address?
			// TODO: This could be a hostname rather than an IP address
			case kDHCPOptionsTFTPServer: {
				uint8_t length = *opt++;

				// ensure length is at least 4, and a multiple thereof
				if(length < 4 && (length % 4) != 0) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy the first four bytes as an IP address
				this->offer.tftpServer = *((stack_ipv4_addr_t *) opt);

				// ignore the remaining addresses
				opt += length;
				break;
			}

			// is it the NTP server address?
			case kDHCPOptionsNTPServer: {
				uint8_t length = *opt++;

				// ensure length is at least 4, and a multiple thereof
				if(length < 4 && (length % 4) != 0) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy the first four bytes as an IP address
				this->offer.ntpServer = *((stack_ipv4_addr_t *) opt);

				// ignore the remaining addresses
				opt += length;
				break;
			}
			// is it the UTC offset option?
			case kDHCPOptionsUTCOffset: {
				uint8_t length = *opt++;

				// ensure length is 4
				if(length != 4) {
					LOG(S_WARN, "Invalid length %u for option %u", length, option);

					opt += length;
					break;
				}

				// copy one 32-bit value
				uint32_t expiry = *((uint32_t *) opt);
				expiry = __builtin_bswap32(expiry);

				this->offer.utcOffset = expiry;

				// increment the read pointer
				opt += length;
				break;
			}


			// is it padding?
			case kDHCPOptionPadding:
				break;
			// is it the end of the list?
			case kDHCPOptionEnd:
				end = true;
				break;
			// if we don't know the type of option, skip length bytes
			default:
				LOG(S_DEBUG, "Ignoring unknown option %u", option);

				uint8_t length = *opt++;
				opt += length;
				break;
		}
	}

	// we're done
	return type;
}



/**
 * Swaps all multibyte fields in the DHCP header. This does not include any
 * options.
 */
void DHCPClient::convertPacketByteOrder(void *_dhcp) {
	dhcp_packet_ipv4_t *dhcp = (dhcp_packet_ipv4_t *) _dhcp;

	dhcp->xid = __builtin_bswap32(dhcp->xid);
	dhcp->secs = __builtin_bswap16(dhcp->secs);
	dhcp->flags = __builtin_bswap16(dhcp->flags);

	dhcp->cookie = __builtin_bswap32(dhcp->cookie);
}



/**
 * Prints offer information.
 */
void DHCPClient::printOfferInfo(void) {
	char ipStr1[16], ipStr2[16];

	// display the offered address and netmask
	Stack::ipToString(this->offer.address, ipStr1, 16);
	Stack::ipToString(this->offer.netmask, ipStr2, 16);

	LOG(S_INFO, "DHCP server offered %s (%s); expires in %u seconds", ipStr1, ipStr2, this->offer.leaseExpiration);

	// display router and DNS server address
	Stack::ipToString(this->offer.router, ipStr1, 16);
	Stack::ipToString(this->offer.dnsServer, ipStr2, 16);

	LOG(S_INFO, "DHCP router at %s, DNS server %s", ipStr1, ipStr2);

	// display router and DNS server address
	Stack::ipToString(this->offer.syslogServer, ipStr1, 16);
	Stack::ipToString(this->offer.tftpServer, ipStr2, 16);

	LOG(S_INFO, "DHCP syslog server at %s, TFTP server %s", ipStr1, ipStr2);

	// display NTP server address
	Stack::ipToString(this->offer.ntpServer, ipStr1, 16);

	LOG(S_INFO, "DHCP NTP server at %s, UTC offset %d", ipStr1, this->offer.utcOffset);

	// display server info
	Stack::ipToString(this->offer.serverAddress, ipStr1, 16);

	LOG(S_INFO, "DHCP server is %s", ipStr1);
}

} /* namespace ip */
