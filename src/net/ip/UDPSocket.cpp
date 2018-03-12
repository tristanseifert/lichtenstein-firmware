/*
 * UDPSocket.cpp
 *
 *  Created on: Mar 2, 2018
 *      Author: tristan
 */
#define LOG_MODULE "UDP"

#include "UDPSocket.h"

#define UDP_PRIVATE 1
#include "UDPPrivate.h"
#include "UDP.h"

#include "IPv4.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>



// log RX/TX map
#define LOG_BUFFER_MAPS						0
// log received frames
#define LOG_RECEPTION						0



namespace ip {

/**
 * Initializes an UDP socket.
 */
UDPSocket::UDPSocket(UDP *_udp) : udp(_udp) {
	// set up the queue
	this->rxMessageQueue = xQueueCreate(UDPSocket::rxMessageQueueSize,
			sizeof(udp_sock_msg_t));

	if(!this->rxMessageQueue) {
		LOG(S_ERROR, "Couldn't create RX message queue!");
	}

	// clear the rx/tx b uffer maps
	memset(this->rxBufferMap, 0, sizeof(this->rxBufferMap));
	memset(this->txBufferMap, 0, sizeof(this->txBufferMap));

	// get protocol pointers
	this->ipv4 = udp->ipv4;
}

/**
 * Removes any pending descriptors from the socket.
 */
UDPSocket::~UDPSocket() {
	// deallocate queue
	if(this->rxMessageQueue) {
		vQueueDelete(this->rxMessageQueue);
	}

	// the socket should've been closed now
	if(this->isOpened) {
		LOG(S_ERROR, "Deallocating socket 0x%x, but it was never closed!", this);
	}
}



/**
 * Called by the UDP stack when a frame was received. This posts the message
 * to unblock the client app.
 */
bool UDPSocket::receivedFrame(void *_rx, int type) {
	BaseType_t ok;
	udp_sock_msg_t msg;

	// get the buffer
	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *) _rx;

	msg.type = kMessageTypeReceivedPacket;
	msg.buffer.rx = rx;

#if LOG_RECEPTION
	udp_header_ipv4_t *header = (udp_header_ipv4_t *) rx->payload;
	LOG(S_DEBUG, "Received packet for socket 0x%x at port %u", this, header->destPort);
#endif

	// userdata is the type of reception
	msg.userData = type;

	// post message without timeout. message is dropped if queue is full
	ok = xQueueSendToBack(this->rxMessageQueue, &msg, 0);

	// if the queue is full, discard the buffer
	if(ok == errQUEUE_FULL) {
		this->udp->ipv4->releaseRxBuffer(rx);

		this->droppedRxPackets++;
		LOG(S_ERROR, "Overflow on socket 0x%x, discarding packet", this);

		return false;
	} else if(ok != pdTRUE) {
		this->udp->ipv4->releaseRxBuffer(rx);

		LOG(S_ERROR, "Unknown error posting receive message: %u", ok);

		return false;
	}

	// if we get down here, the message was sent
	return true;
}

/**
 * Blocks the specified amount of time for a packet to arrive on the socket.
 */
int UDPSocket::receive(void **buffer, size_t *bytesRead, unsigned int timeout) {
	BaseType_t ok;
	udp_sock_msg_t msg;

	// handle negative timeout values
	if(((int) timeout) < 0) {
		timeout = portMAX_DELAY;
	}

	// read the queue
	ok = xQueueReceive(this->rxMessageQueue, &msg, timeout);

	if(ok != pdTRUE) {
		// if timeout is infinite, there was a queue error
		if(timeout == portMAX_DELAY) {
			LOG(S_INFO, "Couldn't get receive message");
			return Socket::ErrReceiveIO;
		}
		// if the timeout was finite, we timed out
		else {
			return Socket::ErrTimeout;
		}
	}

	// return the address of the actual data in the packet
	*buffer = ((uint8_t *) msg.buffer.rx->payload) + sizeof(udp_header_ipv4_t);

	udp_header_ipv4_t *header = (udp_header_ipv4_t *) msg.buffer.rx->payload;
	*bytesRead = header->length - sizeof(udp_header_ipv4_t);

	// write it into the buffer map
	for(size_t i = 0; i < UDPSocket::rxMapSize; i++) {
		// is the buffer null?
		if(this->rxBufferMap[i].buffer == nullptr) {
			this->rxBufferMap[i].buffer = *buffer;
			this->rxBufferMap[i].rxBuffer = msg.buffer.rx;

#if LOG_BUFFER_MAPS
			LOG(S_DEBUG, "RX Buffer 0x%x -> rx 0x%x, index %u", *buffer, msg.buffer.rx, i);
#endif

			return Socket::ErrSuccess;
		}
	}

	// if we get down here, there was no space available
	LOG(S_ERROR, "Couldn't find rx buffer map space");
	this->udp->ipv4->releaseRxBuffer(msg.buffer.rx);

	return Socket::ErrNoBookkeepingSpace;
}

/**
 * Calls into the UDP stack to return the receive buffer to the stack.
 */
int UDPSocket::discardRx(void *buffer) {
	// find the address in the buffer list
	for(size_t i = 0; i < UDPSocket::rxMapSize; i++) {
		// is the buffer address equal?
		if(this->rxBufferMap[i].buffer == buffer) {
			// if so, release it and clear the struct
			this->udp->ipv4->releaseRxBuffer(this->rxBufferMap[i].rxBuffer);

#if LOG_BUFFER_MAPS
			LOG(S_DEBUG, "Clear RX buffer %u", i);
#endif

			this->rxBufferMap[i].buffer = nullptr;
			this->rxBufferMap[i].rxBuffer = nullptr;

			return Socket::ErrSuccess;
		}
	}

	// if we get down here, we couldn't find it
	LOG(S_ERROR, "Couldn't release RX buffer for address 0x%x", buffer);

	return Socket::ErrUnknownBuffer;
}



/**
 * Attempts to get a transmit buffer.
 */
int UDPSocket::prepareTx(void **buffer, size_t length, unsigned int timeout) {
	// handle negative timeout values
	if(((int) timeout) < 0) {
		timeout = portMAX_DELAY;
	}

	// attempt to get a tx packet
	udp_tx_packet_t *packet = this->udp->getTxBuffer(this, length, timeout);

	if(packet == nullptr) {
		LOG(S_ERROR, "Couldn't get TX packet for socket 0x%x", this);
		return -1;
	}

	// write it into the tx map
	for(size_t i = 0; i < UDPSocket::txMapSize; i++) {
		// is the buffer null?
		if(this->txBufferMap[i].buffer == nullptr) {
			this->txBufferMap[i].buffer = packet->payload;
			this->txBufferMap[i].txPacket = packet;

#if LOG_BUFFER_MAPS
			LOG(S_DEBUG, "TX Buffer 0x%x -> txPacket 0x%x, index %u", packet->payload, packet, i);
#endif

			goto success;
		}
	}

	// if we get here, there wasn't sufficient space in the tx map
	this->udp->discardTxBuffer(this, packet);

	LOG(S_ERROR, "Insufficient space in tx buffer map");
	return -2;

	// if we get here, we were able to insert the packet
success: ;
	*buffer = packet->payload;

	return Socket::ErrSuccess;
}

/**
 * Queues a transmit buffer.
 */
int UDPSocket::queueTx(void *buffer, unsigned int timeout) {
	// make sure socket is connected
	if(!this->isConnected) {
		this->discardTx(buffer);

		LOG(S_ERROR, "Couldn't send packet: socket is not connected");
		return Socket::ErrNotConnected;
	}

	// transmit the packet
	return this->sendTo(buffer, this->remoteAddr,
			(uint16_t) this->remotePort, timeout);
}

/**
 * Sends a packet to the given address and port.
 */
int UDPSocket::sendTo(void *buffer, stack_ipv4_addr_t addr, uint16_t port, unsigned int timeout) {
	int err = 0;
	udp_tx_packet_t *tx;

	// handle negative timeout values
	if(timeout == (unsigned int) -1) {
		timeout = portMAX_DELAY;
	}

	// find the address in the buffer list
	for(size_t i = 0; i < UDPSocket::txMapSize; i++) {
		// is the buffer address equal?
		if(this->txBufferMap[i].buffer == buffer) {
			tx = (udp_tx_packet_t *) this->txBufferMap[i].txPacket;


			// set the TTL on the IP packet
			this->ipv4->setIPv4TTL(tx->ipTx, this->ipv4TTL);

			/*// write the port number into the UDP header and set destination
			this->udp->ipv4->setIPv4Destination(tx->ipTx, addr);

			udp_header_ipv4_t *udpHeader = (udp_header_ipv4_t *) tx->ipTx->payload;
			udpHeader->destPort = port;*/

			// then, transmit it
			err = this->udp->sendTxBuffer(this, addr, port, tx, !this->sendWithInvalidIP);

			// then empty the fields
			this->txBufferMap[i].buffer = nullptr;
			this->txBufferMap[i].txPacket = nullptr;

#if LOG_BUFFER_MAPS
			LOG(S_DEBUG, "Clear TX buffer %u", i);
#endif

			// return error state of the transmit call
			return err;
		}
	}

	// if we get down here, we couldn't find the tx buffer
	LOG(S_ERROR, "Couldn't find TX buffer for address 0x%x", buffer);
	return Socket::ErrUnknownBuffer;
}

/**
 * Discards a transmit buffer without sending it.
 */
int UDPSocket::discardTx(void *buffer) {
	udp_tx_packet_t *tx;

	// find the address in the buffer list
	for(size_t i = 0; i < UDPSocket::txMapSize; i++) {
		// is the buffer address equal?
		if(this->txBufferMap[i].buffer == buffer) {
			// discard buffer
			tx = (udp_tx_packet_t *) this->txBufferMap[i].txPacket;
			this->udp->discardTxBuffer(this, tx);

			// then empty the fields
			this->txBufferMap[i].buffer = nullptr;
			this->txBufferMap[i].txPacket = nullptr;

			return Socket::ErrSuccess;
		}
	}

	// if we get down here, we couldn't find the buffer
	LOG(S_ERROR, "Couldn't find buffer 0x%x for socket 0x%x", buffer, this);

	return Socket::ErrUnknownBuffer;
}



/**
 * Opens the socket for use.
 */
int UDPSocket::open(void) {
	if(this->isOpened) {
		LOG(S_ERROR, "Called open on already opened socket 0x%x", this);
	}

	this->isOpened = true;

	return Socket::ErrSuccess;
}

/**
 * Closes the socket.
 */
int UDPSocket::close(void)  {
	if(!this->isOpened) {
		LOG(S_ERROR, "Called close on unopened socket 0x%x", this);

		return Socket::ErrNotOpen;
	}

	this->isOpened = false;

	// remove any bindings
	this->udp->unbindSocket(this);

	return Socket::ErrSuccess;
}



/**
 * Sets the port and address to which sent packets are addressed.
 */
int UDPSocket::connect(stack_ipv4_addr_t address, unsigned int port) {
	// just set the values. we do nothing more
	this->remoteAddr = address;
	this->remotePort = port;

	// there is no connection to establish
	this->isConnected = true;

	return Socket::ErrSuccess;
}

/**
 * Accepts packets received on the given port. If the port is already in use,
 * indicate an error.
 */
int UDPSocket::bind(unsigned int port) {
	int err;

	// return if we're already bound
	if(this->localPort) {
		LOG(S_ERROR, "Socket 0x%x is already bound to port %u", this, this->localPort);
		return Socket::ErrPortInUse;
	}

	// try to register for this port
	err = this->udp->bindSocketForPort(this, (uint16_t) port);

	if(err != 0) {
		LOG(S_ERROR, "Couldn't bind port %u: %u", port, err);
		return err;
	}

	// set the local port and return success
	this->localPort = port;

	return Socket::ErrSuccess;
}



/**
 * Handles setting a socket option.
 */
int UDPSocket::setSockOpt(socket_protocol_t protocol, socket_option_t option, const void *value, size_t length) {
	// if joining a multicast group, enable multicast reception
	if(protocol == Socket::kSocketProtocolIPv4) {
		if(option == Socket::kSockOptJoinMulticast) {
			// enable multicast reception and increment refcount
			this->udp->setMulticastReceptionState(this, true);

			this->multicastRefCount++;
		} else if(option == Socket::kSockOptLeaveMulticast) {
			// decrement ref counter; if it's zero, disable multicast
			this->multicastRefCount--;

			if(this->multicastRefCount == 0) {
				// disable multicast reception
				this->udp->setMulticastReceptionState(this, false);
			}
		}

		// don't return so the IP protocol can handle it too
	}

	// handle UDP packets here
	if(protocol == Socket::kSocketProtocolUDP) {
		// run the appropriate option
		switch(option) {
			// accept broadcast packets
			case kSockOptAcceptBroadcast: {
				// size must be equal to bool size
				if(length != sizeof(bool)) {
					return Socket::ErrInvalidOptionLength;
				}

				bool *boolValue = (bool *) value;
				this->udp->setBroadcastReceptionState(this, *boolValue);

				return Socket::ErrSuccess;
			}

			// accept multicast packets
			case kSockOptAcceptMulticast: {
				// size must be equal to bool size
				if(length != sizeof(bool)) {
					return Socket::ErrInvalidOptionLength;
				}

				bool *boolValue = (bool *) value;
				this->udp->setMulticastReceptionState(this, *boolValue);

				return Socket::ErrSuccess;
			}

			// unsupported options
			default:
				return Socket::ErrInvalidOption;
		}

		// if we get down here, the option was unknown/unsupported
		return Socket::ErrInvalidOption;
	}

	// handle it within the socket class
	return Socket::setSockOpt(protocol, option, value, length);
}

/**
 * Retrieves the value of a socket option.
 */
int UDPSocket::getSockOpt(socket_protocol_t protocol, socket_option_t option, void *out, size_t length) {
	// TODO: implement
	return Socket::ErrUnimplemented;
}

} /* namespace ip */
