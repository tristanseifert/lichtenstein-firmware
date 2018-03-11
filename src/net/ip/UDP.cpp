/*
 * UDP.cpp
 *
 *  Created on: Feb 26, 2018
 *      Author: tristan
 */
#define LOG_MODULE "UDP"

#define UDP_PRIVATE 1
#include "UDPPrivate.h"
#include "UDP.h"

#include "UDPSocket.h"

#include "IPv4.h"
#include "IPv4Private.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>

#include <cstddef>
#include <cstring>

// set to 1 to log when packets are handled
#define LOG_HANDLING						1
// log transmitted packets
#define LOG_TX_PACKETS					0




namespace ip {

/**
 * Sets up the UDP processing task.
 */
UDP::UDP(Stack *_s, IPv4 *_ipv4) : stack(_s), ipv4(_ipv4) {
	// allocate listening ports list and clear it
	const size_t listenPortsSz = sizeof(udp_listen_t) * UDP::listenPortsEntries;
	this->listenPorts = (udp_listen_t *) pvPortMalloc(listenPortsSz);

	if(this->listenPorts == nullptr) {
		LOG(S_FATAL, "Couldn't allocate listen ports struct");
	}

	memset(this->listenPorts, 0, listenPortsSz);

}

/**
 * Kills the UDP processing task and any associated resources.
 */
UDP::~UDP() {
	// release memory from the listening ports
	vPortFree(this->listenPorts);
}



/**
 * Handles a received frame. This looks up whether any socket is listening on
 * the port, and if the packet was received with multicast/broadcast, checks
 * if the socket is set up to receive those.
 */
void UDP::handleReceivedFrame(void *_rx, int type) {
	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *) _rx;

	// byteswap the UDP header
	udp_header_ipv4_t *header = (udp_header_ipv4_t *) rx->payload;
	this->packetNetworkToHost(header);

	// check if any tasks are listening on this port
	for(size_t i = 0; i < UDP::listenPortsEntries; i++) {
		udp_listen_t *listen = &this->listenPorts[i];

#if 0
		LOG(S_DEBUG, "%u: %s port %u to sock 0x%08x", i, listen->valid ? "valid" : "invalid", listen->port, listen->sock);
#endif

		// is the port equal to the port we received the packet on?
		if(listen->valid && listen->port == header->destPort) {
			switch(type) {
				// directly forward unicast
				case UNICAST: {
					// send it to the socket
					listen->sock->receivedFrame(rx, UDPSocket::UNICAST);

					// increment counter
					this->handledRxUnicast++;

					goto handled;
				}

				// forward multicast if requested
				case MULTICAST: {
					// if multicast is accepted, send it
					if(listen->acceptsMulticast) {
						// send it to the socket
						listen->sock->receivedFrame(rx, UDPSocket::MULTICAST);

						// increment counter
						this->handledRxMulticast++;

						goto handled;
					}

					// otherwise, we get down here
					break;
				}

				// forward broadcast if requested
				case BROADCAST: {
					// check that the socket accepts broadcast
					if(listen->acceptsBroadcast) {
						// send it to the socket
						listen->sock->receivedFrame(rx, UDPSocket::BROADCAST);

						// increment counter
						this->handledRxBroadcast++;

						goto handled;
					}

					// otherwise, we get down here
					break;
				}

				// default case: should NEVER get here
				default: {
					LOG(S_ERROR, "Invalid type %u for 0x%x", type, listen);

					goto discard;
				}
			}
		}
	}

discard: ;
	// if we get down here, discard the packet since nobody is handling it
	this->unhandledRxPackets++;

#if LOG_HANDLING
	char ipStrSrc[16], ipStrDest[16];

	Stack::ipToString(rx->ipv4Header->source, ipStrSrc, 16);
	Stack::ipToString(rx->ipv4Header->dest, ipStrDest, 16);

	LOG(S_DEBUG, "Unhandled packet: %s:%u -> %s:%u (%u)", ipStrSrc,
			header->sourcePort, ipStrDest, header->destPort,
			this->unhandledRxPackets);
#endif

	this->ipv4->releaseRxBuffer(rx);

	// clean-up
handled: ;
}



/**
 * Allocates a new UDP socket, without any bindings set up.
 */
UDPSocket *UDP::createSocket(void) {
	return new UDPSocket(this);
}



/**
 * Registers the given socket for listening on a certain port.
 */
int UDP::bindSocketForPort(UDPSocket *sock, uint16_t port) {
	size_t i;
	int err = UDP::ErrSuccess;

	// check the port isn't already used
	for(i = 0; i < UDP::listenPortsEntries; i++) {
		udp_listen_t *listen = &this->listenPorts[i];

		// if it's valid and port is equal, the port is in use
		if(listen->valid && listen->port == port) {
			LOG(S_ERROR, "Port %u is already in use", port);

			err = UDP::ErrPortInUse; goto done;
		}
	}

	// find an empty entry and write this socket into it
	for(i = 0; i < UDP::listenPortsEntries; i++) {
		udp_listen_t *listen = &this->listenPorts[i];

		// if it's invalid, fill it in
		if(!listen->valid) {
			// zero this entry
			memset(listen, 0, sizeof(udp_listen_t));

			// fill in the various fields
			listen->valid = true;

			listen->port = port;
			listen->sock = sock;

			err = UDP::ErrSuccess; goto done;
		}
	}

	// if we get down here, we couldn't find any space
	LOG(S_ERROR, "No space to accept more listening sockets");
	err = UDP::ErrNoBookkeepingSpace; goto done;


	// clean up
done: ;
	return err;
}

/**
 * Removes any bindings for the given socket.
 */
void UDP::unbindSocket(UDPSocket *sock) {
	unsigned int removed = 0;

	// iterate over each entry
	for(size_t i = 0; i < UDP::listenPortsEntries; i++) {
		udp_listen_t *listen = &this->listenPorts[i];

		// if it's valid and the socket is equal, remove it
		if(listen->valid && listen->sock == sock) {
			// invalidate the entry
			listen->valid = false;

			removed++;
		}
	}

	// logging
	LOG(S_DEBUG, "Removed %u mappings for socket 0x%x", removed, sock);
}



/**
 * Finds the given socket in the listening array, and sets its multicast
 * reception flag.
 */
int UDP::setMulticastReceptionState(UDPSocket *sock, bool enabled) {
	// search through all listening entries
	for(size_t i = 0; i < UDP::listenPortsEntries; i++) {
		udp_listen_t *listen = &this->listenPorts[i];

		// is this entry valid and is the socket pointer equal?
		if(listen->valid && listen->sock == sock) {
			// update the multicast flag
			listen->acceptsMulticast = (enabled == true) ? 1 : 0;
			return 0;
		}
	}

	// if we get down here, the socket isn't listening?
	return -1;
}

/**
 * Finds the given socket in the listening array, and sets its broadcast
 * reception flag.
 */
int UDP::setBroadcastReceptionState(UDPSocket *sock, bool enabled) {
	// search through all listening entries
	for(size_t i = 0; i < UDP::listenPortsEntries; i++) {
		udp_listen_t *listen = &this->listenPorts[i];

		// is this entry valid and is the socket pointer equal?
		if(listen->valid && listen->sock == sock) {
			// update the broadcast flag
			listen->acceptsBroadcast = (enabled == true) ? 1 : 0;
			return 0;
		}
	}

	// if we get down here, the socket isn't listening?
	return -1;
}



/**
 * Allocates a transmit buffer.
 */
udp_tx_packet_t *UDP::getTxBuffer(UDPSocket *sock, size_t payloadLength, int timeout) {
	// attempt to get a tx buffer
	size_t responseSize = sizeof(udp_header_ipv4_t);
	responseSize += payloadLength;

	void *_tx = this->ipv4->getIPv4TxBuffer(responseSize, kIPv4ProtocolUDP, timeout);
	stack_ipv4_tx_packet_t *tx = (stack_ipv4_tx_packet_t *) _tx;

	if(tx == nullptr) {
		// exit if we can't get a buffer
		LOG(S_ERROR, "Couldn't get transmit buffer");
		return nullptr;
	}

	// allocate the tx packet
	udp_tx_packet_t *packet;
	packet = (udp_tx_packet_t *) pvPortMalloc(sizeof(udp_tx_packet_t));

	if(packet == nullptr) {
		// exit if we can't allocate the tx packet
		this->ipv4->discardTxBuffer(tx);

		LOG(S_ERROR, "Couldn't allocate tx packet");
		return nullptr;
	}

	packet->ipTx = tx;

	// populate the UDP header
	udp_header_ipv4_t *udpHeader = (udp_header_ipv4_t *) tx->payload;

	udpHeader->checksum = 0x0000; // inserted by MAC
	udpHeader->length = ((uint16_t) (sizeof(udp_header_ipv4_t) + payloadLength));

	// set the location of the payload and finish
	packet->payload = ((uint8_t *) udpHeader) + sizeof(udp_header_ipv4_t);

	return packet;
}

/**
 * Discards a transmit buffer without sending it.
 */
int UDP::discardTxBuffer(__attribute__((unused)) UDPSocket *sock,
		udp_tx_packet_t *buffer) {
	// discard IP TX buffer and deallocate the UDP tx packet
	this->ipv4->discardTxBuffer(buffer->ipTx);
	vPortFree(buffer);

	return 0;
}

/**
 * Queues a previously created transmit buffer for transmission.
 */
int UDP::sendTxBuffer(UDPSocket *sock, stack_ipv4_addr_t address, unsigned int port, udp_tx_packet_t *buffer) {
	// set the destination address in the IP packet
	this->ipv4->setIPv4Destination(buffer->ipTx, address);

	// set the source/dest port in the UDP packet
	udp_header_ipv4_t *udpHeader = (udp_header_ipv4_t *) buffer->ipTx->payload;

	udpHeader->destPort = (uint16_t) port;
	udpHeader->sourcePort = (uint16_t) sock->localPort;

	this->packetHostToNetwork(udpHeader);

	// logging
#if LOG_TX_PACKETS
	char ipStr[16];
	Stack::ipToString(address, ipStr, 16);
	LOG(S_DEBUG, "Sending UDP packet to %s:%u", ipStr, port);
#endif

	// transmit the buffer
	if(this->ipv4->transmitIPv4TxBuffer(buffer->ipTx) == false) {
		LOG(S_ERROR, "Couldn't send buffer");
		return UDP::ErrTxFailure;
	}

	// success; free the tx structure
	vPortFree(buffer);

	return UDP::ErrSuccess;
}



/**
 * Swaps the order of multi-byte fields in the UDP packet.
 */
void UDP::convertPacketByteOrder(void *_udp) {
	udp_header_ipv4_t *udp = (udp_header_ipv4_t *) _udp;

	udp->sourcePort = __builtin_bswap16(udp->sourcePort);
	udp->destPort = __builtin_bswap16(udp->destPort);

	udp->length = __builtin_bswap16(udp->length);

	udp->checksum = __builtin_bswap16(udp->checksum);
}

} /* namespace ip */
