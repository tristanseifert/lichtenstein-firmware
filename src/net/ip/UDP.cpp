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

#include "IPv4.h"
#include "IPv4Private.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>

#include <cstddef>
#include <cstring>

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
 * Forwards a received unicast frame to any applications listening on the port
 * the packet was received on.
 */
void UDP::processUnicastFrame(void *_rx) {
	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *) _rx;
}

/**
 * Forwards a received multicast packet to any applications listening on the
 * port the packet was received on, and are also members of the multicast
 * group this message was received on.
 */
void UDP::processMulticastFrame(void *_rx) {
	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *) _rx;

}

/**
 * Forwards a received broadcast packet to any applications that are listening
 * on the port the packet was received on, and have enabled reception of
 * broadcast frames.
 */
void UDP::processBroadcastFrame(void *_rx) {
	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *) _rx;

}

/**
 * Handles a received frame. This looks up whether any socket is listening on
 * the port, and if the packet was received with multicast/broadcast, checks
 * if the socket is set up to receive those.
 */
void UDP::hadleReceivedFrame(void *_rx, int type) {
	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *) _rx;

	// byteswap the UDP header
	udp_header_ipv4_t *header = (udp_header_ipv4_t *) rx->payload;
	this->packetNetworkToHost(rx);

	// check if any tasks are listening on this port
	for(size_t i = 0; i < UDP::listenPortsEntries; i++) {
		udp_listen_t *listen = &this->listenPorts[i];

		// is the port equal to the port we received the packet on?
		if(listen->valid && listen->port == header->destPort) {
			// if it's unicast, forward it
			if(type == UNICAST) {
				// TODO: handle packet
				this->ipv4->releaseRxBuffer(rx);

				goto handled;
			}
			// if multicast is selected, ensure that the socket receives it
			else if(type == MULTICAST && listen->acceptsMulticast) {
				// TODO: handle packet
				this->ipv4->releaseRxBuffer(rx);

				goto handled;
			}
			// if broadcast is selected, ensure that the socket receives it
			else if(type == BROADCAST && listen->acceptsBroadcast) {
				// TODO: handle packet
				this->ipv4->releaseRxBuffer(rx);

				goto handled;
			}
			// if we get down here, the packet is unhandled
			else {
				goto discard;
			}
		}
	}

discard: ;
	// if we get down here, discard the packet since nobody is handling it
	this->ipv4->releaseRxBuffer(rx);

	// clean-up
handled: ;
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
