/*
 * UDPPrivate.h
 *
 *  Created on: Feb 26, 2018
 *      Author: tristan
 */

#ifndef NET_IP_UDPPRIVATE_H_
#define NET_IP_UDPPRIVATE_H_

#include <cstdint>

#include "UDPSocket.h"

/**
 * Packet header of an UDP packet.
 */
typedef struct __attribute__((__packed__)) {
	// port from which the packet was sent on the sender
	uint16_t sourcePort;
	// port to which the packet is addressed on this system
	uint16_t destPort;

	// length of the packet, including the UDP header
	uint16_t length;

	// packet checksum: includes an IP pseudo-header
	uint16_t checksum;
} udp_header_ipv4_t;



/**
 * Entries in the listen ports list
 */
#undef udp_listen_t

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// set if this is a valid entry
	bool valid;

	// port we're listening on
	uint16_t port;

	// is this socket accepting multicast?
	uint8_t acceptsMulticast				:1;
	// is this socket accepting broadcast?
	uint8_t acceptsBroadcast				:1;

	// address of the socket
	ip::UDPSocket *sock;
} udp_listen_t;

#pragma GCC diagnostic pop

#endif /* NET_IP_UDPPRIVATE_H_ */
