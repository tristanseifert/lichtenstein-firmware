/*
 * IPv4Private.h
 *
 *  Created on: Feb 25, 2018
 *      Author: tristan
 */

#ifndef NET_IP_IPV4PRIVATE_H_
#define NET_IP_IPV4PRIVATE_H_

#include "StackTypes.h"
#include "StackPrivate.h"

#include <cstdint>

/// mask to get the version out of the header
#define NET_IPV4_VERSION_MASK				(0xF0)
/// version: IPv4
#define NET_IPV4_VERSION_4					(4)

/// mask to get the header length out of the header
#define NET_IPV4_IHL_MASK					(0x0F)


/// Fragment flags: reserved
#define NET_IPV4_FRAGMENT_RESERVED			(1 << 15)
/// Fragment flags: Don't fragment
#define NET_IPV4_FRAGMENT_DONT				(1 << 14)
/// Fragment flags: More fragments
#define NET_IPV4_FRAGMENT_MORE				(1 << 13)
/// fragment offset mask: this is multiplied by 8 for a byte offset
#define NET_IPV4_FRAGMENT_OFFSET_MASK		(0x1FFF)

/**
 * Defines the structure of an IPv4 packet.
 */
typedef struct __attribute__((__packed__)) {
	// version and header length
	uint8_t version;

	// DSCP and ECN (unused)
	uint8_t dscp;

	// length of packet (header plus data)
	uint16_t length;

	// identification (used for fragments)
	uint16_t id;
	// fragment flags and offset
	uint16_t fragmentFlags;

	// TTL (decremented by one for each hop)
	uint8_t ttl;
	// IPv4 protocol (see stack_protocol_ipv4_t)
	uint8_t protocol;
	// header checksum (see https://en.wikipedia.org/wiki/IPv4_header_checksum)
	uint16_t headerChecksum;

	// source IP address
	stack_ipv4_addr_t source;
	// destination IP address
	stack_ipv4_addr_t dest;

	// options (iff IHL > 4)
	uint8_t options[];
} net_ipv4_packet_t;



/**
 * IPv4 transmit buffer; this encapsulates a stack TX buffer and adds a small
 * pointer to the IPv4 packet and the packet's payload.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// stack transmit buffer
	stack_tx_packet_t *stackBuffer;

	// pointer to the IPv4 header
	net_ipv4_packet_t *ipv4Header;

	// pointer to payload (directly after ipv4 header)
	void *payload;
	// length of payload
	size_t payloadLength;
} stack_ipv4_tx_packet_t;

#pragma GCC diagnostic pop

#endif /* NET_IP_IPV4PRIVATE_H_ */
