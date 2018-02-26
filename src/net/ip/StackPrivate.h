/*
 * StackPrivate.h
 *
 * Various private types used by the IP stack.
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */

#ifndef NET_IP_STACKPRIVATE_H_
#define NET_IP_STACKPRIVATE_H_

#include "StackTypes.h"

#include <cstddef>
#include <cstdint>

/**
 * Defines the various Layer 2 protocols that we handle, based on their
 * EtherType values.
 */
typedef enum {
	// unknown
	kProtocolUnknown							= -1,
	// ARP
	kProtocolARP								= 0x0806,
	// IPv4
	kProtocolIPv4							= 0x0800,
	// IPv6
	kProtocolIPv6							= 0x86DD,
} stack_protocol_l2_t;

/**
 * Defines the various IPv4 Layer 3 protocols that we handle, based on the type
 * field in the IPv4 header.
 */
typedef enum {
	// not applicable/unknown
	kIPv4ProtocolUnknown						= -1,

	// Internet Control Message Protocol
	kIPv4ProtocolICMP						= 0x01,
	// Internet Group Management Protocol
	kIPv4ProtocolIGMP						= 0x02,
	// User Datagram Protocol
	kIPv4ProtocolUDP							= 0x11
} stack_protocol_ipv4_t;



/**
 * Ethernet frame header
 */
typedef struct __attribute__((__packed__)) {
	// destination MAC address
	stack_mac_addr_t macDest;
	// source MAC address
	stack_mac_addr_t macSrc;

	// EtherType
	uint16_t etherType;

	// payload data
	uint8_t payload[];
} stack_802_3_header_t;



/**
 * Encapsulates a packet as it's being parsed. This structure is populated
 * progressively as the packet is parsed, and each protocol header is
 * "popped" from the top of the data.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// length of the packet, in bytes
	size_t length;
	// address of the original received buffer
	void *origBuf;
	// user data passed to the rx function
	uint32_t userData;

	// Ethernet header (offset into origBuf)
	stack_802_3_header_t *ethHeader;
	// is this packet unicast or multicast/broadcast?
	bool unicast;

	// pointer to the first byte of the L2 frame's payload
	void *payload;

	// what protocol is carried by the Ethernet frame?
	stack_protocol_l2_t l2Proto;
	// If the L2 protocol is IP, what IP protocol is being carried?
	stack_protocol_ipv4_t ipv4Proto;
} stack_rx_packet_t;

#pragma GCC diagnostic pop



/**
 * Encapsulates a packet as it's being parsed. This structure is populated
 * progressively as the packet is parsed, and each protocol header is
 * "popped" from the top of the data.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// number of bytes to transmit on the wire
	size_t length;
	// pointer to the tx buffer
	void *txBuffer;

	// length of the payload requested
	size_t payloadLength;
	// start of the payload
	void *payload;

	// Ethernet header (offset into txBuffer)
	stack_802_3_header_t *ethHeader;
} stack_tx_packet_t;

#pragma GCC diagnostic pop

#endif /* NET_IP_STACKPRIVATE_H_ */
