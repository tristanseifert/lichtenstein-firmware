/*
 * ARPPrivate.h
 *
 * Various private structures for the ARP handler.
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */

#ifndef NET_IP_ARPPRIVATE_H_
#define NET_IP_ARPPRIVATE_H_

#include "StackPrivate.h"

#include <cstdint>

/// Request frame
#define ARP_OP_REQUEST						1
/// Reply frame
#define ARP_OP_REPLY							2

/// hardware type for Ethernet
#define ARP_HW_ETHERNET						1

/**
 * ARP frame
 */
typedef struct __attribute__((__packed__)) {
	// hardware type
	uint16_t hwType;
	// protocol address type (same as EtherType)
	uint16_t protoType;

	// hardware address length, in bytes
	uint8_t hwAddressLength;
	// protocol address length, in bytes
	uint8_t protoAddressLength;

	// operation
	uint16_t op;

	// sender MAC address
	stack_mac_addr_t senderMAC;
	// sender IP address
	stack_ipv4_addr_t senderIP;

	// target MAC address
	stack_mac_addr_t targetMAC;
	// target IP address
	stack_ipv4_addr_t targetIP;
} arp_ipv4_packet_t;



/**
 * A single entry in the ARP cache.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// is this entry valid?
	bool valid;
	// age (ticks)
	uint32_t age;

	// MAC address
	stack_mac_addr_t mac;
	// IPv4 address
	stack_ipv4_addr_t ip;
} arp_ipv4_cache_entry_t;

#pragma GCC diagnostic pop



/**
 * Possible actions for the ARP task.
 */
typedef enum {
	/**
	 * Processes a received ARP message, inserting it into the cache.
	 */
	kARPMessageReceived						= 1,

	/**
	 * Formulates and sends a response to an ARP request for our MAC address.
	 */
	kARPMessageSendReply,
} arp_task_message_type_t;

/**
 * Message passed to the ARP task.
 */
typedef struct {
	// message type
	arp_task_message_type_t type;

	// ARP packet
	arp_ipv4_packet_t packet;
} arp_task_message_t;

#endif /* NET_IP_ARPPRIVATE_H_ */
